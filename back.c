#include <Arduino.h>

#define LED_PIN PC13

// Protocol: [START] [LENGTH] [DATA...] [CHECKSUM]
#define START_MARKER 0xAA
#define STOP_MARKER  0x55

volatile uint8_t message[] = "hello pi4";
volatile uint8_t messageLen = 9;

// Transmission state machine
typedef enum {
  TX_START,
  TX_LENGTH,
  TX_DATA,
  TX_CHECKSUM,
  TX_STOP
} TxState_t;

volatile TxState_t txState = TX_START;
volatile uint8_t txIndex = 0;
volatile uint8_t txChecksum = 0;

// Status
volatile bool spiActive = false;
volatile uint32_t lastTransferTime = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Enable clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;
  
  // GPIO Configuration
  // PA5 SCK - Input floating
  GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
  GPIOA->CRL |= GPIO_CRL_CNF5_0;
  
  // PA6 MISO - AF push-pull 50MHz
  GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
  GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1;
  
  // PA7 MOSI - Input floating
  GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
  GPIOA->CRL |= GPIO_CRL_CNF7_0;
  
  // PA4 NSS - Input floating
  GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
  GPIOA->CRL |= GPIO_CRL_CNF4_0;
  
  // Reset SPI
  RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
  
  // Configure SPI1 as Slave
  SPI1->CR1 = 0;
  SPI1->CR1 &= ~SPI_CR1_MSTR;
  SPI1->CR1 &= ~SPI_CR1_CPOL;
  SPI1->CR1 &= ~SPI_CR1_CPHA;
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
  SPI1->CR1 &= ~SPI_CR1_SSM;
  SPI1->CR1 |= SPI_CR1_SSI;
  
  // Interrupts
  SPI1->CR2 = 0;
  SPI1->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE;
  
  // Calculate checksum
  txChecksum = 0;
  for(int i = 0; i < messageLen; i++) {
    txChecksum ^= message[i];
  }
  
  // Load START marker
  SPI1->DR = START_MARKER;
  txState = TX_LENGTH;
  
  // Enable SPI
  SPI1->CR1 |= SPI_CR1_SPE;
  
  if (!(SPI1->CR1 & SPI_CR1_SPE)) {
    digitalWrite(LED_PIN, LOW);
    while(1) delay(1000);
  }
  
  NVIC_SetPriority(SPI1_IRQn, 0);
  NVIC_EnableIRQ(SPI1_IRQn);
  
  // Startup blinks
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(300);
    digitalWrite(LED_PIN, HIGH);
    delay(300);
  }
  
  lastTransferTime = millis();
}

void loop() {
  uint32_t now = millis();
  
  // LED status
  if (now - lastTransferTime < 2000) {
    spiActive = true;
    if ((now / 50) % 2 == 0) {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
  } else {
    spiActive = false;
    if ((now / 500) % 2 == 0) {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  
  // Error handling
  if (SPI1->SR & (SPI_SR_OVR | SPI_SR_MODF)) {
    volatile uint8_t dummy = SPI1->DR;
    volatile uint16_t status = SPI1->SR;
    (void)dummy; (void)status;
    
    txState = TX_START;
    txIndex = 0;
    SPI1->DR = START_MARKER;
  }
  
  delay(10);
}

extern "C" void SPI1_IRQHandler(void) {
  uint16_t sr = SPI1->SR;
  
  lastTransferTime = millis();
  
  // Handle errors
  if (sr & (SPI_SR_OVR | SPI_SR_MODF)) {
    volatile uint8_t dummy = SPI1->DR;
    volatile uint16_t status = SPI1->SR;
    (void)dummy; (void)status;
    
    txState = TX_START;
    txIndex = 0;
    return;
  }
  
  // Read received data
  if (sr & SPI_SR_RXNE) {
    volatile uint8_t dummy = SPI1->DR;
    (void)dummy;
  }
  
  // Send next byte
  if (sr & SPI_SR_TXE) {
    uint8_t nextByte = 0x00;
    
    switch(txState) {
      case TX_START:
        nextByte = START_MARKER;
        txState = TX_LENGTH;
        break;
        
      case TX_LENGTH:
        nextByte = messageLen;
        txState = TX_DATA;
        txIndex = 0;
        break;
        
      case TX_DATA:
        nextByte = message[txIndex++];
        if (txIndex >= messageLen) {
          txState = TX_CHECKSUM;
        }
        break;
        
      case TX_CHECKSUM:
        nextByte = txChecksum;
        txState = TX_STOP;
        break;
        
      case TX_STOP:
        nextByte = STOP_MARKER;
        txState = TX_START;
        break;
    }
    
    SPI1->DR = nextByte;
  }
}