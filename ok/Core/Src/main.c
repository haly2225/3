/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hybrid Oscilloscope - VanAn ADC + SPI Slave for Raspberry Pi
  ******************************************************************************
  * Based on: Bui Van An (vanan92) - DIY Oscilloscope 1.2.2
  * Modified for: Raspberry Pi 4 display via SPI slave transmission
  *
  * Features:
  * - Dual ADC @ 1 MSPS/channel (ADC1: PA2, ADC2: PA4)
  * - 80MHz overclock for better performance
  * - Timer-triggered ADC sampling (TIM3)
  * - DMA circular buffer
  * - SPI slave transmission to Raspberry Pi
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define NUM_SAMPLES          2048
#define PACKET_HEADER_SIZE   4
#define TX_BUFFER_SIZE       (PACKET_HEADER_SIZE + NUM_SAMPLES * 2 * 2)  // Header + dual channel (2 bytes each)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
TIM_HandleTypeDef htim3;

uint32_t adc_buffer[NUM_SAMPLES] __attribute__((aligned(4)));  // 32-bit: lower=ADC1, upper=ADC2
uint16_t ch1Capture[NUM_SAMPLES];   // ADC1 data
uint16_t ch2Capture[NUM_SAMPLES];   // ADC2 data
uint8_t  tx_buffer[TX_BUFFER_SIZE] __attribute__((aligned(4)));

volatile uint16_t frame_counter = 0;
volatile uint8_t conversion_ready = 0;
volatile uint8_t data_ready_for_spi = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);

/* Pack dual channel data into SPI transmission buffer */
void pack_dual_channel_buffer(void)
{
  // Header: 0xAA 0x55 + frame counter
  tx_buffer[0] = 0xAA;
  tx_buffer[1] = 0x55;
  tx_buffer[2] = (frame_counter >> 8) & 0xFF;
  tx_buffer[3] = frame_counter & 0xFF;

  // Interleaved dual channel data: CH1[0] CH2[0] CH1[1] CH2[1] ...
  for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
    uint16_t ch1_val = ch1Capture[i];
    uint16_t ch2_val = ch2Capture[i];

    tx_buffer[4 + i*4 + 0] = (ch1_val >> 8) & 0xFF;  // CH1 high byte
    tx_buffer[4 + i*4 + 1] = ch1_val & 0xFF;         // CH1 low byte
    tx_buffer[4 + i*4 + 2] = (ch2_val >> 8) & 0xFF;  // CH2 high byte
    tx_buffer[4 + i*4 + 3] = ch2_val & 0xFF;         // CH2 low byte
  }

  frame_counter++;
}

/* Separate dual ADC data from DMA buffer */
void extract_dual_adc_data(void)
{
  for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
    ch1Capture[i] = adc_buffer[i] & 0xFFFF;         // Lower 16-bit: ADC1
    ch2Capture[i] = (adc_buffer[i] >> 16) & 0xFFFF; // Upper 16-bit: ADC2
  }
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick */
  HAL_Init();

  /* Configure the system clock to 80MHz (overclocked) */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();

  /* ADC Calibration */
  LL_ADC_Enable(ADC1);
  LL_ADC_StartCalibration(ADC1);
  while (LL_ADC_IsCalibrationOnGoing(ADC1));
  LL_mDelay(10);

  LL_ADC_Enable(ADC2);
  LL_ADC_StartCalibration(ADC2);
  while (LL_ADC_IsCalibrationOnGoing(ADC2));
  LL_mDelay(10);

  /* Startup LED blink */
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(100);
  }

  /* Initialize buffers */
  memset(ch1Capture, 0, sizeof(ch1Capture));
  memset(ch2Capture, 0, sizeof(ch2Capture));
  pack_dual_channel_buffer();

  /* Start SPI DMA transmission */
  HAL_SPI_Transmit_DMA(&hspi1, tx_buffer, TX_BUFFER_SIZE);
  HAL_Delay(10);

  /* Configure DMA for dual ADC */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&adc_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_SAMPLES);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Start ADC with external trigger (TIM3) */
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
  LL_TIM_EnableCounter(TIM3);

  /* LED on to indicate running */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* Infinite loop */
  while (1)
  {
    if (conversion_ready) {
      conversion_ready = 0;

      /* Extract dual channel data from DMA buffer */
      extract_dual_adc_data();

      /* Pack into SPI transmission buffer */
      pack_dual_channel_buffer();

      /* Restart DMA for next acquisition */
      LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&adc_buffer);
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_SAMPLES);
      LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

      /* Start ADC again */
      LL_TIM_EnableCounter(TIM3);

      /* Toggle LED to show activity */
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
}

/**
  * @brief System Clock Configuration - 80MHz Overclock
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2);

  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_10);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  LL_SetSystemCoreClock(80000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
  * @brief ADC1 Initialization - Dual mode master
  */
static void MX_ADC1_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /* ADC1 GPIO Configuration: PA2 -> ADC1_IN2 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* ADC Configuration */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  /* Dual mode: simultaneous regular conversion */
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /* Regular channel configuration */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  /* Channel 2 configuration */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
}

/**
  * @brief ADC2 Initialization - Dual mode slave
  */
static void MX_ADC2_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /* ADC2 GPIO Configuration: PA4 -> ADC2_IN4 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC Configuration */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);

  /* Regular channel configuration */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

  /* Channel 4 configuration */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_1CYCLE_5);
}

/**
  * @brief SPI1 Initialization - Slave mode for Raspberry Pi
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;

  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization - ADC trigger @ ~200kHz (adjustable)
  */
static void MX_TIM3_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /*
   * Timer configuration for ADC trigger:
   * Clock: 80MHz / (Prescaler+1) / (AutoReload+1)
   * Example: 80MHz / 1 / 400 = 200kHz per channel
   * For 1 MSPS: use Prescaler=0, AutoReload=79
   */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 399;  // 200kHz trigger rate
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
}

/**
  * @brief DMA Initialization
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA1_Channel1 (ADC1) interrupt init */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* DMA1_Channel3 (SPI1_TX) interrupt init */
  NVIC_SetPriority(DMA1_Channel3_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* LED pin PC13 */
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for (volatile int i = 0; i < 100000; i++);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add implementation to report error */
}
#endif
