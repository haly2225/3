/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 Oscilloscope - DMA Double Buffering (Ping-Pong)
  *
  * CONTINUOUS STREAMING - NO DEAD TIME
  * ====================================
  * This version uses DMA Circular Mode with Half/Complete callbacks
  * to achieve continuous ADC sampling without any gaps.
  *
  * How it works:
  * - ADC uses a double-sized buffer (1024 samples)
  * - DMA runs in CIRCULAR mode (never stops)
  * - Half Complete IRQ: First 512 samples ready → pack & send
  * - Complete IRQ: Last 512 samples ready → pack & send
  * - While sending one half, ADC continues filling the other half
  * - Result: ZERO dead time, continuous data stream
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <string.h>

/* ============================================================================
 * DOUBLE BUFFERING CONFIGURATION
 * ============================================================================ */
#define HALF_BUFFER_SIZE     512       // Samples per half buffer
#define FULL_BUFFER_SIZE     1024      // Total samples (double buffer)
#define TX_BYTES             (HALF_BUFFER_SIZE * 2 + 4)  // Header + data

/* Peripheral handles */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
TIM_HandleTypeDef htim2;

/* ============================================================================
 * PING-PONG BUFFERS
 * ============================================================================ */
// ADC double buffer - DMA fills this continuously in circular mode
uint16_t adc_buffer[FULL_BUFFER_SIZE] __attribute__((aligned(4)));

// Two TX buffers for ping-pong transmission
uint8_t tx_buffer_A[TX_BYTES] __attribute__((aligned(4)));
uint8_t tx_buffer_B[TX_BYTES] __attribute__((aligned(4)));

// Pointers for ping-pong
volatile uint8_t* current_tx_buffer = NULL;
volatile uint8_t active_buffer = 0;  // 0 = A, 1 = B

// Frame counter for packet sequencing
volatile uint16_t frame_counter = 0;

// Status flags
volatile uint8_t tx_busy = 0;         // SPI transmission in progress
volatile uint8_t half_ready = 0;      // First half of buffer ready
volatile uint8_t full_ready = 0;      // Second half of buffer ready

// Debug counters
volatile uint32_t half_irq_count = 0;
volatile uint32_t full_irq_count = 0;
volatile uint32_t tx_complete_count = 0;
volatile uint32_t missed_frames = 0;  // Frames dropped due to SPI busy

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);

/* ============================================================================
 * PACK BUFFER - Convert ADC data to TX format
 * ============================================================================ */
void pack_buffer(uint16_t* src, uint8_t* dst)
{
    // Header: 0xAA 0x55 + frame counter (big endian)
    dst[0] = 0xAA;
    dst[1] = 0x55;
    dst[2] = (frame_counter >> 8) & 0xFF;
    dst[3] = frame_counter & 0xFF;

    // Pack 512 ADC samples as big-endian 16-bit values
    for (uint16_t i = 0; i < HALF_BUFFER_SIZE; i++) {
        uint16_t val = src[i];
        dst[4 + i*2]     = (val >> 8) & 0xFF;  // High byte
        dst[4 + i*2 + 1] = val & 0xFF;          // Low byte
    }

    frame_counter++;
}

/* ============================================================================
 * START SPI TRANSMISSION (if not busy)
 * ============================================================================ */
void start_spi_tx(uint8_t* buffer)
{
    if (tx_busy) {
        // SPI still sending previous buffer - skip this frame
        missed_frames++;
        return;
    }

    tx_busy = 1;
    HAL_SPI_Transmit_DMA(&hspi1, buffer, TX_BYTES);
}

/* ============================================================================
 * DMA CALLBACKS - PING-PONG LOGIC
 * ============================================================================ */

/**
 * @brief  Half Transfer Complete callback
 *         Called when DMA fills first half of adc_buffer (samples 0-511)
 *         At this point, ADC is filling second half (samples 512-1023)
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    half_irq_count++;

    // Pack first half (samples 0-511) into available TX buffer
    if (active_buffer == 0) {
        pack_buffer(&adc_buffer[0], tx_buffer_A);
        start_spi_tx(tx_buffer_A);
        active_buffer = 1;  // Next time use buffer B
    } else {
        pack_buffer(&adc_buffer[0], tx_buffer_B);
        start_spi_tx(tx_buffer_B);
        active_buffer = 0;  // Next time use buffer A
    }
}

/**
 * @brief  Transfer Complete callback
 *         Called when DMA fills second half of adc_buffer (samples 512-1023)
 *         At this point, DMA wraps around to fill first half again
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    full_irq_count++;

    // Pack second half (samples 512-1023) into available TX buffer
    if (active_buffer == 0) {
        pack_buffer(&adc_buffer[HALF_BUFFER_SIZE], tx_buffer_A);
        start_spi_tx(tx_buffer_A);
        active_buffer = 1;
    } else {
        pack_buffer(&adc_buffer[HALF_BUFFER_SIZE], tx_buffer_B);
        start_spi_tx(tx_buffer_B);
        active_buffer = 0;
    }
}

/**
 * @brief  SPI TX Complete callback
 *         Called when DMA finishes sending TX buffer
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    tx_complete_count++;
    tx_busy = 0;  // Ready for next transmission
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();

    // Start PWM output (test signal on PA1)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // Calibrate ADC
    HAL_ADCEx_Calibration_Start(&hadc1);

    // Startup LED blink (3 times)
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    // Clear buffers
    memset(adc_buffer, 0, sizeof(adc_buffer));
    memset(tx_buffer_A, 0, sizeof(tx_buffer_A));
    memset(tx_buffer_B, 0, sizeof(tx_buffer_B));

    // =========================================================================
    // START CONTINUOUS ADC WITH DMA CIRCULAR MODE
    // =========================================================================
    // This starts the ADC and DMA in circular mode.
    // The DMA will continuously fill adc_buffer, wrapping around when full.
    // Half Complete and Complete IRQs trigger the ping-pong transmission.
    //
    // ADC never stops! Zero dead time!
    // =========================================================================
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FULL_BUFFER_SIZE);

    // Main loop - just LED heartbeat and debug
    uint32_t last_led_toggle = 0;
    uint8_t led_state = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // LED heartbeat every 500ms
        if (now - last_led_toggle >= 500) {
            last_led_toggle = now;
            led_state = !led_state;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        // Main loop is now free - all data handling is done in IRQ callbacks
        // This improves timing consistency
    }
}

/* ============================================================================
 * SYSTEM CLOCK CONFIGURATION - 64MHz
 * ============================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    // HSI (8MHz) / 2 * 16 = 64MHz
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    // ADC Clock: 64MHz / 6 = 10.67MHz
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* ============================================================================
 * ADC1 INITIALIZATION - Continuous Mode with DMA
 * ============================================================================ */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;        // CRITICAL: Continuous mode
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Channel 8 (PB0) with balanced sampling
    // ADC clock = 10.67MHz, with 28.5 cycles: 10.67MHz / (12.5 + 28.5) = ~260kSps
    // This gives Pi enough time to read data before buffer overrun
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;  // ~260kHz sample rate (balanced for SPI)

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ============================================================================
 * SPI1 INITIALIZATION - Slave Mode
 * ============================================================================ */
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

/* ============================================================================
 * TIM2 INITIALIZATION - 10kHz PWM Test Signal
 * ============================================================================ */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 64MHz / (71+1) / (99+1) = 8.89kHz (~10kHz PWM)
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 50% duty cycle
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 50;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

/* ============================================================================
 * DMA INITIALIZATION
 * ============================================================================ */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    // ADC DMA - High priority (continuous sampling is critical)
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // SPI TX DMA - Lower priority
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/* ============================================================================
 * GPIO INITIALIZATION
 * ============================================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // LED on PC13
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ============================================================================
 * ERROR HANDLER
 * ============================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        // Fast LED blink indicates error
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile int i = 0; i < 100000; i++);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
