/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SINE_SAMPLES 256
#define MAX_VOICES 8
#define AUDIO_BUF_SIZE 512
#define SAMPLING_RATE 48000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

ETH_HandleTypeDef heth;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
int16_t sine_wave[SINE_SAMPLES];

// Force pusher and buffer into SRAM to avoid DTCM (which DMA cannot access)
uint8_t dtcm_pusher[128 * 1024] __attribute__((section(".data")));
uint16_t audio_buffer[AUDIO_BUF_SIZE] __attribute__((section(".data"))); 

volatile uint32_t dac_callback_count = 0;

// Correct Matrix to MIDI Note Mapping for AKAI MPK mini MK3
// Based on discovery log pressing Left to Right.
// Note: 0 means no key assigned.
const uint8_t note_map[8][8] = {
    {62, 63, 56, 57, 50, 51,  0,  0}, // Row 0 (S1)
    {62, 63, 56, 57, 50, 51,  0,  0}, // Row 1 (S2)
    {64, 65, 58, 59, 52, 53,  0,  0}, // Row 2 (S1)
    {64, 65, 58, 59, 52, 53,  0,  0}, // Row 3 (S2)
    {66, 67, 60, 61, 54, 55, 48, 49}, // Row 4 (S1)
    {66, 67, 60, 61, 54, 55, 48, 49}, // Row 5 (S2)
    {72,  0, 70, 71, 68, 69,  0,  0}, // Row 6 (S1)
    {72,  0, 70, 71, 68, 69,  0,  0}  // Row 7 (S2)
};

typedef struct {
    uint8_t active;
    float phase;
    float phase_step;
    float amplitude;      // Current amplitude (0.0 to 1.0)
    uint8_t row;
    uint8_t col;
} Voice;

Voice voices[MAX_VOICES];
uint32_t s1_timestamp[4][8] = {0}; // Add S1 timing for Velocity
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
// static void MX_ETH_Init(void); // Conflict with PE0, PE1
static void MX_USART3_UART_Init(void);
// static void MX_USB_OTG_FS_PCD_Init(void); // Potential conflicts
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
void Prepare_Sine_Wave(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Prepare_Sine_Wave(void) {
  for (int i = 0; i < SINE_SAMPLES; i++) {
    float angle = i * 2.0f * (float)M_PI / SINE_SAMPLES;
    // Additive synthesis: Fundamental + 2nd Harmonic + 3rd Harmonic + 4th Harmonic
    // Piano has many overtones.
    float sample = sinf(angle)
                 + 0.5f * sinf(2.0f * angle)
                 + 0.25f * sinf(3.0f * angle)
                 + 0.125f * sinf(4.0f * angle);

    // Normalize and scale to signed 16-bit (-16383 to 16383)
    // The sum of 1 + 0.5 + 0.25 + 0.125 = 1.875, so we divide by ~2.0 to be safe.
    sine_wave[i] = (int16_t)(sample / 2.0f * 16383.0f);
  }
}

void Update_Audio_Buffer(uint32_t start_idx, uint32_t size) {
    dac_callback_count++;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // LD1 Green

    // Check Blue User Button (PC13)
    uint8_t test_mode = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
    if (test_mode) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    static uint32_t square_phase = 0;

    for (uint32_t i = 0; i < size; i += 2) {
        if (test_mode) {
            int16_t val = (square_phase < 24) ? 15000 : -15000;
            audio_buffer[start_idx + i] = (uint16_t)val;
            audio_buffer[start_idx + i + 1] = (uint16_t)val;
            square_phase = (square_phase + 1) % 48;
        } else {
            float mix = 0;
            for (int v = 0; v < MAX_VOICES; v++) {
                if (voices[v].active) {
                    mix += (float)sine_wave[(int)voices[v].phase] * voices[v].amplitude;
                    
                    voices[v].phase += voices[v].phase_step;
                    if (voices[v].phase >= SINE_SAMPLES) voices[v].phase -= SINE_SAMPLES;
                    
                    // Slower decay for testing: 0.99998f
                    voices[v].amplitude *= 0.99998f;
                    if (voices[v].amplitude < 0.005f) voices[v].active = 0;
                }
            }
            // Increase Gain: Divide by 2 instead of MAX_VOICES
            int16_t sample = (int16_t)(mix / 2.0f);
            audio_buffer[start_idx + i] = (uint16_t)sample;
            audio_buffer[start_idx + i + 1] = (uint16_t)sample;
        }
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    Update_Audio_Buffer(0, AUDIO_BUF_SIZE / 2);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    Update_Audio_Buffer(AUDIO_BUF_SIZE / 2, AUDIO_BUF_SIZE / 2);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Enable DWT Cycle Counter for high-resolution timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  // MX_ETH_Init(); // Disabled: Pin Conflict
  MX_USART3_UART_Init();
  // MX_USB_OTG_FS_PCD_Init(); // Disabled: Pin Conflict
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
  char msg[128];
  
  // Diagnostic: Print buffer address
  int len = sprintf(msg, "Audio Buffer Address: 0x%08X\r\n", (unsigned int)audio_buffer);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);

  Prepare_Sine_Wave();
  
  // Verify Sine Table is not all zeros
  len = sprintf(msg, "Sine Table Debug: %d, %d, %d, %d, %d\r\n", 
                sine_wave[0], sine_wave[1], sine_wave[2], sine_wave[3], sine_wave[4]);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);

  // Initialize voices
  for(int v=0; v<MAX_VOICES; v++) voices[v].active = 0;
  for(int b=0; b<AUDIO_BUF_SIZE; b++) audio_buffer[b] = 0;

  // Start I2S DMA
  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)audio_buffer, AUDIO_BUF_SIZE);
  
  HAL_UART_Transmit(&huart3, (uint8_t*)"--- Project Piano Ready with Velocity ---\r\n", 43, 100);
  
  uint8_t key_state[8][8] = {0};
  uint8_t active_keys = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_debug = 0;
  while (1)
  {
    // Every 5 seconds, print detailed diagnostic status
    if (HAL_GetTick() - last_debug > 5000) {
        int d_state = (hi2s2.hdmatx != NULL) ? (int)HAL_DMA_GetState(hi2s2.hdmatx) : -1;
        uint32_t d_err = (hi2s2.hdmatx != NULL) ? hi2s2.hdmatx->ErrorCode : 0;

        int len = sprintf(msg, "Callbacks: %lu | Voices Active: %d | I2S State: %d\r\n", 
                          dac_callback_count, active_keys, (int)HAL_I2S_GetState(&hi2s2));
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
        last_debug = HAL_GetTick();
    }

    for (int r = 0; r < 8; r++)
    {
      HAL_GPIO_WritePin(GPIOE, (uint16_t)(1 << r), GPIO_PIN_SET);
      HAL_Delay(1); // Stabilization delay for Ribbon Cable

      for (int c = 0; c < 8; c++)
      {
        GPIO_PinState state = HAL_GPIO_ReadPin(GPIOF, (uint16_t)(1 << c));

        if (state == GPIO_PIN_SET && key_state[r][c] == 0)
        {
          key_state[r][c] = 1;
          
          uint8_t midi_note = note_map[r][c];
          if (midi_note == 0) continue;

          if (r % 2 == 0) { // S1 (Even Rows) - Record Start Time
            s1_timestamp[r/2][c] = DWT->CYCCNT;
          } else { // S2 (Odd Rows) - Calculate Velocity and Trigger Note
            uint32_t current_time = DWT->CYCCNT;
            uint32_t dt = current_time - s1_timestamp[r/2][c];
            
            // Convert cycles to ms: SystemClock is 216MHz
            float ms = (float)dt / 216000.0f;
            
            // Velocity Mapping: < 5ms = 127, > 50ms = 1
            int vel = 127 - (int)((ms - 5.0f) * (126.0f / 45.0f));
            if (vel > 127) vel = 127;
            if (vel < 1) vel = 1;

            active_keys++;
            // Find a free voice
            for (int v = 0; v < MAX_VOICES; v++) {
              if (!voices[v].active) {
                  voices[v].active = 1;
                  voices[v].amplitude = (float)vel / 127.0f; // Velocity scales amplitude
                  voices[v].row = r;
                  voices[v].col = c;
                  voices[v].phase = 0;
                  
                  // MIDI Note to Frequency: f = 440 * 2^((n-69)/12)
                  float freq = 440.0f * powf(2.0f, (midi_note - 69.0f) / 12.0f);
                  voices[v].phase_step = (freq * SINE_SAMPLES) / SAMPLING_RATE;

                  int len = sprintf(msg, "[Key On] Note:%d Vel:%d dT:%.2fms\r\n", midi_note, vel, ms);
                  HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 50);
                  break;
              }
            }
          }
        }
        else if (state == GPIO_PIN_RESET && key_state[r][c] == 1)
        {
          key_state[r][c] = 0;
          
          if (r % 2 == 1) // S2 Release
          {
            uint8_t midi_note = note_map[r][c];
            if (active_keys > 0) active_keys--;

            for (int v = 0; v < MAX_VOICES; v++) {
              if (voices[v].active && voices[v].row == r && voices[v].col == c) {
                  voices[v].amplitude *= 0.1f; // Fast decay on release
                  break;
              }
            }
          }
        }
      }
      HAL_GPIO_WritePin(GPIOE, (uint16_t)(1 << r), GPIO_PIN_RESET);
    }
    HAL_Delay(2); // Reduced main loop delay for faster scanning
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2249;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
