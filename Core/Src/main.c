#include "main.h"
#include "audio_engine.h"
#include "tft_driver.h"
#include "tft_gfx.h"
#include "ui_controller.h"
#include "touch_driver.h"
#include "keyboard_handler.h"
#include "keyboard_matrix.h"
#include "sequencer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Defines */
#define AUDIO_BUF_SIZE 1024

/* Peripheral Handles */
ADC_HandleTypeDef hadc1;
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;

/* Global Variables */
uint16_t audio_buffer[AUDIO_BUF_SIZE];
int last_drawn_note = -1;
volatile uint8_t log_note = 0;
volatile uint8_t log_type = 0;

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
void UART_Log(char* msg);

/* Callbacks */
void UART_Log(char* msg) { HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg), 10); }
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) { AudioEngine_Process(audio_buffer, 0, AUDIO_BUF_SIZE / 2); }
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) { AudioEngine_Process(audio_buffer, AUDIO_BUF_SIZE / 2, AUDIO_BUF_SIZE / 2); }

int main(void) {
  HAL_Init(); // Basic init first
  HAL_Delay(500); // --- CRITICAL: Wait for power to stabilize ---
  
  SCB_EnableICache(); 
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  UART_Log("\r\n--- PIANO SYSTEM BOOT ---\r\n");
  
  __HAL_SPI_ENABLE(&hspi1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET); 
  
  TFT_Init(); 
  Touch_Init();
  Keyboard_Init();
  Sequencer_Init();
  
  AudioEngine_Init(); 
  AudioEngine_SetVolume(0.8f);
  AudioEngine_SetEngine(ENGINE_PIANO);
  
  memset(audio_buffer, 0, sizeof(audio_buffer));
  HAL_I2S_Transmit_DMA(&hi2s2, audio_buffer, AUDIO_BUF_SIZE);
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
  Draw_PlayUI(); 
  UART_Log("READY.\r\n");

  // Initialize button state to actual current state before loop to prevent false trigger
  static uint8_t last_btn_state;
  last_btn_state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);

  while (1) {
    // Keyboard_Scan(); // --- TEMPORARILY DISABLED TO TEST TOUCH ---
    UI_ProcessTouch();
    Sequencer_Update();
    // UI_UpdatePlayerProgress();
    // UI_UpdateRecordingTimer();

    if (Touch_IsPressed()) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    // --- Blue Button (USER_Btn) Audio Test Logic ---
    static uint8_t last_btn_state = 0;
    uint8_t current_btn_state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
    
    if (current_btn_state != last_btn_state) {
        if (current_btn_state) {
            AudioEngine_NoteOn(60, 0.8f); // Play C4
            UART_Log("TEST: Note On C4\r\n");
        } else {
            AudioEngine_NoteOff(60);      // Stop C4
            UART_Log("TEST: Note Off C4\r\n");
        }
        last_btn_state = current_btn_state;
    }
    
    // --- VR (Potentiometer) Volume Control ---
    static uint32_t last_adc_time = 0;
    static uint32_t last_adc_val = 0;
    if (HAL_GetTick() - last_adc_time > 100) {
        last_adc_time = HAL_GetTick();
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
            int32_t diff = (int32_t)adc_val - (int32_t)last_adc_val;
            
            // Only update if change is significant (>50) to avoid noise
            if (diff > 50 || diff < -50) {
                float new_vol = ((float)adc_val / 4095.0f) * 0.8f;
                if (new_vol < 0.05f) new_vol = 0.0f;
                
                master_volume = new_vol;
                AudioEngine_SetVolume(new_vol);
                Draw_VolumeBar(new_vol);
                last_adc_val = adc_val;
            }
        }
    }

    HAL_Delay(1);
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  HAL_PWREx_EnableOverDrive();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

static void MX_I2S2_Init(void) {
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  HAL_I2S_Init(&hi2s2);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_spi2_tx.Instance = DMA1_Stream4;
  hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
  hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
  hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_spi2_tx);
  __HAL_LINKDMA(&hi2s2, hdmatx, hdma_spi2_tx);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  HAL_SPI_Init(&hspi1);
  SET_BIT(SPI1->CR2, SPI_CR2_FRXTH); 
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart3);
}

static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_3; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // Matrix Setup
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_SET);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_5; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_4; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_2; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_4; HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13; HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // TFT & LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_7, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Force Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_14; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_13; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1); }
