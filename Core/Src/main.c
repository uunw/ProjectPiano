#include "main.h"
#include "audio_engine.h"
#include <string.h>
#include <stdio.h>

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint16_t audio_buffer[AUDIO_BUF_SIZE];
uint8_t uart_rx_byte;
uint8_t esp_handshake_done = 0;
uint32_t last_esp_heartbeat = 0;
volatile uint32_t i2s_callback_count = 0;
uint8_t i2s_error_detected = 0;

// Queue สำหรับเก็บโน้ตที่ต้อง Log (เพื่อไม่ให้ขวาง Interrupt)
volatile uint8_t log_note = 0;
volatile uint8_t log_type = 0; // 1: ON, 2: OFF

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

void UART_Log(char* msg) {
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
}

void Silence_All_Notes(void) {
    for(int i=0; i<127; i++) AudioEngine_NoteOff(i);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uint8_t data = uart_rx_byte;
        if (data == 0xBB) { esp_handshake_done = 1; last_esp_heartbeat = HAL_GetTick(); }
        else if (data == 0xDD) { last_esp_heartbeat = HAL_GetTick(); }
        
        static uint8_t midi_state = 0;
        static uint8_t midi_note;

        if (data >= 0x80) {
            uint8_t cmd = data & 0xF0;
            if (cmd == 0x90) midi_state = 1; 
            else if (cmd == 0x80) midi_state = 4;
            else if (cmd == 0xB0) midi_state = 7;
            else if (cmd == 0xC0) midi_state = 10;
            else midi_state = 0;
        } else {
            switch(midi_state) {
                case 1: midi_note = data; midi_state = 2; break;
                case 2: 
                    if (data > 0) { AudioEngine_NoteOn(midi_note, (float)data / 127.0f); log_note = midi_note; log_type = 1; }
                    else { AudioEngine_NoteOff(midi_note); log_note = midi_note; log_type = 2; }
                    midi_state = 1; break;
                case 4: midi_note = data; midi_state = 5; break;
                case 5: AudioEngine_NoteOff(midi_note); log_note = midi_note; log_type = 2; midi_state = 4; break;
                case 7: if (data == 7) midi_state = 8; else midi_state = 0; break;
                case 8: AudioEngine_SetVolume((float)data / 127.0f); midi_state = 7; break;
                case 10: if (data <= 2) AudioEngine_SetEngine((SoundEngine)data); midi_state = 0; break;
            }
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) { 
    i2s_callback_count++;
    AudioEngine_Process(audio_buffer, 0, AUDIO_BUF_SIZE / 2); 
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) { 
    i2s_callback_count++;
    AudioEngine_Process(audio_buffer, AUDIO_BUF_SIZE / 2, AUDIO_BUF_SIZE / 2); 
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

int main(void) {
    HAL_Init(); SystemClock_Config(); MX_GPIO_Init(); MX_DMA_Init(); MX_I2S2_Init(); MX_USART2_UART_Init(); MX_USART3_UART_Init();
    AudioEngine_Init(); memset(audio_buffer, 0, sizeof(audio_buffer));
    
    UART_Log("\r\n--- PIANO ENGINE READY ---\r\n");
    AudioEngine_SetVolume(0.8f);

    if (HAL_I2S_Transmit_DMA(&hi2s2, audio_buffer, AUDIO_BUF_SIZE) == HAL_OK) {
        UART_Log("STM32: Audio DMA Started.\r\n");
    }

    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    uint32_t last_log = 0;
    uint8_t hb = 0xAA;
    uint8_t connect_cmd = 0xCC;
    uint8_t btn_state = 0;

    while (1) {
        // --- Standalone Audio Test (Blue Button PC13) ---
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
            if (btn_state == 0) {
                AudioEngine_NoteOn(60, 0.8f); // C4
                UART_Log("STM32: Manual Test Note ON (Button)\r\n");
                btn_state = 1;
            }
        } else {
            if (btn_state == 1) {
                AudioEngine_NoteOff(60);
                UART_Log("STM32: Manual Test Note OFF (Button)\r\n");
                btn_state = 0;
            }
        }

        // --- Deferred Logging ---
        if (log_type > 0) {
            char buf[32];
            sprintf(buf, "[MIDI] %s: %d\r\n", (log_type == 1) ? "ON " : "OFF", log_note);
            UART_Log(buf);
            log_type = 0; // Reset
        }

        if (HAL_GetTick() - last_log > 1000) {
            if (!esp_handshake_done) {
                HAL_UART_Transmit(&huart2, &connect_cmd, 1, 10);
            } else {
                if (HAL_GetTick() - last_esp_heartbeat > 3000) {
                    Silence_All_Notes();
                    esp_handshake_done = 0; 
                } else {
                    HAL_UART_Transmit(&huart2, &hb, 1, 10);
                    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
                }
            }
            last_log = HAL_GetTick();
        }
    }
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2; huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_USART3_UART_Init(void) {
    huart3.Instance = USART3; huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B; huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE; huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

static void MX_I2S2_Init(void) {
    hi2s2.Instance = SPI2; hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS; hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE; hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW; hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    HAL_I2S_Init(&hi2s2);
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0); HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4; RCC_OscInitStruct.PLL.PLLN = 216; RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2; HAL_RCC_OscConfig(&RCC_OscInitStruct); HAL_PWREx_EnableOverDrive();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192; PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S; 
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
