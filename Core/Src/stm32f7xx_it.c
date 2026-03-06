/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi2_tx;
extern I2S_HandleTypeDef hi2s2;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream4 global interrupt (I2S TX).
  */
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

/**
  * @brief This function handles SPI2 global interrupt (I2S).
  */
void SPI2_IRQHandler(void)
{
  HAL_I2S_IRQHandler(&hi2s2);
}

/**
  * @brief This function handles USART2 global interrupt (MIDI/ESP32).
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
