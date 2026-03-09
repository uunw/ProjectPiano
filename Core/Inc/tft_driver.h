#ifndef TFT_DRIVER_H
#define TFT_DRIVER_H

#include "main.h"

// --- Fast Direct Register Access Macros ---
#define CS_HIGH()   GPIOD->BSRR = GPIO_PIN_14
#define CS_LOW()    GPIOD->BSRR = (uint32_t)GPIO_PIN_14 << 16
#define DC_HIGH()   GPIOD->BSRR = GPIO_PIN_15
#define DC_LOW()    GPIOD->BSRR = (uint32_t)GPIO_PIN_15 << 16
#define TCS_HIGH()  GPIOF->BSRR = GPIO_PIN_13
#define TCS_LOW()   GPIOF->BSRR = (uint32_t)GPIO_PIN_13 << 16

void TFT_Init(void);
void TFT_WriteCommand(uint8_t cmd);
void TFT_WriteData(uint8_t data);
uint32_t TFT_ReadID(void);
void FastSPI_SendByte(uint8_t data);

#endif
