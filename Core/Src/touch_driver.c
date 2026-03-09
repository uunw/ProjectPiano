#include "touch_driver.h"
#include "tft_driver.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern void UART_Log(char* msg);

// --- Bit-bang SPI Helpers ---
static void Touch_WriteBit(uint8_t bit) {
    if (bit) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // MOSI
    else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    
    for(volatile int i=0; i<20; i++); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // SCK High
    for(volatile int i=0; i<40; i++); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // SCK Low
    for(volatile int i=0; i<40; i++);
}

static uint8_t Touch_ReadBit(void) {
    uint8_t bit = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // SCK High
    for(volatile int i=0; i<40; i++);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) bit = 1;     // MISO
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // SCK Low
    for(volatile int i=0; i<40; i++);
    return bit;
}

static uint16_t Touch_ReadValue(uint8_t cmd) {
    // --- Switch Pins to GPIO Mode for Bit-bang ---
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // SCK
    
    GPIO_InitStruct.Pin = GPIO_PIN_7; HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); // MOSI

    TCS_LOW();
    for(volatile int i=0; i<50; i++);

    // Send Command (8-bit)
    for (int i = 7; i >= 0; i--) Touch_WriteBit((cmd >> i) & 1);
    
    // Wait for conversion
    for(volatile int i=0; i<200; i++);

    // Read Data (12-bit)
    uint16_t val = 0;
    // อ่าน Dummy bit ก่อน
    Touch_ReadBit();
    
    for (int i = 11; i >= 0; i--) {
        if (Touch_ReadBit()) val |= (1 << i);
    }
    TCS_HIGH();
    for(volatile int i=0; i<50; i++);

    // --- Switch Pins back to SPI Alternate Function for TFT ---
    GPIO_InitStruct.Pin = GPIO_PIN_5; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_7; HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    return val;
}

void Touch_Init(void) {
    TCS_HIGH();
    Touch_ReadValue(0x80); // Wake up
}

uint8_t Touch_IsPressed(void) {
    return (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14) == GPIO_PIN_RESET);
}

uint8_t Touch_ReadRaw(uint16_t* x, uint16_t* y) {
    if (!Touch_IsPressed()) return 0;
    
    uint32_t tx = 0, ty = 0;
    for (int i = 0; i < 8; i++) {
        tx += Touch_ReadValue(0xD0); // X
        ty += Touch_ReadValue(0x90); // Y
    }
    *x = tx / 8;
    *y = ty / 8;
    return 1;
}

uint8_t Touch_Read(TouchPoint* p) {
    uint16_t rx, ry;
    if (!Touch_ReadRaw(&rx, &ry)) {
        p->pressed = 0;
        return 0;
    }
    
    // Calibration for Portrait/Landscape (Adjust based on display rotation)
    // ใช้ค่าจาก Log เดิมที่เคยใช้งานได้:
    // rx: 3600(Top) -> 400(Bottom), ry: 3600(Left) -> 400(Right)
    int32_t x_map = (3600 - (int32_t)rx) * 320 / (3600 - 400);
    int32_t y_map = (3600 - (int32_t)ry) * 240 / (3600 - 400);

    if (x_map < 0) x_map = 0; if (x_map > 319) x_map = 319;
    if (y_map < 0) y_map = 0; if (y_map > 239) y_map = 239;

    p->x = (uint16_t)x_map;
    p->y = (uint16_t)y_map;
    p->pressed = 1;
    
    static uint32_t last_log = 0;
    if (HAL_GetTick() - last_log > 200) {
        char buf[64];
        sprintf(buf, "BITBANG: Raw(%u,%u) -> Map(%u,%u)\r\n", rx, ry, p->x, p->y);
        UART_Log(buf);
        last_log = HAL_GetTick();
    }
    return 1;
}
