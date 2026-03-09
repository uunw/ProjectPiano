#include "touch_driver.h"
#include "tft_driver.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern void UART_Log(char* msg);

// --- Private Helpers ---
static uint16_t Touch_ReadValue(uint8_t cmd) {
    uint8_t tx[3] = {cmd, 0x00, 0x00};
    uint8_t rx[3] = {0};
    
    TCS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
    TCS_HIGH();
    
    uint16_t val = ((uint16_t)rx[1] << 8) | rx[2];
    return val >> 3; // 12-bit result
}

void Touch_Init(void) {
    TCS_HIGH();
    Touch_ReadValue(0x80); // Wake up & Enable IRQ mode
}

uint8_t Touch_IsPressed(void) {
    // T_IRQ (PF14) is Active Low
    return (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14) == GPIO_PIN_RESET);
}

uint8_t Touch_ReadRaw(uint16_t* x, uint16_t* y) {
    if (!Touch_IsPressed()) return 0;
    
    uint32_t tx = 0, ty = 0;
    for (int i = 0; i < 16; i++) {
        ty += Touch_ReadValue(0x90);
        tx += Touch_ReadValue(0xD0);
    }
    *x = tx / 16;
    *y = ty / 16;
    return 1;
}

uint8_t Touch_Read(TouchPoint* p) {
    uint16_t rx, ry;
    if (!Touch_ReadRaw(&rx, &ry)) {
        p->pressed = 0;
        return 0;
    }
    
    // --- 4-POINT CALIBRATED MAPPING (From User Data) ---
    // P1 (Top-Left):     rx=3238, ry=3648
    // P2 (Top-Right):    rx=3245, ry=511
    // P3 (Bottom-Right): rx=687,  ry=564
    // P4 (Bottom-Left):  rx=891,  ry=3218
    
    // Calculate X (Uses ry: 3648 -> 0, 511 -> 319)
    int32_t x_map = (3648 - (int32_t)ry) * 320 / (3648 - 511);
    
    // Calculate Y (Uses rx: 3245 -> 0, 687 -> 239)
    int32_t y_map = (3245 - (int32_t)rx) * 240 / (3245 - 687);

    // Clamp values
    if (x_map < 0) x_map = 0;
    if (x_map > 319) x_map = 319;
    if (y_map < 0) y_map = 0;
    if (y_map > 239) y_map = 239;

    p->x = (uint16_t)x_map;
    p->y = (uint16_t)y_map;
    p->pressed = 1;
    
    return 1;
}
