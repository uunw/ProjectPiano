#ifndef TOUCH_DRIVER_H
#define TOUCH_DRIVER_H

#include "main.h"

// --- XPT2046 Commands ---
#define CMD_X_POS  0xD0
#define CMD_Y_POS  0x90

// --- Calibration Values (Derived from ESP32 project) ---
#define TS_MIN_X 350
#define TS_MAX_X 3550
#define TS_MIN_Y 380
#define TS_MAX_Y 3250

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t pressed;
} TouchPoint;

void Touch_Init(void);
uint8_t Touch_Read(TouchPoint* p);
uint8_t Touch_ReadRaw(uint16_t* x, uint16_t* y);
uint8_t Touch_IsPressed(void);

#endif
