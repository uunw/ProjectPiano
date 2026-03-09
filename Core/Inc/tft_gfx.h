#ifndef TFT_GFX_H
#define TFT_GFX_H

#include "main.h"

#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// Arctic Light Theme Colors
#define COLOR_BG         0xFFFF  // White
#define COLOR_PANEL      0xEF7D  // Light Grey-Blue
#define COLOR_TEXT_MAIN  0x2104  // Dark Grey
#define COLOR_TEXT_DIM   0x8410  // Medium Grey
#define COLOR_ACCENT     0x0410  // Deep Blue
#define COLOR_NOTE       0x051F  // Piano Blue
#define COLOR_VOL_BASE   0xD6BA  // Muted Grey
#define COLOR_VOL_LOW    0x4E2A  // Greenish
#define COLOR_VOL_MID    0xF6A0  // Orangeish
#define COLOR_VOL_HIGH   0xF800  // Red
#define COLOR_BTN_BG     0xDEFB  // Very Light Blue
#define COLOR_BTN_TEXT   0x2104  // Dark Grey (Main text color)

void GFX_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void GFX_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void GFX_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void GFX_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
void GFX_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size);
const char* GFX_NoteToName(int note);

#endif
