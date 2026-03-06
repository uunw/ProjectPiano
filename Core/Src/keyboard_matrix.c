/**
 * @file keyboard_matrix.c
 * @brief [DEPRECATED] Keypad scanning moved to ESP32.
 * This file is kept for reference only.
 */
#include "keyboard_matrix.h"
#include "audio_engine.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

// note_map moved to ESP32 side
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

uint8_t key_state[8][8] = {0};
uint32_t s1_timestamp[4][8] = {0};

void Keyboard_Init(void) {
    // Keyboard initialization moved to ESP32
}

void Keyboard_Scan(void) {
    // Legacy scan loop removed. 
    // Now handled by ESP32 via UART commands (0x90, 0x80).
}
