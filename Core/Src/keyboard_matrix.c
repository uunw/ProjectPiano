#include "keyboard_matrix.h"
#include "audio_engine.h"
#include "ui_controller.h"
#include <stdio.h>

extern void UART_Log(char* msg);

// --- PIN MAPPING (INSIDE PINS) ---
static GPIO_TypeDef* ROW_PORTS[] = {GPIOC, GPIOB, GPIOB, GPIOA, GPIOC, GPIOB, GPIOB, GPIOA};
static uint16_t ROW_PINS[] = {GPIO_PIN_6, GPIO_PIN_15, GPIO_PIN_13, GPIO_PIN_15, GPIO_PIN_7, GPIO_PIN_5, GPIO_PIN_3, GPIO_PIN_4};

static GPIO_TypeDef* COL_PORTS[] = {GPIOB, GPIOC, GPIOF, GPIOB, GPIOB, GPIOD, GPIOD, GPIOD};
static uint16_t COL_PINS[] = {GPIO_PIN_4, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_2, GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_11};

/**
 * Note map configured for working keys (Left to Right)
 * Skips broken hardware keys: 2, 3, 12, 19, 20, 25
 */
const uint8_t note_map[8][8] = {
    {68, 69, 70, 71,  0,  0,  0,  0}, // Row 0 (S1) - High notes
    {68, 69, 70, 71,  0,  0,  0,  0}, // Row 1 (S2) - High notes
    {60, 61, 62, 63, 64, 65,  0,  0}, // Row 2 (S1) - Mid-High notes
    {60, 61, 62, 63, 64, 65,  0,  0}, // Row 3 (S2) - Mid-High notes
    {52, 53, 54, 55, 56, 57, 58,  0}, // Row 4 (S1) - Mid-Low notes
    {52, 53, 54, 55, 56, 57, 58,  0}, // Row 5 (S2) - Mid-Low notes
    { 0,  0,  0,  0, 48, 51,  0,  0}, // Row 6 (S1) - Low notes (C3, Eb3)
    { 0,  0,  0,  0, 48, 51,  0,  0}  // Row 7 (S2) - Low notes
};

// State tracking variables
static uint8_t key_pressed_s1[4][8] = {0}; // 0=Idle, 1=S1_Pressed, 2=S2_Triggered
static uint32_t s1_timestamp[4][8] = {0};
static uint8_t note_active[128] = {0};

void Keyboard_Init(void) {
    // GPIO Config is handled in main.c (MX_GPIO_Init)
}

void Keyboard_Scan(void) {
    uint32_t now = HAL_GetTick();
    
    for (int r = 0; r < MATRIX_ROWS; r++) {
        // Drive Row LOW
        HAL_GPIO_WritePin(ROW_PORTS[r], ROW_PINS[r], GPIO_PIN_RESET);
        for(volatile int i=0; i<50; i++); // Signal stabilization
        
        for (int c = 0; c < MATRIX_COLS; c++) {
            uint8_t note = note_map[r][c];
            if (note == 0) continue;
            
            int is_pressed = (HAL_GPIO_ReadPin(COL_PORTS[c], COL_PINS[c]) == GPIO_PIN_RESET);
            int group = r / 2;
            int is_s2_row = (r % 2 == 1);
            
            if (is_pressed) {
                if (!is_s2_row) { // S1 Contact (Initial)
                    if (key_pressed_s1[group][c] == 0) {
                        key_pressed_s1[group][c] = 1;
                        s1_timestamp[group][c] = now;
                    }
                } else { // S2 Contact (Trigger)
                    if (key_pressed_s1[group][c] == 1 && note_active[note] == 0) {
                        uint32_t dt = now - s1_timestamp[group][c];
                        float velocity = (dt > 1) ? (1.0f - ((float)dt / 120.0f)) : 1.0f;
                        if (velocity < 0.15f) velocity = 0.15f;
                        
                        AudioEngine_NoteOn(note, velocity);
                        Update_NoteUI(note);
                        note_active[note] = 1;
                        key_pressed_s1[group][c] = 2;
                        
                        // Debug Log
                        char buf[32]; sprintf(buf, "NOTE ON: %d\r\n", note); UART_Log(buf);
                    }
                }
            } else {
                if (!is_s2_row && key_pressed_s1[group][c] != 0) {
                    key_pressed_s1[group][c] = 0;
                    if (note_active[note]) {
                        AudioEngine_NoteOff(note);
                        note_active[note] = 0;
                        Update_NoteUI(-1);
                        
                        char buf[32]; sprintf(buf, "NOTE OFF: %d\r\n", note); UART_Log(buf);
                    }
                }
            }
        }
        // Drive Row HIGH
        HAL_GPIO_WritePin(ROW_PORTS[r], ROW_PINS[r], GPIO_PIN_SET);
    }
}
