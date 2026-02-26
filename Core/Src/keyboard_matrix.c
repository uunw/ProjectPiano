#include "keyboard_matrix.h"
#include "audio_engine.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

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
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Keyboard_Scan(void) {
    char msg[64];
    for (int r = 0; r < 8; r++) {
        HAL_GPIO_WritePin(GPIOE, (uint16_t)(1 << r), GPIO_PIN_SET);
        HAL_Delay(1); // Stabilization delay

        for (int c = 0; c < 8; c++) {
            GPIO_PinState state = HAL_GPIO_ReadPin(GPIOF, (uint16_t)(1 << c));

            if (state == GPIO_PIN_SET && key_state[r][c] == 0) {
                key_state[r][c] = 1;
                uint8_t midi_note = note_map[r][c];
                if (midi_note == 0) continue;

                if (r % 2 == 0) { // S1
                    s1_timestamp[r/2][c] = DWT->CYCCNT;
                } else { // S2
                    uint32_t current_time = DWT->CYCCNT;
                    uint32_t dt = current_time - s1_timestamp[r/2][c];
                    float ms = (float)dt / 216000.0f;
                    int vel = 127 - (int)((ms - 5.0f) * (126.0f / 45.0f));
                    if (vel > 127) vel = 127;
                    if (vel < 1) vel = 1;

                    AudioEngine_NoteOn(midi_note, (float)vel / 127.0f, r, c);
                    
                    int len = sprintf(msg, "[Key On] Note:%d Vel:%d dT:%.2fms\r\n", midi_note, vel, ms);
                    HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 50);
                }
            } else if (state == GPIO_PIN_RESET && key_state[r][c] == 1) {
                key_state[r][c] = 0;
                if (r % 2 == 1) {
                    AudioEngine_NoteOff(r, c);
                }
            }
        }
        HAL_GPIO_WritePin(GPIOE, (uint16_t)(1 << r), GPIO_PIN_RESET);
    }
}
