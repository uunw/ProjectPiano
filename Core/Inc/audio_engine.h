#ifndef __AUDIO_ENGINE_H
#define __AUDIO_ENGINE_H

#include "main.h"

#define SINE_SAMPLES 256
#define MAX_VOICES 8
#define AUDIO_BUF_SIZE 512
#define SAMPLING_RATE 48000

typedef struct {
    uint8_t active;
    float phase;
    float phase_step;
    float amplitude;
    uint8_t row;
    uint8_t col;
} Voice;

void AudioEngine_Init(void);
void AudioEngine_Process(uint16_t *buffer, uint32_t start_idx, uint32_t size);
void AudioEngine_NoteOn(uint8_t midi_note, float velocity, uint8_t row, uint8_t col);
void AudioEngine_NoteOff(uint8_t row, uint8_t col);

#endif /* __AUDIO_ENGINE_H */
