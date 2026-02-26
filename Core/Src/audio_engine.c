#include "audio_engine.h"
#include <math.h>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

int16_t sine_wave[SINE_SAMPLES];
Voice voices[MAX_VOICES];

void AudioEngine_Init(void) {
    // Prepare Sine Wave Table (Additive Synthesis)
    for (int i = 0; i < SINE_SAMPLES; i++) {
        float angle = i * 2.0f * (float)M_PI / SINE_SAMPLES;
        float sample = sinf(angle)
                     + 0.5f * sinf(2.0f * angle)
                     + 0.25f * sinf(3.0f * angle)
                     + 0.125f * sinf(4.0f * angle);
        sine_wave[i] = (int16_t)(sample / 2.0f * 16383.0f);
    }
    
    // Initialize voices
    for(int v=0; v<MAX_VOICES; v++) voices[v].active = 0;
}

void AudioEngine_Process(uint16_t *buffer, uint32_t start_idx, uint32_t size) {
    for (uint32_t i = 0; i < size; i += 2) {
        float mix = 0;
        for (int v = 0; v < MAX_VOICES; v++) {
            if (voices[v].active) {
                mix += (float)sine_wave[(int)voices[v].phase] * voices[v].amplitude;
                
                voices[v].phase += voices[v].phase_step;
                if (voices[v].phase >= SINE_SAMPLES) voices[v].phase -= SINE_SAMPLES;
                
                voices[v].amplitude *= 0.99998f; // Decay
                if (voices[v].amplitude < 0.005f) voices[v].active = 0;
            }
        }
        int16_t sample = (int16_t)(mix / 2.0f);
        buffer[start_idx + i] = (uint16_t)sample;
        buffer[start_idx + i + 1] = (uint16_t)sample;
    }
}

void AudioEngine_NoteOn(uint8_t midi_note, float velocity, uint8_t row, uint8_t col) {
    // Find a free voice
    for (int v = 0; v < MAX_VOICES; v++) {
        if (!voices[v].active) {
            voices[v].active = 1;
            voices[v].amplitude = velocity;
            voices[v].row = row;
            voices[v].col = col;
            voices[v].phase = 0;
            
            float freq = 440.0f * powf(2.0f, (midi_note - 69.0f) / 12.0f);
            voices[v].phase_step = (freq * SINE_SAMPLES) / SAMPLING_RATE;
            break;
        }
    }
}

void AudioEngine_NoteOff(uint8_t row, uint8_t col) {
    for (int v = 0; v < MAX_VOICES; v++) {
        if (voices[v].active && voices[v].row == row && voices[v].col == col) {
            voices[v].amplitude *= 0.1f; // Fast decay
            break;
        }
    }
}
