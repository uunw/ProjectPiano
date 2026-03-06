#ifndef __AUDIO_ENGINE_H
#define __AUDIO_ENGINE_H

#include "main.h"

#define SINE_SAMPLES 256
#define MAX_VOICES 16
#define AUDIO_BUF_SIZE 1024
#define SAMPLING_RATE 48000

typedef enum {
    STATE_OFF,
    STATE_ATTACK,
    STATE_DECAY,
    STATE_SUSTAIN,
    STATE_RELEASE
} EnvelopeState;

typedef enum {
    ENGINE_PIANO,
    ENGINE_ORGAN,
    ENGINE_SYNTH
} SoundEngine;

typedef struct {
    uint8_t active;
    float phase;
    float phase_step;
    float amplitude;
    float target_amplitude;
    uint8_t midi_note;
    EnvelopeState env_state;
} Voice;

void AudioEngine_Init(void);
void AudioEngine_Process(uint16_t *buffer, uint32_t start_idx, uint32_t size);
void AudioEngine_NoteOn(uint8_t midi_note, float velocity);
void AudioEngine_NoteOff(uint8_t midi_note);
void AudioEngine_SetVolume(float volume);
void AudioEngine_SetEngine(SoundEngine engine);

#endif /* __AUDIO_ENGINE_H */
