#include "audio_engine.h"
#include "sequencer.h"
#include <math.h>
#include <string.h>

static Voice voices[MAX_VOICES];
static float sine_table[SINE_SAMPLES];
volatile SoundEngine current_engine = ENGINE_PIANO;
volatile float master_volume = 0.8f;

typedef struct {
    float attack;
    float decay;
    float sustain;
    float release;
} ADSR;

static const ADSR ENGINE_ADSR[] = {
    [ENGINE_PIANO]   = {0.08f, 0.002f, 0.3f, 0.005f}, // Sharp Attack
    [ENGINE_E_PIANO] = {0.04f, 0.001f, 0.6f, 0.003f}, // Mellow
    [ENGINE_STRINGS] = {0.001f, 0.0005f, 0.9f, 0.002f} // Very slow build-up
};

static float soft_clip(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x * (1.5f - 0.5f * x * x);
}

void AudioEngine_Init(void) {
    for (int i = 0; i < SINE_SAMPLES; i++) sine_table[i] = sinf(2.0f * M_PI * i / SINE_SAMPLES);
    memset(voices, 0, sizeof(voices));
}

void AudioEngine_NoteOn(uint8_t midi_note, float velocity) {
    Sequencer_RecordEvent(midi_note, velocity, 1);
    for (int i = 0; i < MAX_VOICES; i++) {
        if (!voices[i].active || voices[i].env_state == STATE_OFF) {
            voices[i].midi_note = midi_note;
            float freq = 440.0f * powf(2.0f, (midi_note - 69) / 12.0f);
            voices[i].phase_step = (freq * SINE_SAMPLES) / SAMPLING_RATE;
            voices[i].phase = 0; voices[i].amplitude = 0;
            voices[i].target_amplitude = velocity;
            voices[i].env_state = STATE_ATTACK;
            voices[i].active = 1;
            return;
        }
    }
}

void AudioEngine_NoteOff(uint8_t midi_note) {
    Sequencer_RecordEvent(midi_note, 0, 0);
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active && voices[i].midi_note == midi_note) voices[i].env_state = STATE_RELEASE;
    }
}

void AudioEngine_SetEngine(SoundEngine engine) { current_engine = engine; }
void AudioEngine_SetVolume(float volume) { master_volume = (volume > 0.8f) ? 0.8f : volume; }

void AudioEngine_Process(uint16_t *buffer, uint32_t start_idx, uint32_t size) {
    // Current engine is volatile, fetched once per block for consistency
    SoundEngine mode = current_engine;
    ADSR p = ENGINE_ADSR[mode];
    
    for (uint32_t i = 0; i < size; i++) {
        float mix = 0;
        for (int v = 0; v < MAX_VOICES; v++) {
            if (!voices[v].active) continue;
            
            float sample = 0;
            int ph = (int)voices[v].phase;
            
            // --- WAVE SYNTHESIS PER MODE ---
            if (mode == ENGINE_PIANO) {
                sample += sine_table[ph % SINE_SAMPLES];
                sample += 0.4f * sine_table[(ph * 2) % SINE_SAMPLES]; // Brighter
                sample += 0.2f * sine_table[(ph * 3) % SINE_SAMPLES];
            } else if (mode == ENGINE_E_PIANO) {
                sample += sine_table[ph % SINE_SAMPLES];
                sample += 0.3f * sine_table[(ph * 4) % SINE_SAMPLES]; // Bell-like tink
            } else if (mode == ENGINE_STRINGS) {
                // Approximate a Sawtooth for "Strings" richness
                sample += (2.0f * (voices[v].phase / SINE_SAMPLES)) - 1.0f;
                sample *= 0.5f; // Scale down sawtooth
            }

            // --- ADSR LOGIC ---
            switch (voices[v].env_state) {
                case STATE_ATTACK:
                    voices[v].amplitude += p.attack;
                    if (voices[v].amplitude >= voices[v].target_amplitude) {
                        voices[v].amplitude = voices[v].target_amplitude;
                        voices[v].env_state = STATE_DECAY;
                    }
                    break;
                case STATE_DECAY:
                    voices[v].amplitude -= p.decay;
                    if (voices[v].amplitude <= p.sustain * voices[v].target_amplitude) {
                        voices[v].amplitude = p.sustain * voices[v].target_amplitude;
                        voices[v].env_state = STATE_SUSTAIN;
                    }
                    break;
                case STATE_RELEASE:
                    voices[v].amplitude -= p.release;
                    if (voices[v].amplitude <= 0) { voices[v].amplitude = 0; voices[v].env_state = STATE_OFF; voices[v].active = 0; }
                    break;
                default: break;
            }

            mix += sample * voices[v].amplitude;
            voices[v].phase += voices[v].phase_step;
            if (voices[v].phase >= SINE_SAMPLES) voices[v].phase -= SINE_SAMPLES;
        }
        mix *= master_volume * 0.20f;
        mix = soft_clip(mix);
        buffer[start_idx + i] = (uint16_t)((mix + 1.0f) * 32767.0f);
    }
}
