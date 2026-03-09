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
    [ENGINE_PIANO]   = {0.1f, 0.005f, 0.3f, 0.01f}, 
    [ENGINE_E_PIANO] = {0.05f, 0.002f, 0.6f, 0.005f},
    [ENGINE_STRINGS] = {0.002f, 0.001f, 0.9f, 0.002f}
};

/**
 * @brief Cubic Soft Clipper (Approximation of tanh)
 * Input: -1.0 to 1.0, Output: -0.66 to 0.66
 */
static float soft_clip(float x) {
    if (x >= 1.0f) return 0.666f;
    if (x <= -1.0f) return -0.666f;
    return x - ((x * x * x) * 0.333333f);
}

void AudioEngine_Init(void) {
    for (int i = 0; i < SINE_SAMPLES; i++) sine_table[i] = sinf(2.0f * M_PI * i / SINE_SAMPLES);
    memset(voices, 0, sizeof(voices));
}

void AudioEngine_NoteOn(uint8_t midi_note, float velocity) {
    Sequencer_RecordEvent(midi_note, velocity, 1);
    for (int v = 0; v < MAX_VOICES; v++) {
        if (!voices[v].active || voices[v].env_state == STATE_OFF) {
            voices[v].midi_note = midi_note;
            float freq = 440.0f * powf(2.0f, (midi_note - 69) / 12.0f);
            voices[v].phase_step = (freq * SINE_SAMPLES) / SAMPLING_RATE;
            voices[v].phase = 0; 
            voices[v].amplitude = 0;
            voices[v].target_amplitude = velocity;
            voices[v].env_state = STATE_ATTACK;
            voices[v].active = 1;
            return;
        }
    }
}

void AudioEngine_NoteOff(uint8_t midi_note) {
    Sequencer_RecordEvent(midi_note, 0, 0);
    for (int v = 0; v < MAX_VOICES; v++) {
        if (voices[v].active && voices[v].midi_note == midi_note) voices[v].env_state = STATE_RELEASE;
    }
}

void AudioEngine_SetEngine(SoundEngine engine) { current_engine = engine; }
void AudioEngine_SetVolume(float volume) { master_volume = (volume > 0.8f) ? 0.8f : volume; }

void AudioEngine_Process(uint16_t *buffer, uint32_t start_idx, uint32_t size) {
    SoundEngine mode = current_engine;
    ADSR p = ENGINE_ADSR[mode];
    int16_t *signed_buffer = (int16_t*)buffer; // Use signed pointer for I2S
    
    for (uint32_t i = 0; i < size; i++) {
        float mix = 0;
        int active_count = 0;
        
        for (int v = 0; v < MAX_VOICES; v++) {
            if (!voices[v].active) continue;
            active_count++;
            
            float sample = 0;
            int ph = (int)voices[v].phase;
            
            // --- SCALED WAVE SYNTHESIS ---
            if (mode == ENGINE_PIANO) {
                sample += sine_table[ph % SINE_SAMPLES] * 0.7f;
                sample += sine_table[(ph * 2) % SINE_SAMPLES] * 0.2f; // Lower harmonics
                sample += sine_table[(ph * 3) % SINE_SAMPLES] * 0.1f;
            } else if (mode == ENGINE_E_PIANO) {
                sample += sine_table[ph % SINE_SAMPLES] * 0.8f;
                sample += sine_table[(ph * 4) % SINE_SAMPLES] * 0.2f;
            } else if (mode == ENGINE_STRINGS) {
                sample += (2.0f * (voices[v].phase / SINE_SAMPLES)) - 1.0f;
                sample *= 0.4f; // Scale down Sawtooth
            }

            // --- ADSR ---
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
                    if (voices[v].amplitude <= 0) {
                        voices[v].amplitude = 0;
                        voices[v].env_state = STATE_OFF;
                        voices[v].active = 0;
                    }
                    break;
                default: break;
            }

            mix += sample * voices[v].amplitude;
            voices[v].phase += voices[v].phase_step;
            if (voices[v].phase >= SINE_SAMPLES) voices[v].phase -= SINE_SAMPLES;
        }

        // --- POLYPHONIC MIXING ---
        if (active_count > 1) {
            mix *= 1.0f / sqrtf((float)active_count);
        }
        
        // Final Output Gain (Safe scaling)
        mix *= master_volume * 0.8f; 
        
        // Cubic Saturation
        float final_sample = soft_clip(mix);
        
        // Convert to 16-bit Signed PCM (-32768 to 32767)
        signed_buffer[start_idx + i] = (int16_t)(final_sample * 32767.0f);
    }
}
