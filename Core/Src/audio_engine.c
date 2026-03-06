#include "audio_engine.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

int16_t wave_table[SINE_SAMPLES];
Voice voices[MAX_VOICES];
float master_volume = 0.8f;

// Micro Fade Parameters (เพื่อป้องกันเสียงดังแป๊กเวลาปล่อยคีย์)
// 0.05f ใช้เวลาประมาณ 5ms ในการขึ้น/ลง ซึ่งเร็วมากจนหูฟังไม่ออกว่าหน่วง แต่ถนอมลำโพง
const float RAMP_SPEED = 0.05f; 

// Simple Wave Generation (Pure Sine + Small Harmonics for clarity)
void generate_wave(SoundEngine engine) {
    for (int i = 0; i < SINE_SAMPLES; i++) {
        float angle = i * 2.0f * (float)M_PI / SINE_SAMPLES;
        // Pure sine with a hint of 2nd harmonic to make it easier to hear on small speakers
        float sample = sinf(angle) + 0.2f * sinf(2.0f * angle);
        wave_table[i] = (int16_t)(sample / 1.2f * 25000.0f);
    }
}

void AudioEngine_Init(void) {
    generate_wave(ENGINE_PIANO);
    for(int v=0; v<MAX_VOICES; v++) {
        voices[v].active = 0;
        voices[v].phase = 0;
        voices[v].phase_step = 0;
        voices[v].amplitude = 0;
        voices[v].midi_note = 0;
        voices[v].env_state = STATE_OFF;
    }
    master_volume = 0.8f;
}

void AudioEngine_Process(uint16_t *buffer, uint32_t start_idx, uint32_t size) {
    for (uint32_t i = 0; i < size; i += 2) {
        float mix = 0;
        int active_voices = 0;

        for (int v = 0; v < MAX_VOICES; v++) {
            if (voices[v].active) {
                active_voices++;
                
                // --- Micro Envelope (Anti-Clicking) ---
                if (voices[v].env_state == STATE_ATTACK) {
                    voices[v].amplitude += RAMP_SPEED;
                    if (voices[v].amplitude >= 1.0f) {
                        voices[v].amplitude = 1.0f;
                        voices[v].env_state = STATE_SUSTAIN;
                    }
                } else if (voices[v].env_state == STATE_RELEASE) {
                    voices[v].amplitude -= RAMP_SPEED;
                    if (voices[v].amplitude <= 0.0f) {
                        voices[v].amplitude = 0.0f;
                        voices[v].active = 0; // ปิด Voice ทิ้งเมื่อ Fade จบ
                        voices[v].env_state = STATE_OFF;
                    }
                }

                mix += (float)wave_table[(int)voices[v].phase] * voices[v].amplitude;
                
                voices[v].phase += voices[v].phase_step;
                if (voices[v].phase >= SINE_SAMPLES) voices[v].phase -= SINE_SAMPLES;
            }
        }
        
        // Anti-clipping: Scale volume based on number of notes
        float current_gain = master_volume;
        if (active_voices > 2) current_gain *= (2.0f / (float)active_voices);

        int32_t sample = (int32_t)(mix * current_gain);
        
        // Hard Limiter for Safety
        if (sample > 32760) sample = 32760;
        if (sample < -32760) sample = -32760;

        buffer[start_idx + i] = (uint16_t)sample;     // Left
        buffer[start_idx + i + 1] = (uint16_t)sample; // Right
    }
}

void AudioEngine_NoteOn(uint8_t midi_note, float velocity) {
    // If note is already playing, re-trigger Attack
    for (int v = 0; v < MAX_VOICES; v++) {
        if (voices[v].active && voices[v].midi_note == midi_note) {
            voices[v].env_state = STATE_ATTACK;
            return;
        }
    }
    // Assign to new voice
    for (int v = 0; v < MAX_VOICES; v++) {
        if (!voices[v].active) {
            voices[v].active = 1;
            voices[v].midi_note = midi_note;
            voices[v].phase = 0;
            voices[v].amplitude = 0.0f; // Start from 0
            voices[v].env_state = STATE_ATTACK;
            
            float freq = 440.0f * powf(2.0f, (midi_note - 69.0f) / 12.0f);
            voices[v].phase_step = (freq * SINE_SAMPLES) / SAMPLING_RATE;
            break;
        }
    }
}

void AudioEngine_NoteOff(uint8_t midi_note) {
    for (int v = 0; v < MAX_VOICES; v++) {
        if (voices[v].active && voices[v].midi_note == midi_note) {
            // แทนที่จะดับทันที (Instant = 0) ให้เข้าสู่โหมด Release (Fade out เร็วๆ)
            voices[v].env_state = STATE_RELEASE;
        }
    }
}

void AudioEngine_SetVolume(float volume) { master_volume = volume; }
void AudioEngine_SetEngine(SoundEngine engine) { /* Not used in simple mode */ }
