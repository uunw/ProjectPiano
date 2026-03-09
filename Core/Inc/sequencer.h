#ifndef __SEQUENCER_H
#define __SEQUENCER_H

#include "main.h"

#define MAX_EVENTS 2000
#define MAX_SLOTS 4
#define MAX_RECORD_TIME 180000 // 3 Minutes in ms

typedef struct {
    uint8_t note;
    float velocity;
    uint32_t timestamp;
    uint8_t type; // 1=On, 0=Off
} MidiEvent;

typedef enum {
    SEQ_IDLE,
    SEQ_RECORDING,
    SEQ_PLAYING,
    SEQ_PAUSED
} SeqState;

typedef struct {
    MidiEvent events[MAX_EVENTS];
    uint32_t event_count;
    uint32_t duration;
} RecordingSlot;

void Sequencer_Init(void);
void Sequencer_StartRecord(uint8_t slot);
void Sequencer_StopRecord(void);
void Sequencer_RecordEvent(uint8_t note, float velocity, uint8_t type);

void Sequencer_StartPlay(uint8_t slot);
void Sequencer_PausePlay(void);
void Sequencer_ResumePlay(void);
void Sequencer_StopPlay(void);
void Sequencer_Update(void);

uint32_t Sequencer_GetProgress(void); // 0-100
uint32_t Sequencer_GetDuration(uint8_t slot);
uint32_t Sequencer_GetCurrentTime(void); // Get current elapsed time in ms
SeqState Sequencer_GetState(void);
uint8_t Sequencer_GetActiveSlot(void);
uint32_t Sequencer_GetEventCount(uint8_t slot);

#endif
