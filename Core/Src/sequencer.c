#include "sequencer.h"
#include "audio_engine.h"
#include <string.h>

#define FLASH_STORAGE_ADDR 0x081C0000 

static RecordingSlot library[MAX_SLOTS];
static uint8_t active_slot = 0;
static SeqState current_state = SEQ_IDLE;
static uint32_t start_time = 0;
static uint32_t pause_time = 0;
static uint32_t play_ptr = 0;

void Sequencer_SaveToFlash(void) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit = {FLASH_TYPEERASE_SECTORS, 0, FLASH_SECTOR_11, 1, FLASH_VOLTAGE_RANGE_3};
    uint32_t sectorError = 0;
    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK) {
        uint32_t *ptr = (uint32_t*)&library;
        for (uint32_t i = 0; i < sizeof(library)/4; i++) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + (i*4), ptr[i]);
        }
    }
    HAL_FLASH_Lock();
}

void Sequencer_LoadFromFlash(void) {
    memcpy(&library, (void*)FLASH_STORAGE_ADDR, sizeof(library));
    if (library[0].event_count == 0xFFFFFFFF) memset(&library, 0, sizeof(library));
}

void Sequencer_Init(void) { Sequencer_LoadFromFlash(); }

void Sequencer_StartRecord(uint8_t slot) {
    if (slot >= MAX_SLOTS) return;
    active_slot = slot;
    library[slot].event_count = 0;
    library[slot].duration = 0;
    start_time = HAL_GetTick();
    current_state = SEQ_RECORDING;
}

void Sequencer_StopRecord(void) {
    if (current_state == SEQ_RECORDING) {
        library[active_slot].duration = HAL_GetTick() - start_time;
        current_state = SEQ_IDLE;
        Sequencer_SaveToFlash();
    }
}

void Sequencer_RecordEvent(uint8_t note, float velocity, uint8_t type) {
    if (current_state != SEQ_RECORDING) return;
    RecordingSlot *s = &library[active_slot];
    if (s->event_count < MAX_EVENTS) {
        s->events[s->event_count] = (MidiEvent){note, velocity, HAL_GetTick() - start_time, type};
        s->event_count++;
    } else Sequencer_StopRecord();
}

void Sequencer_StartPlay(uint8_t slot) {
    if (slot >= MAX_SLOTS || library[slot].event_count == 0) return;
    active_slot = slot;
    play_ptr = 0;
    start_time = HAL_GetTick();
    current_state = SEQ_PLAYING;
}

void Sequencer_PausePlay(void) {
    if (current_state == SEQ_PLAYING) {
        pause_time = HAL_GetTick();
        current_state = SEQ_PAUSED;
        for (int i = 0; i < 128; i++) AudioEngine_NoteOff(i);
    }
}

void Sequencer_ResumePlay(void) {
    if (current_state == SEQ_PAUSED) {
        start_time += (HAL_GetTick() - pause_time);
        current_state = SEQ_PLAYING;
    }
}

void Sequencer_StopPlay(void) {
    for (int i = 0; i < 128; i++) AudioEngine_NoteOff(i);
    current_state = SEQ_IDLE;
}

void Sequencer_Update(void) {
    if (current_state == SEQ_RECORDING) {
        // Limit record time to 3 minutes
        if (HAL_GetTick() - start_time >= MAX_RECORD_TIME) {
            Sequencer_StopRecord();
        }
        return;
    }
    
    if (current_state != SEQ_PLAYING) return;
    
    uint32_t elapsed = HAL_GetTick() - start_time;
    RecordingSlot *s = &library[active_slot];
    while (play_ptr < s->event_count && elapsed >= s->events[play_ptr].timestamp) {
        if (s->events[play_ptr].type == 1) AudioEngine_NoteOn(s->events[play_ptr].note, s->events[play_ptr].velocity);
        else AudioEngine_NoteOff(s->events[play_ptr].note);
        play_ptr++;
    }
    if (play_ptr >= s->event_count) Sequencer_StopPlay();
}

uint32_t Sequencer_GetCurrentTime(void) {
    if (current_state == SEQ_IDLE) return 0;
    if (current_state == SEQ_PAUSED) return pause_time - start_time;
    return HAL_GetTick() - start_time;
}

uint32_t Sequencer_GetProgress(void) {
    if (current_state == SEQ_IDLE || current_state == SEQ_RECORDING) return 0;
    RecordingSlot *s = &library[active_slot];
    if (s->duration == 0) return 0;
    uint32_t elapsed = Sequencer_GetCurrentTime();
    uint32_t prog = (elapsed * 100) / s->duration;
    return (prog > 100) ? 100 : prog;
}

uint32_t Sequencer_GetDuration(uint8_t slot) { return library[slot].duration; }
SeqState Sequencer_GetState(void) { return current_state; }
uint8_t Sequencer_GetActiveSlot(void) { return active_slot; }
uint32_t Sequencer_GetEventCount(uint8_t slot) { return library[slot].event_count; }
