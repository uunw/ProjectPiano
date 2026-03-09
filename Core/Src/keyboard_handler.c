#include "keyboard_handler.h"
#include "audio_engine.h"
#include "ui_controller.h"

extern uint8_t log_note;
extern uint8_t log_type;

// รายชื่อขา GPIO สำหรับเปียโน 12 ปุ่ม (Configured by User)
const uint16_t PIANO_PINS[] = {
    GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_7,
    GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13
};

void Keyboard_Scan_Direct(void) {
    static uint16_t last_key_state = 0;
    uint16_t current_state = 0;
    uint32_t idr = GPIOE->IDR;
    
    for (int i = 0; i < 11; i++) { 
        if (!(idr & PIANO_PINS[i])) current_state |= (1 << i); 
    }
    
    if (current_state != last_key_state) {
        int highest_note = -1;
        for (int i = 0; i < 11; i++) {
            uint16_t mask = (1 << i); uint8_t note = 60 + i;
            if ((current_state & mask) && !(last_key_state & mask)) { 
                AudioEngine_NoteOn(note, 0.9f); log_note = note; log_type = 1; 
            } 
            else if (!(current_state & mask) && (last_key_state & mask)) { 
                AudioEngine_NoteOff(note); log_note = note; log_type = 2; 
            }
            if (current_state & mask) highest_note = note;
        }
        Update_NoteUI(highest_note);
        last_key_state = current_state;
    }
}
