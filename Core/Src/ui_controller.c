#include "ui_controller.h"
#include "tft_gfx.h"
#include "touch_driver.h"
#include "audio_engine.h"
#include "sequencer.h"
#include <stdio.h>
#include <math.h>

extern int last_drawn_note;
static int current_mode = 0;

typedef enum { SCREEN_PLAY, SCREEN_LIBRARY, SCREEN_PLAYER } AppScreen;
static AppScreen current_screen = SCREEN_PLAY;
static uint32_t last_progress = 0;
static uint32_t last_rec_sec = 0xFFFFFFFF;

const char* MODE_NAMES[] = {"PIANO", "E-PIANO", "STRINGS"};

void Draw_ModeButtons(int active_mode) {
    for (int i = 0; i < 3; i++) {
        uint16_t x = MODE_X_START + (i * (MODE_W + 5));
        uint16_t bg_color = (i == active_mode) ? COLOR_ACCENT : COLOR_BTN_BG;
        uint16_t text_color = (i == active_mode) ? 0x0000 : COLOR_TEXT_MAIN;
        GFX_FillRect(x, MODE_Y + 5, MODE_W, MODE_H, bg_color);
        GFX_DrawString(x + 10, MODE_Y + 12, MODE_NAMES[i], text_color, bg_color, 1);
    }
}

void Draw_VolumeBar(float vol) {
    int v = (int)(vol * 10);
    if (v > 10) v = 10;
    GFX_FillRect(VOL_X, 12, 80, 10, COLOR_PANEL);
    for (int i = 0; i < 10; i++) {
        uint16_t color = (i < v) ? (i < 6 ? COLOR_VOL_LOW : (i < 9 ? COLOR_VOL_MID : COLOR_VOL_HIGH)) : COLOR_VOL_BASE;
        GFX_FillRect(VOL_X + (i * 8), 12, 6, 10, color);
    }
}

void UI_UpdateRecordingTimer(void) {
    if (current_screen != SCREEN_PLAY || Sequencer_GetState() != SEQ_RECORDING) {
        last_rec_sec = 0xFFFFFFFF;
        return;
    }
    
    uint32_t current_sec = Sequencer_GetCurrentTime() / 1000;
    if (current_sec != last_rec_sec) {
        char buf[16];
        sprintf(buf, "REC %02lu:%02lu", current_sec / 60, current_sec % 60);
        GFX_FillRect(220, 60, 80, 20, COLOR_BG);
        GFX_DrawString(220, 65, buf, 0xF800, COLOR_BG, 1); // Red color for Timer
        last_rec_sec = current_sec;
        
        // Auto-refresh UI if record stopped by timeout
        if (Sequencer_GetState() == SEQ_IDLE) Draw_PlayUI();
    }
}

void Draw_LibraryUI(void) {
    current_screen = SCREEN_LIBRARY;
    GFX_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BG);
    GFX_FillRect(0, 0, TFT_WIDTH, 40, COLOR_PANEL);
    GFX_DrawString(110, 15, "SOUND LIBRARY", COLOR_TEXT_MAIN, COLOR_PANEL, 1);
    
    for (int i = 0; i < 4; i++) {
        uint16_t y = 50 + (i * 40);
        GFX_FillRect(20, y, 280, 35, COLOR_BTN_BG);
        char buf[32];
        uint32_t count = Sequencer_GetEventCount(i);
        if (count > 0) sprintf(buf, "SLOT %d: %lu EVENTS", i + 1, count);
        else sprintf(buf, "SLOT %d: EMPTY", i + 1);
        GFX_DrawString(40, y + 12, buf, COLOR_TEXT_MAIN, COLOR_BTN_BG, 1);
    }
    GFX_FillRect(20, 210, 280, 25, COLOR_ACCENT);
    GFX_DrawString(130, 217, "BACK", 0x0000, COLOR_ACCENT, 1);
}

void Draw_PlayerUI(void) {
    current_screen = SCREEN_PLAYER;
    GFX_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BG);
    GFX_FillRect(0, 0, TFT_WIDTH, 40, COLOR_PANEL);
    char buf[32];
    sprintf(buf, "PLAYING SLOT %d", Sequencer_GetActiveSlot() + 1);
    GFX_DrawString(100, 15, buf, COLOR_TEXT_MAIN, COLOR_PANEL, 1);

    GFX_DrawRect(20, 60, 280, 80, COLOR_VOL_BASE);
    uint32_t dur = Sequencer_GetDuration(Sequencer_GetActiveSlot()) / 1000;
    sprintf(buf, "DURATION: %02lu:%02lu", dur / 60, dur % 60);
    GFX_DrawString(40, 80, buf, COLOR_TEXT_DIM, COLOR_BG, 1);
    
    GFX_DrawRect(40, 110, 240, 10, COLOR_VOL_BASE);
    last_progress = 0;

    GFX_FillRect(PLR_BACK_X, PLR_BTN_Y, PLR_BTN_W, PLR_BTN_H, COLOR_BTN_BG);
    GFX_DrawString(PLR_BACK_X + 25, PLR_BTN_Y + 15, "BACK", COLOR_TEXT_MAIN, COLOR_BTN_BG, 1);

    const char* play_txt = (Sequencer_GetState() == SEQ_PAUSED) ? "RESUME" : "PAUSE";
    GFX_FillRect(PLR_PLAY_X, PLR_BTN_Y, PLR_BTN_W, PLR_BTN_H, COLOR_ACCENT);
    GFX_DrawString(PLR_PLAY_X + 15, PLR_BTN_Y + 15, play_txt, 0x0000, COLOR_ACCENT, 1);

    GFX_FillRect(PLR_STOP_X, PLR_BTN_Y, PLR_BTN_W, PLR_BTN_H, 0xF800);
    GFX_DrawString(PLR_STOP_X + 25, PLR_BTN_Y + 15, "STOP", COLOR_TEXT_MAIN, 0xF800, 1);
}

void UI_UpdatePlayerProgress(void) {
    if (current_screen != SCREEN_PLAYER) return;
    uint32_t prog = Sequencer_GetProgress();
    if (prog != last_progress) {
        GFX_FillRect(40, 110, (prog * 240) / 100, 10, COLOR_ACCENT);
        last_progress = prog;
        if (Sequencer_GetState() == SEQ_IDLE) Draw_LibraryUI();
    }
}

void Draw_PlayUI(void) {
    current_screen = SCREEN_PLAY;
    GFX_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BG);
    GFX_FillRect(0, 0, TFT_WIDTH, 40, COLOR_PANEL);
    Draw_ModeButtons(current_mode);
    Draw_VolumeBar(master_volume);
    
    GFX_DrawRect(15, 50, 290, 105, COLOR_VOL_BASE);
    GFX_DrawString(25, 60, "CURRENT NOTE", COLOR_TEXT_DIM, COLOR_BG, 1);
    
    uint16_t rec_color = (Sequencer_GetState() == SEQ_RECORDING) ? 0xF800 : COLOR_BTN_BG;
    const char* rec_text = (Sequencer_GetState() == SEQ_RECORDING) ? "STOP REC" : "RECORD";
    GFX_FillRect(BTN_REC_X, BTN_REC_Y, BTN_REC_W, BTN_REC_H, rec_color);
    GFX_DrawString(BTN_REC_X + 30, BTN_REC_Y + 15, rec_text, COLOR_TEXT_MAIN, rec_color, 1);
    
    GFX_FillRect(BTN_LIB_X, BTN_LIB_Y, BTN_LIB_W, BTN_LIB_H, COLOR_BTN_BG);
    GFX_DrawString(BTN_LIB_X + 35, BTN_LIB_Y + 15, "LIBRARY", COLOR_TEXT_MAIN, COLOR_BTN_BG, 1);
    
    GFX_FillRect(0, 215, TFT_WIDTH, 25, COLOR_PANEL);
    GFX_DrawString(140, 222, "READY", COLOR_ACCENT, COLOR_PANEL, 1);
    Update_NoteUI(-1);
    last_rec_sec = 0xFFFFFFFF; // Reset timer tracking
}

void Update_NoteUI(int note) {
    if (current_screen != SCREEN_PLAY) return;
    if (note == last_drawn_note) return;
    GFX_FillRect(100, 75, 120, 75, COLOR_BG);
    if (note >= 0) GFX_DrawString(125, 90, GFX_NoteToName(note), COLOR_NOTE, COLOR_BG, 4);
    else GFX_DrawString(130, 90, "---", COLOR_TEXT_DIM, COLOR_BG, 4);
    last_drawn_note = note;
}

void UI_ProcessTouch(void) {
    static uint32_t last_touch_time = 0;
    TouchPoint p;
    if (!Touch_Read(&p)) return;

    if (current_screen == SCREEN_PLAYER) {
        if (HAL_GetTick() - last_touch_time < 300) return;
        last_touch_time = HAL_GetTick();
        if (p.x >= PLR_BACK_X && p.x <= PLR_BACK_X + PLR_BTN_W && p.y >= PLR_BTN_Y) {
            Sequencer_StopPlay(); Draw_LibraryUI(); return;
        }
        if (p.x >= PLR_PLAY_X && p.x <= PLR_PLAY_X + PLR_BTN_W && p.y >= PLR_BTN_Y) {
            if (Sequencer_GetState() == SEQ_PLAYING) Sequencer_PausePlay();
            else Sequencer_ResumePlay();
            Draw_PlayerUI(); return;
        }
        if (p.x >= PLR_STOP_X && p.x <= PLR_STOP_X + PLR_BTN_W && p.y >= PLR_BTN_Y) {
            Sequencer_StopPlay(); Draw_LibraryUI(); return;
        }
        return;
    }

    if (current_screen == SCREEN_LIBRARY) {
        if (HAL_GetTick() - last_touch_time < 300) return;
        last_touch_time = HAL_GetTick();
        if (p.y >= 210) { Draw_PlayUI(); return; }
        for (int i = 0; i < 4; i++) {
            uint16_t y = 50 + (i * 40);
            if (p.y >= y && p.y <= y + 35) {
                if (Sequencer_GetEventCount(i) > 0) { Sequencer_StartPlay(i); Draw_PlayerUI(); }
                return;
            }
        }
        return;
    }

    if (p.y >= MODE_Y && p.y <= (MODE_Y + MODE_H + 10)) {
        for (int i = 0; i < 3; i++) {
            uint16_t x = MODE_X_START + (i * (MODE_W + 5));
            if (p.x >= x && p.x <= (x + MODE_W)) {
                if (current_mode != i) {
                    current_mode = i; AudioEngine_SetEngine((SoundEngine)i);
                    Draw_ModeButtons(current_mode); HAL_Delay(100);
                }
                return;
            }
        }
    }

    if (p.x >= VOL_X && p.x <= (VOL_X + VOL_W) && p.y >= VOL_Y && p.y <= (VOL_Y + VOL_H)) {
        float new_vol = (float)(p.x - VOL_X) / 80.0f;
        if (new_vol < 0.0f) new_vol = 0.0f;
        if (new_vol > 1.0f) new_vol = 1.0f;
        if (fabs(new_vol - master_volume) > 0.01f) {
            master_volume = new_vol; AudioEngine_SetVolume(master_volume);
            Draw_VolumeBar(master_volume);
        }
        return;
    }

    if (HAL_GetTick() - last_touch_time < 400) return;
    last_touch_time = HAL_GetTick();
    if (p.x >= BTN_REC_X && p.x <= (BTN_REC_X + BTN_REC_W) && p.y >= BTN_REC_Y && p.y <= (BTN_REC_Y + BTN_REC_H)) {
        if (Sequencer_GetState() == SEQ_IDLE) Sequencer_StartRecord(0); 
        else Sequencer_StopRecord();
        Draw_PlayUI();
    }
    if (p.x >= BTN_LIB_X && p.x <= (BTN_LIB_X + BTN_LIB_W) && p.y >= BTN_LIB_Y && p.y <= (BTN_LIB_Y + BTN_LIB_H)) {
        Draw_LibraryUI();
    }
}

void Draw_CalibrateUI(int step) {
    GFX_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, COLOR_BG);
    GFX_DrawString(60, 100, "4-POINT CALIBRATION", COLOR_TEXT_MAIN, COLOR_BG, 1);
    uint16_t color = COLOR_ACCENT;
    if (step == 0) GFX_FillRect(10, 10, 15, 15, color);
    else if (step == 1) GFX_FillRect(295, 10, 15, 15, color);
    else if (step == 2) GFX_FillRect(295, 215, 15, 15, color);
    else if (step == 3) GFX_FillRect(10, 215, 15, 15, color);
}

void UI_RunCalibration(void) {
    uint16_t rx[4], ry[4];
    for (int i = 0; i < 4; i++) {
        Draw_CalibrateUI(i);
        HAL_Delay(500);
        while (!Touch_IsPressed());
        Touch_ReadRaw(&rx[i], &ry[i]);
        while (Touch_IsPressed());
    }
    GFX_FillRect(0, 0, 320, 240, COLOR_BG);
    GFX_DrawString(60, 100, "CALIBRATION COMPLETE", COLOR_ACCENT, COLOR_BG, 1);
    HAL_Delay(2000);
    Draw_PlayUI();
}
