#ifndef UI_CONTROLLER_H
#define UI_CONTROLLER_H

#include "main.h"
#include "tft_gfx.h"

// --- Button Coordinates ---
#define BTN_REC_X 22
#define BTN_REC_Y 165
#define BTN_REC_W 132
#define BTN_REC_H 45

#define BTN_LIB_X 165
#define BTN_LIB_Y 165
#define BTN_LIB_W 132
#define BTN_LIB_H 45

// --- Player Button Coordinates ---
#define PLR_BTN_Y 170
#define PLR_BTN_W 80
#define PLR_BTN_H 45
#define PLR_PLAY_X 120
#define PLR_STOP_X 210
#define PLR_BACK_X 30

// --- Mode Button Coordinates ---
#define MODE_Y 5
#define MODE_W 65
#define MODE_H 25
#define MODE_X_START 100

// --- Volume Bar Coordinates ---
#define VOL_X 10
#define VOL_Y 5
#define VOL_W 100
#define VOL_H 30

void Draw_PlayUI(void);
void Update_NoteUI(int note);
void Draw_VolumeBar(float vol);
void Draw_ModeButtons(int active_mode);
void UI_ProcessTouch(void);
void UI_RunCalibration(void);
void Draw_CalibrateUI(int step);

// --- New Features ---
void UI_UpdatePlayerProgress(void);
void UI_UpdateRecordingTimer(void);

#endif
