#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "TFT_eSPI.h"
#include <Arduino.h>

// Initialize Display (TFT, Sprite, Boot Image)
void initDisplay();

// Draw the main dashboard screen
void drawScreen();

// Draw the lockscreen
void lockscreen(int x, int y, int mode1, int mode2, int throttleCal);

// External access to TFT/Sprite if needed (optional)
extern TFT_eSPI tft;
extern TFT_eSprite mainSprite;
extern bool confMode;
extern String entry;
enum LockMode {
    PATTERN = 0,
    PIN = 1
};
extern LockMode lockMode; // Toggle for lockscreen modeif

#endif
