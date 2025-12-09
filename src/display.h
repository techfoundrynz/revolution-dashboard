#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <Arduino.h>
#include "TFT_eSPI.h"

// Initialize Display (TFT, Sprite, Boot Image)
void initDisplay();

// Draw the main dashboard screen
void drawScreen();

// Draw the lockscreen
void lockscreen(int x, int y, int mode1, int mode2, int throttleCal);

// External access to TFT/Sprite if needed (optional)
extern TFT_eSPI tft;
extern TFT_eSprite mainSprite;

#endif
