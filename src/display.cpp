#include "display.h"

#include "DSEG7.h"
#include "Esc.h"
#include "Light.h"
#include "Mot.h"
#include "Rev1.h"

// Instances
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite mainSprite = TFT_eSprite(&tft);

// Globals from main.cpp (Externs)
extern bool WIFI;
extern int batt;
extern int battPerc;
extern float trip;
extern unsigned int throttleRAW;
extern int escT;
extern int motT;
extern bool modeS;
extern bool lightF;
extern float speed;

extern bool lock;
extern bool confMode;
extern String entry;

extern bool showThReading; // from config.h (linked)

// Unit Macros (Duplicated from main.cpp or platformio logic)
#if USE_IMPERIAL_UNITS == 1
  #define CONVERT_UNIT(v) ((v) * 0.621371)
  #define UNIT_DIST_STR "mi"
#else
  #define CONVERT_UNIT(v) (v)
  #define UNIT_DIST_STR "km"
#endif

void initDisplay() {
  tft.init();
  tft.setRotation(0);
  tft.setSwapBytes(true);
#ifndef BOOT_LOGO_REPLACE_COLOR
// tft.pushImage(0,0,170,320,Rev1);
#else

  // Runtime color replacement for Boot Logo
  #ifndef BOOT_LOGO_REPLACE_COLOR
    #define BOOT_LOGO_REPLACE_COLOR 0x104B
  #endif

  #ifndef BOOT_LOGO_TOLERANCE
    #define BOOT_LOGO_TOLERANCE 10
  #endif

  // Extract target components once
  uint16_t targetColor = BOOT_LOGO_REPLACE_COLOR;
  uint8_t tr = (targetColor >> 11) & 0x1F;
  uint8_t tg = (targetColor >> 5) & 0x3F;
  uint8_t tb = targetColor & 0x1F;

  uint16_t lineBuffer[170];
  for (int y = 0; y < 320; y++) {
    for (int x = 0; x < 170; x++) {
      uint16_t p = Rev1[y * 170 + x];

      // Exact match optimization
      if (p == targetColor) {
        lineBuffer[x] = THEME_COLOR;
        continue;
      }

      // Check similarity
      uint8_t pr = (p >> 11) & 0x1F;
      uint8_t pg = (p >> 5) & 0x3F;
      uint8_t pb = p & 0x1F;

      int dist = abs(pr - tr) + abs(pg - tg) + abs(pb - tb);

      if (dist <= BOOT_LOGO_TOLERANCE) {
        lineBuffer[x] = THEME_COLOR;
      }
      else {
        lineBuffer[x] = p;
      }
    }
    tft.pushImage(0, y, 170, 1, lineBuffer);
  }
#endif
  mainSprite.createSprite(170, 320);
  mainSprite.setSwapBytes(true);
}

LockMode lockMode = PATTERN;

void drawPatternLock(int x, int y, int mode1, int mode2, int throttleCal) {
  mainSprite.fillSprite(TFT_BLACK);

  // Status Text
  mainSprite.setTextDatum(4);
  mainSprite.drawString("Locked", 85, 25, 4);

  // Grid Config
  int startX = 35;
  int startY = 100;
  int spacing = 50;

  // Check Touch Input & Update Entry
  int hitNode = 0;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      int cx = startX + c * spacing;
      int cy = startY + r * spacing;
      int id = r * 3 + c + 1;

      // Check Input radius 20
      if (x != -1 && abs(x - cx) < 20 && abs(y - cy) < 20) {
        hitNode = id;
      }
    }
  }

  if (hitNode > 0) {
    String s = String(hitNode);
    if (entry.indexOf(s) == -1) { // Unique nodes only
      entry += s;
    }
  }

  // Draw Connections
  for (int i = 0; i < (int)entry.length() - 1; i++) {
    int n1 = String(entry.charAt(i)).toInt();
    int n2 = String(entry.charAt(i + 1)).toInt();

    int r1 = (n1 - 1) / 3;
    int c1 = (n1 - 1) % 3;
    int x1 = startX + c1 * spacing;
    int y1 = startY + r1 * spacing;

    int r2 = (n2 - 1) / 3;
    int c2 = (n2 - 1) % 3;
    int x2 = startX + c2 * spacing;
    int y2 = startY + r2 * spacing;

    // Draw thick line
    mainSprite.drawLine(x1, y1, x2, y2, TFT_WHITE);
    mainSprite.drawLine(x1 + 1, y1, x2 + 1, y2, TFT_WHITE);
    mainSprite.drawLine(x1, y1 + 1, x2, y2 + 1, TFT_WHITE);
  }

  // Draw Nodes
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      int cx = startX + c * spacing;
      int cy = startY + r * spacing;
      int id = r * 3 + c + 1;

      // Highlight if in entry
      if (entry.indexOf(String(id)) != -1) {
        mainSprite.fillCircle(cx, cy, 8, TFT_WHITE);
        mainSprite.drawCircle(cx, cy, 12, TFT_WHITE);
      }
      else {
        // Inactive nodes: Thicker and lighter
        mainSprite.drawCircle(cx, cy, 5, TFT_SILVER);
        mainSprite.drawCircle(cx, cy, 6, TFT_SILVER);
      }
    }
  }

  // Check Codes
  if (entry.length() > 0) {
    long code = entry.toInt();
    if (code == mode1) {
      lock = 0;
    }
    if (code == mode2) {
      modeS = 1;
      lock = 0;
    }
    if (code == throttleCal) {
      confMode = 1;
    }
  }

  mainSprite.pushSprite(0, 0);
}

void drawPinLock(int x, int y, int mode1, int mode2, int throttleCal) {
  mainSprite.fillSprite(TFT_BLACK);
  mainSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  mainSprite.setTextDatum(4);

  // Status Text
  mainSprite.drawString("Locked", 85, 25, 4);

  int xpos[3] = {3, 58, 113};
  int ypos[4] = {73, 128, 183, 238};
  char chars[4][3] = {{'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {' ', '0', ' '}};
  int sx = 10, sy = 10;

  for (int i = 0; i < 4; i++) {
    if (y >= ypos[i] && y <= ypos[i] + 54)
      sy = i;
    for (int j = 0; j < 3; j++) {
      if (x >= xpos[j] && x <= xpos[j] + 54)
        sx = j;

      if (chars[i][j] != ' ') {
        int cx = xpos[j] + 27;
        int cy = ypos[i] + 27;

        if (sx == j && sy == i) {
          // Pressed State: 2px wide circle
          mainSprite.drawCircle(cx, cy, 24, THEME_COLOR);
          mainSprite.drawCircle(cx, cy, 23, THEME_COLOR);
        }
        else {
          // Normal State: 2px wide circle
          mainSprite.drawCircle(cx, cy, 24, TFT_WHITE);
          mainSprite.drawCircle(cx, cy, 23, TFT_WHITE);
        }

        mainSprite.drawString(String(chars[i][j]), cx, cy + 3, 4);
      }
    }
  }

  // Mask decorations removed as we simply skip drawing the empty space now

  // Process Input if Valid
  if (sx < 3 && sy < 4) {
    if (chars[sy][sx] != ' ') {
      entry += String(chars[sy][sx]);
    }
  }

  mainSprite.drawString(entry, 85, 52, 4);

  // Check Codes
  if (entry.toInt() == mode1)
    lock = 0;
  if (entry.toInt() == mode2) {
    modeS = 1;
    lock = 0;
  }
  if (entry.toInt() == throttleCal)
    confMode = 1;

  // Auto-reset if 4 digits and incorrect
  if (entry.length() >= 4 && lock == 1 && confMode == 0) {
    mainSprite.pushSprite(0, 0); // Ensure user sees the 4th digit
    delay(300);
    entry = "";
    // Next frame will draw empty
  }
  else {
    mainSprite.pushSprite(0, 0);
  }
}

void lockscreen(int x, int y, int mode1, int mode2, int throttleCal) {
  // Check Toggle (Long Press on "Locked" text)
  // Area: Centered 85, 25. Width 100, Height 50. => X: 35-135, Y: 0-50
  static unsigned long btnStart = 0;
  static bool btnTriggered = false;

  if (x > 35 && x < 135 && y > 0 && y < 50) {
    if (btnStart == 0) {
      btnStart = millis();
      btnTriggered = false;
    }
    else {
      if (millis() - btnStart > 1000 && !btnTriggered) {
        lockMode = (lockMode == PATTERN) ? PIN : PATTERN;
        entry = "";
        btnTriggered = true;
      }
    }
  }
  else {
    btnStart = 0;
    btnTriggered = false;
  }

  if (lockMode == PATTERN) {
    drawPatternLock(x, y, mode1, mode2, throttleCal);
  }
  else {
    drawPinLock(x, y, mode1, mode2, throttleCal);
  }
}

void drawScreen() {
  // Sprite
  mainSprite.fillSprite(TFT_BLACK);
  mainSprite.unloadFont(); // to draw all other txt before DSEG7 font
  // WIFI connection
  mainSprite.setTextColor(THEME_COLOR, TFT_BLACK);
  mainSprite.setTextDatum(4);
  if (WIFI == 1)
    mainSprite.drawString("WIFI connected", 110, 37, 2);
  // batt bar
  mainSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  if (battPerc > 15) {
    mainSprite.fillRoundRect(60, 10, battPerc, 15, 2, TFT_GREEN);
  }
  else if (battPerc > 0) {
    mainSprite.fillRoundRect(60, 10, battPerc, 15, 2, TFT_RED);
  }
  mainSprite.drawRoundRect(60, 10, 100, 15, 2, TFT_WHITE);
  // batt txt
  mainSprite.drawString(String(batt) + "V", 30, 8, 2);
  mainSprite.drawString(String(battPerc) + "%", 30, 26, 2);
  // trip txt
  mainSprite.setTextDatum(0);
  mainSprite.drawString(String("Trip"), 10, 182, 2);
  mainSprite.setTextDatum(2);
  mainSprite.drawString(String(CONVERT_UNIT(trip), 2) + UNIT_DIST_STR, 160, 175, 4);
  // show throttle reading
  if (showThReading == 1) {
    mainSprite.setTextDatum(0);
    mainSprite.drawString(String(throttleRAW), 10, 162, 2);
  }
  // line
  mainSprite.drawLine(0, 210, 170, 210, TFT_DARKGREY);
  // ESCTemp txt
  mainSprite.setTextDatum(2);
  mainSprite.drawString(String(escT), 40, 290, 4);
  mainSprite.drawCircle(46, 293, 3, TFT_WHITE);
  mainSprite.pushImage(10, 230, 40, 40, Esc);
  // motTemp txt
  mainSprite.setTextDatum(2);
  mainSprite.drawString(String(motT), 152, 290, 4);
  mainSprite.drawCircle(158, 293, 3, TFT_WHITE);
  mainSprite.pushImage(120, 230, 40, 40, Mot);
  // mode
  if (modeS == 1) {
    mainSprite.drawRoundRect(75, 286, 22, 27, 3, TFT_RED);
  }
  else {
    mainSprite.drawRoundRect(75, 286, 22, 27, 5, TFT_DARKGREY);
  }
  mainSprite.setTextDatum(4);
  mainSprite.drawString(String("S"), 86, 299, 2);
  // light
  mainSprite.pushImage(65, 230, 40, 40, Light);
  if (lightF == 1) {
    mainSprite.drawRoundRect(62, 229, 46, 41, 3, THEME_COLOR);
  }
  else {
    mainSprite.drawRoundRect(62, 229, 46, 41, 5, TFT_DARKGREY);
  }
  // speed
  mainSprite.setTextDatum(4);
  mainSprite.loadFont(DSEG7);
  float dispSpeed = CONVERT_UNIT(speed);
  if (dispSpeed < 0) {
    dispSpeed = 0;
  }
  else {
    mainSprite.drawString(String(dispSpeed, 0), 79, 102, 8);
  }
  // push Sprite to disp
  mainSprite.pushSprite(0, 0);
}
