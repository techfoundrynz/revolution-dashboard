/*
Project: Revolution Dashboard 1.0
Author: Roland Stollnberger (stollnberger@gmx.at)
Created: 12.05.2024
Description: Control VESC based ESC, set Profiles, Display Data, Lockscreen, Over The Air programming...
*/

#include "Arduino.h"
#include "LiPoCheck.h"
#include "TFT_eSPI.h"
#include "VescComms.h"
#include "Wire.h"
#include "config.h"
#include "display.h"
#include "driver/ledc.h" //for PWM
#include <ArduinoOTA.h>
#include <CST816S.h> //TouchLib
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
bool WIFI = 0;

#include <SimpleKalmanFilter.h> //to smooth throttle value
SimpleKalmanFilter thFilter(2, 2, 0.01);

#include <Preferences.h>
Preferences pref;

#if VESC_COMM_TYPE == 2
// Using CAN
#else
  #include <HardwareSerial.h>
HardwareSerial SerialVESC(2);
#endif
VescComms Vesc;

#if USE_IMPERIAL_UNITS == 1
  #define CONVERT_UNIT(v) ((v) * 0.621371)
  #define UNIT_DIST_STR "mi"
#else
  #define CONVERT_UNIT(v) (v)
  #define UNIT_DIST_STR "km"
#endif

// images & font
#include "DSEG7.h"
#include "Esc.h"
#include "Light.h"
#include "Mot.h"
#include "Rev1.h"

#ifndef VESC_COMM_TYPE
  #define VESC_COMM_TYPE 1 // 1=UART, 2=CAN
#endif

#if VESC_COMM_TYPE == 2 && (!defined(VESC_CONTROLLER_CAN_ID) || !defined(CAN_ID))
  #error "For CAN communication, VESC_CONTROLLER_CAN_ID and CAN_ID must be defined in config"
#endif

#if defined(BOOSTED_BMS) && BOOSTED_BMS == 1
  #if VESC_COMM_TYPE != 2
    #error "Boosted BMS requires CAN communication (VESC_COMM_TYPE needs to be 2)"
  #endif
#endif

#define PIN_TX 44 // Lilygo Pin44=TX to VescRX
#define PIN_RX 43 // Lilygo Pin43=RX to VescTX
#define DEVICE_NAME "revolution-dashboard"

void lockscreen(int x, int y);
void drawScreen();

// setup PWM for rearlight
const int PWM_CHANNEL = 1;
const int PWM_FREQ = 500;
const int PWM_RESOLUTION = 8;
const int backLight_DUTY_CYCLE = dimmBL;
const int brakeLight_DUTY_CYCLE = 255; // 255 for max brightness = brakelight

float erpm = 0;
float rpm = 0;
float speed = 0;
int batt = 0;
int battPerc;
float trip;
int escT = 0;
int motT = 0;
bool profSet = 0;

bool lightF = 0;
bool lock = 1;
bool modeS = 0;
bool confMode = 0;

String entry = "";

int nunck = 127;
uint32_t filterTime = 0; // for Kalman Filter
bool filterDelay = 1;

unsigned int maxVal = 0;
unsigned int minVal = 0;
unsigned int thMax;
unsigned int thZero;
unsigned int thMin;
unsigned int throttleRAW;

// debounce for touch
unsigned long lastTouchTime = 0;   // to manage debounce
unsigned long debounceDelay = 300; // delay in milliseconds to wait for next valid touch input

CST816S touch(18, 17, 21, 16); // sda, scl, rst, irq

void configureWifi() {
  if (!WIFI && WiFi.status() == WL_CONNECTED) {
    WIFI = true;

    // Setup mDNS
    MDNS.begin(DEVICE_NAME);
    MDNS.addService("http", "tcp", 80);

    // Setup OTA
    ArduinoOTA.setHostname(DEVICE_NAME);
    ArduinoOTA.begin();
  }
}

bool wasTouched = false;

void setup() {
  pref.begin("thValues", false); //"false" defines read/write access
  thMax = pref.getUInt("thMax", 0);
  thZero = pref.getUInt("thZero", 0);
  thMin = pref.getUInt("thMin", 0);
  pref.end();

  // Load Lock Mode
  pref.begin("lockCfg", false);
  lockMode = (LockMode)pref.getUInt("lockMode", (int)PATTERN);
  pref.end();

  // OTA
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_NAME);
  WiFi.begin(ssid, password);
  ArduinoOTA.begin();
// comm VESC
#if VESC_COMM_TYPE == 2
  Vesc.beginCAN(PIN_TX, PIN_RX, VESC_CONTROLLER_CAN_ID, CAN_ID);
#else
  SerialVESC.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  while (!SerialVESC) {
    ;
  }
  Vesc.setSerialPort(&SerialVESC);
#endif
  Vesc.getFWversion();
  // setup the input & output pins
  pinMode(headlight, OUTPUT);
  pinMode(brakeSw, INPUT);
  pinMode(throttle, INPUT);
  maxVal = analogRead(throttle);
  minVal = analogRead(throttle);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(rearlight, PWM_CHANNEL);
  // display
  initDisplay();
  // touch
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  touch.begin();

  delay(2500); // waiting to start the VESC
  lockscreen(-1, -1, mode1, mode2, throttleCal);
}

void loop() {
  // OTA
  configureWifi();
  ArduinoOTA.handle();

  // Lockscreen
  LockMode lastLockMode = lockMode;
  static unsigned long lastSeenTouch = 0;

  while (lock == 1 && confMode == 0) {
    bool isTouched = touch.available();

    if (isTouched) {
      lastSeenTouch = millis();
      unsigned long currentTouchTime = millis();

      if (lockMode == PATTERN) {
        // Pattern Mode: Continuous Drag
        if (currentTouchTime - lastTouchTime > 500) {
          entry = "";
        }
        lastTouchTime = currentTouchTime;
        lockscreen(touch.data.x, touch.data.y, mode1, mode2, throttleCal);
        delay(20);
        wasTouched = true;
      }
      else {
        // PIN Mode
        if (touch.data.y < 80) {
          // Header Area: Allow continuous calls for Long Press Toggle
          lockscreen(touch.data.x, touch.data.y, mode1, mode2, throttleCal);
          wasTouched = true;
        }
        else {
          // Keypad Area: Single Press (with Hysteresis)
          if (!wasTouched) {
            wasTouched = true;
            lockscreen(touch.data.x, touch.data.y, mode1, mode2, throttleCal);
            lastTouchTime = millis();
          }
        }
      }

      // Save Lock Mode if changed
      if (lockMode != lastLockMode) {
        pref.begin("lockCfg", false);
        pref.putUInt("lockMode", (int)lockMode);
        pref.end();
        lastLockMode = lockMode;
      }
    }
    else {
      // Only reset wasTouched if release is stable (>50ms)
      if (millis() - lastSeenTouch > 50) {
        if (wasTouched) {
          wasTouched = false;
          // Force redraw to clear highlights in PIN mode
          if (lockMode == PIN) {
            lockscreen(-1, -1, mode1, mode2, throttleCal);
          }
        }
      }

      // Pattern Mode: Reset on Release if incorrect
      if (lockMode == PATTERN && entry.length() > 0) {
        if (millis() - lastTouchTime > 150) { // 150ms stable release
          entry = "";
          lockscreen(-1, -1, mode1, mode2, throttleCal); // Clear visuals
        }
      }
    }
  }

  // calculate the estimated value with Kalman Filter
  throttleRAW = thFilter.updateEstimate(analogRead(throttle));

  // calibrate throttle
  while (confMode == 1) {
    mainSprite.fillSprite(TFT_BLACK);
    mainSprite.setTextColor(TFT_WHITE, TFT_BLACK);
    mainSprite.setTextDatum(4);
    mainSprite.drawString("move throttle", 85, 20, 2);
    mainSprite.drawString("a few times from", 85, 40, 2);
    mainSprite.drawString("full throttle to full brake", 85, 60, 2);
    mainSprite.drawString("and press OK", 85, 80, 2);
    mainSprite.drawRoundRect(10, 245, 70, 50, 2, TFT_RED);
    mainSprite.drawString("X", 45, 272, 4);
    mainSprite.drawRoundRect(90, 245, 70, 50, 2, TFT_GREEN);
    mainSprite.drawString("OK", 125, 272, 4);

    unsigned int throttleRAW = thFilter.updateEstimate(analogRead(throttle));

    if (throttleRAW > maxVal) {
      maxVal = throttleRAW;
    }

    if (throttleRAW < minVal) {
      minVal = throttleRAW;
    }

    mainSprite.setTextDatum(0);
    mainSprite.drawString("old Value", 5, 110, 2);
    mainSprite.drawString(String(thMax), 5, 130, 4);
    mainSprite.drawString(String(thZero), 5, 160, 4);
    mainSprite.drawString(String(thMin), 5, 190, 4);

    mainSprite.setTextDatum(2);
    mainSprite.drawString("new Value", 165, 110, 2);
    mainSprite.drawString(String(maxVal), 165, 130, 4);
    mainSprite.drawString(String(throttleRAW), 165, 160, 4);
    mainSprite.drawString(String(minVal), 165, 190, 4);

    mainSprite.pushSprite(0, 0);

    if (touch.available()) {
      if (!wasTouched) {
        wasTouched = true;
        if (touch.data.y > 245 && touch.data.y < 295 && touch.data.x > 85) {
          pref.begin("thValues", false);
          pref.putUInt("thMax", maxVal);       // at 5V input, the Hall Sensor Value should be 4095 on full throttle
          pref.putUInt("thZero", throttleRAW); // schould be about 2880, depends on input Voltage ~ 5V
          pref.putUInt("thMin", minVal);       // schould be about 2100, depends on input Voltage ~ 5V
          pref.end();
          mainSprite.fillSprite(TFT_BLACK);
          mainSprite.pushSprite(0, 0);
          delay(100);
          ESP.restart();
        }
        if (touch.data.y > 245 && touch.data.y < 295 && touch.data.x < 85) {
          mainSprite.fillSprite(TFT_BLACK);
          mainSprite.pushSprite(0, 0);
          ESP.restart();
        }
      }
    }
    else {
      wasTouched = false;
    }
  }

  // set profile
  if (modeS == 1 && profSet == 0) {
    bool store = false;      // save persistently the new profile in vesc memory
    bool forward_can = true; // forward profile to slave Vesc through can bus
    bool ack = false;
    bool divide_by_controllers = false; // divied the watt limit by the number of VESC
    float current_min_rel = 1.0;   // this value is multiplied with the minimum current. It is a convenient method to
                                   // scale the current limits without forgetting the actual minimal value.  DEFAULT = 1
    float current_max_rel = 1.0;   // this value is multiplied with the maximum current. It is a convenient method to
                                   // scale the current limits without forgetting the actual maximum value.  DEFAULT = 1
    float speed_max_reverse = -15; // maximum reverse speed in m/s. In order, to use it make sure to set the wheel
                                   // diameter and gear ratio in VESC Tool   (m/s = (km/h / 3.6) )
    float speed_max =
        15; // maximum speed in m/s. In order, to use it make sure to set the wheel diameter and gear ratio in VESC Tool
    float duty_min = 0.005;      // minimum duty cycle. DEFAULT = 0.005
    float duty_max = 1;          // maximum duty cycle. DEFAULT = 1
    float watt_min = -1500000.0; // minimum watt value. DEFAULT = - 1500000.0
    float watt_max = 1500000.0;  // maximum watt value. DEFAULT =  1500000.0

    Vesc.setLocalProfile(store, forward_can, divide_by_controllers, current_min_rel, current_max_rel, speed_max_reverse,
                         speed_max, duty_min, duty_max, watt_min, watt_max);
    profSet = 1;
  }

  // switch headlight
  // switch headlight
  bool isTouched = touch.available();
  if (isTouched) {
    if (!wasTouched) {
      wasTouched = true;
      if (touch.data.y >= 212)
        lightF = !lightF;
    }
  }
  else {
    wasTouched = false;
  }

  if (lightF == HIGH) {
    digitalWrite(headlight, HIGH);
  }
  else {
    digitalWrite(headlight, LOW);
  }

  // handling brakelight
  if (digitalRead(brakeSw) == HIGH || throttleRAW < thZero - 250) { // reduce -250 for brakelight deadband
    ledcWrite(PWM_CHANNEL, brakeLight_DUTY_CYCLE);
  }
  else {
    ledcWrite(PWM_CHANNEL, backLight_DUTY_CYCLE);
  }

  // reading VESC data
  if (Vesc.getVescValues()) {
    erpm = Vesc.data.rpm;
    batt = Vesc.data.inpVoltage;
    escT = Vesc.data.tempFET;
    motT = Vesc.data.tempMotor;
    trip = Vesc.data.tachometer;
  }
  rpm = erpm / motPol;
  speed = erpm / motPol * wheelDia * 3.1415 * 0.00006;
  trip = trip / wheelDia / 1000 * tachComp;
  battPerc = CapCheckPerc(batt, numbCell);

  // calc nunchuck value
  float maxNunck;
  if (modeS == true) {
    maxNunck = 255;
  }
  else {
    maxNunck = thPercentage * 0.01 * 127 + 127;
  }

  if (lightF == HIGH && voltdropcomp == 1) { // compensation of the voltage drop when headlight is turned on
    throttleRAW = throttleRAW + thComp;
  }

  if (throttleRAW > thZero) {
    nunck = map(throttleRAW, thZero, thMax + 100, 127, maxNunck); //+100 to avoid running out of range

#if BOOSTED_BMS
                                                                  // BMS Keep Alive
    static unsigned long lastKeepAliveTime = 0;
    if (millis() - lastKeepAliveTime > 10000) {
      Vesc.sendKeepAlive();
      lastKeepAliveTime = millis();
    }
#endif
  }
  else if (throttleRAW < thZero) {
    nunck = map(throttleRAW, thZero, thMin - 100, 127, 0); //-100 to avoid running out of range
  }

  if (digitalRead(brakeSw) == 1 && nunck > 127 && stopOnBrake == 1) {
    nunck = 127; // interrupts acceleration when braking
  }

  // send nunchuck value to VESC
  if ((millis() - filterTime) > 1000 &&
      filterDelay == 1) { // this delay prevents the motors from stuttering at start when applying the kalman filter
    filterDelay = 0;
    filterTime = millis();
  }
  else {
    Vesc.nunchuck.valueY = nunck;
    Vesc.setNunchuckValues();
  }
  drawScreen();
}
