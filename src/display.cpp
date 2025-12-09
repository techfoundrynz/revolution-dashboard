#include "display.h"

#include "DSEG7.h"
#include "Rev1.h"
#include "Esc.h"
#include "Mot.h"
#include "Light.h"

// Instances
TFT_eSPI tft = TFT_eSPI(); 
TFT_eSprite mainSprite= TFT_eSprite(&tft);

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
  tft.pushImage(0,0,170,320,Rev1);
  mainSprite.createSprite(170,320);
  mainSprite.setSwapBytes(true); 
}

void lockscreen(int x, int y, int mode1, int mode2, int throttleCal) {

 tft.fillScreen(TFT_BLACK);
 tft.setTextColor(TFT_WHITE,TFT_BLACK);    
 tft.setTextDatum(4);

 int xpos[3]={3,58,113};
 int ypos[4]={73,128,183,238};
 char chars[4][3]={{'1','2','3'},{'4','5','6'},{'7','8','9'},{' ','0','x'}};
 int sx=10, sy=10;
 for(int i=0;i<4;i++){if(y>=ypos[i] && y<=ypos[i]+54)
 sy=i;
 for(int j=0;j<3;j++){if(x>=xpos[j] && x<=xpos[j]+54)
 sx=j;
 if(sx==j && sy==i)    
  tft.drawRoundRect(xpos[j],ypos[i],54,54,2,TFT_DARKGREY);
 else
  tft.drawRoundRect(xpos[j],ypos[i],54,54,2,TFT_WHITE);  
  tft.drawRoundRect(3,238,54,54,2,TFT_BLACK);
  tft.drawRoundRect(113,238,54,54,2,TFT_BLACK);
  tft.drawString(String(chars[i][j]),xpos[j]+27,ypos[i]+30,4);
 if(x>=xpos[j] && x<=xpos[j]+54)
 sx=j;
 }}
              
 if(chars[sy][sx]=='x') //clear entry
   entry="";
 else
   entry=entry+String(chars[sy][sx]);
   
 if(entry.toInt() == mode1) {
   lock=0;
  }

 if(entry.toInt() == mode2) {
   modeS=1;
   lock=0;
  }

  tft.drawString(entry,85,40,6); //draw touched entry

 if(entry.toInt() == throttleCal) {
   confMode = 1;
  }
}

void drawScreen() {
 //Sprite
 mainSprite.fillSprite(TFT_BLACK);
 mainSprite.unloadFont(); //to draw all other txt before DSEG7 font
 //WIFI connection
 mainSprite.setTextColor(TFT_BLUE,TFT_BLACK);
 mainSprite.setTextDatum(4);
 if (WIFI == 1)
   mainSprite.drawString("WIFI connected",110,37,2);
 //batt bar
 mainSprite.setTextColor(TFT_WHITE,TFT_BLACK);
 if (battPerc>15) {
  mainSprite.fillRoundRect(60,10,battPerc,15,2,TFT_GREEN);
  }
  else if (battPerc > 0) {
  mainSprite.fillRoundRect(60,10,battPerc,15,2,TFT_RED);
  }
 mainSprite.drawRoundRect(60,10,100,15,2,TFT_WHITE);
 //batt txt
 mainSprite.drawString(String(batt)+"V",30,8,2);
 mainSprite.drawString(String(battPerc)+"%",30,26,2);
 //trip txt
 mainSprite.setTextDatum(0);
 mainSprite.drawString(String("Trip"),10,182,2);
 mainSprite.setTextDatum(2);
    mainSprite.drawString(String(CONVERT_UNIT(trip),2)+UNIT_DIST_STR,160,175,4);
 //show throttle reading
 if (showThReading == 1) {
   mainSprite.setTextDatum(0);
   mainSprite.drawString(String(throttleRAW),10,162,2);
 }
 //line
 mainSprite.drawLine(0,210,170,210,TFT_DARKGREY);
 //ESCTemp txt
 mainSprite.setTextDatum(2);
 mainSprite.drawString(String(escT),40,290,4);
 mainSprite.drawCircle(46,293,3,TFT_WHITE);
 mainSprite.pushImage(10,230,40,40,Esc);
 //motTemp txt
 mainSprite.setTextDatum(2);
 mainSprite.drawString(String(motT),152,290,4);
 mainSprite.drawCircle(158,293,3,TFT_WHITE);
 mainSprite.pushImage(120,230,40,40,Mot);
 //mode
 if (modeS==1) {mainSprite.drawRoundRect(75,286,22,27,3,TFT_RED);}
 else {mainSprite.drawRoundRect(75,286,22,27,3,TFT_DARKGREY);}
 mainSprite.setTextDatum(4);
 mainSprite.drawString(String("S"),86,299,2);
 //light
 mainSprite.pushImage(65,230,40,40,Light);
 if (lightF==1) {mainSprite.drawRoundRect(62,229,46,41,3,TFT_BLUE);}
 else {mainSprite.drawRoundRect(62,229,46,41,3,TFT_DARKGREY);}
 //speed
 mainSprite.setTextDatum(4);
 mainSprite.loadFont(DSEG7);
  float dispSpeed = CONVERT_UNIT(speed);
 if (dispSpeed < 0) {dispSpeed = 0;}
 else {mainSprite.drawString(String(dispSpeed,0),79,102,8);}
 //push Sprite to disp
 mainSprite.pushSprite(0,0);
}
