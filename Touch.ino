// ***************************************************************
// Touch.ino
//
// rricharz 2016
// ***************************************************************

#include"TouchScreen.h"

//  Arduino pins of touch pannel
#define YP A2  // must be an analog pin
#define XM A3  // must be an analog pin
#define YM 7   // can be a digital pin
#define XP 8   // can be a digital pin

// Calibration of the touchpannels, measured 50 points of borders
#define TS_MINX      845
#define TS_MAXX      156
#define TS_MINY      242
#define TS_MAXY      785

#define MINPRESSURE   10
#define MAXPRESSURE 1000

#define BORDERSIZE     5

// Resistance between X+ und Y-; Set to 300 Ohm
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

////////////////////////
int checkButtonTap(void) 
////////////////////////
{
  // Returns the number of the button, if valid, else -1

  int button;
  
  TSPoint p = ts.getPoint();
 
  // Minimal valid pressure
  
  if (p.z < MINPRESSURE || p.z > MAXPRESSURE) {
     return -1;         // No button pressed
  }

  // Serial.print("TFT: X = "); Serial.print(p.x);
  // Serial.print(" Y = "); Serial.print(p.y);
  // Serial.print(" Pressure = "); Serial.print(p.z);  
   
  // Calibration
  int x = map(p.y, TS_MINX, TS_MAXX, 50, tft.width() - 50) - XBOFFSET;   // MAX and MIN measured 50 points off from border
  int y = map(p.x, TS_MINY, TS_MAXY, 50, tft.height() - 50) - YBOFFSET;
  
  // For debugging: tft.fillCircle(x + XOFFSET, y + YOFFSET, 2, HX8357_RED);
  
  // Points near the border of each button are not valid

  if (((x % XBSIZE) < BORDERSIZE) || ((x % XBSIZE) >= (XBSIZE - BORDERSIZE))
    || ((y % YBSIZE) < BORDERSIZE) || ((y % YBSIZE) >= (YBSIZE - BORDERSIZE))) {
    // Serial.println(" Not within raster!");
    return -1;
  }
  
  x = x / XBSIZE;       // convert from dots to boxes
  y = y / YBSIZE;
  
  if (y != 0) {
    // Serial.print("Touched outside of valid button");
    return -1;
  }
  
  if ((x < 0) || (x > 4)) {
    // Serial.print("Touched outside of valid button");
    return -1;
  }
  
  // Serial.print("Button "); Serial.print(x); Serial.println(" touched");

  return x;
}


