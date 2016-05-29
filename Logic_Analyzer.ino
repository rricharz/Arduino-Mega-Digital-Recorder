////////////////////////////////////////////////////////
// Logic / Analog Recorder (Version 1.7)
// 3 channels digital recorder, max 900 kHz, 7644 points
// or 8 bit analog recorder,    max  40 kHz, 3822 points
// Switch 4 at start selects mode
////////////////////////////////////////////////////////

// rricharz 2016

// #define DUMPDATA                    // define to dump data to Serial Monitor

// Used direct port read and bit manipulations to maximize acquisition speed
// Reads bit 0-3 of port A of the Arduino Mega 2560 corresponding to the arduino pins 22 - 25

// uses faster adc prescaler above 5 kHz sampling rate

// switch 1: Set for i2c mode (connect sda to pin 30, scl to pin 31, digital mode only)
// switch 4: Set for analog mode, clear for digital mode

#define TRIGGER (PINC & 128)        // Pin 30 (PC7) is used as the trigger signal
#define ANALOG_PIN 5                // Analog pin 5 is used as the analog input

const float VOLTat255 = 5.0;

// Define ADC prescalers for faster ADC
const unsigned char PS_16  = (1 << ADPS2);
const unsigned char PS_32  = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64  = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 

#include <Adafruit_GFX.h>
#include <SPI.h>
#include "Adafruit_HX8357.h"
#include "TouchScreen.h"

// Display uses hardware SPI, plus pin 9 and 10
#define TFT_RST -1  // dont use a reset pin, tie to arduino RST if you like
#define TFT_DC 9
#define TFT_CS 10

#define TFT_LITE 46 // pin to dimm or turn off the tft backlite
#define TFT_NODIMM 0

#define CARD_CS   4  // SD card needs to be turned off if unused, because it uses the SPI bus

Adafruit_HX8357 tft  = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

#define BUFFSIZE          3822    // should be a multiple of tft.width() - 2, must be a constant
#define CHANNELS             3

byte buffer[BUFFSIZE];            // Data buffer

byte ybarBuffer[480];             // cannot use tft.width() here, must be a constant
byte ybarBuffer2[480];          

const char triggerLabels[4][8] = { "none", "change", "falling", "rising"};

struct acquisitionParameters {
  int           nPoints;
  unsigned long setMicrosPerPoint;
  float         setTime; 
  float         actualTime;              // total time in microseconds
  float         actualMicrosPerPoint;    // microseconds per point
  byte          trigger;                 // trigger type
} acquisition;

struct displayParameters {
  int           hPixels;
  int           pointsPerPixel;
  int           firstPoint;
  long          microsPerPixel;
  float         time;
  float         start;
} disp;

struct cursorParameters {
  float         pixel;                         // this is a float because of exact calculations if pointsPerPixel > 1
  int           point;
  float         value;
} mainCursor, secondCursor;

#define XBOFFSET  5                            // define button position and size
#define XBSIZE    ((tft.width() - 30) / 5)
#define YBOFFSET  (tft.height() - 38)
#define YBSIZE    33

#define YPOFFSET  28                           // define parameter box position, size same as buttons

#define YGSIZE    48                           // define graph position and size
#define YGSTART   78
#define YGOFFSET  62

#define NUMBUTTONS 5
#define NUMPARAMS  5

const char buttonLabels[NUMBUTTONS][16] = {"TRIGGER TYPE", "TIME PER POINT", "START SCAN OF", "DISPLAY TIME", "DISPLAY START"};
const char buttonUnits[NUMBUTTONS][6]   = {      "",            " us",          " ms"  ,         " ms",          " ms"};

const char paramLabels[NUMPARAMS][16]   = {"CURSOR", "2nd CURSOR", "DIFFERENCE", "ACQ FREQUENCY", "VALUE"};
const char paramUnits[NUMPARAMS][6]     = {   " ms",     " ms",       " ms"  ,    " kHz",      " V"};

boolean buttonState[NUMBUTTONS];

boolean adcMode, i2cMode;

int minMicrosPerPoint;
int maxMicrosPerPoint;

////////////////
void setup(void)
////////////////
{ 
  Serial.begin(115200);
  Serial.println("\nLOGIC/ANALOG RECORDER (switch 4 sets mode)");
   // dimm the tft screen to reduce power consumption
  // arduino + tft:
  // 255 350 mA at 9V VIN
  // 127 250 mA at 9V VIN
  // 63  180 mA at 9V VIN
  pinMode(TFT_LITE, OUTPUT);
  analogWrite(TFT_LITE, 127);
  
  pinMode(CARD_CS, OUTPUT);
  digitalWrite(CARD_CS, HIGH);
 
  pinMode(30, INPUT);          // PC 7
  pinMode(31, INPUT);          // PC 6
  pinMode(32, INPUT);          // PC 5
  
  rotary_setup();
  
  tft.begin(HX8357D);
  tft.setRotation(3);
  
  adcMode = rotary_switch(4);                         // read only once at startup, cannot be changed during execution of program

  if (adcMode)
    i2cMode = false;
  else
    i2cMode = rotary_switch(1);
  
  tft.fillScreen(HX8357_BLACK);
  
  if (adcMode) {
    minMicrosPerPoint              = 25;
    maxMicrosPerPoint              = 128 * minMicrosPerPoint;
    acquisition.nPoints            = BUFFSIZE;
    acquisition.setMicrosPerPoint  = 100;

    tft.drawFastHLine(1, YGSTART - 1, tft.width() - 2, HX8357_YELLOW);
    tft.drawFastHLine(1, YGSTART + (512 / 3) + 1, tft.width() - 2, HX8357_YELLOW);
  }
  else {
    minMicrosPerPoint              = 1;
    maxMicrosPerPoint              = 256 * 5;
    acquisition.nPoints            = 2 * BUFFSIZE;
    acquisition.setMicrosPerPoint  = 5;
  }
  acquisition.setTime              = (float) acquisition.setMicrosPerPoint * (float) acquisition.nPoints / (float) 1000;
  acquisition.actualMicrosPerPoint = (float)acquisition.setMicrosPerPoint;
  acquisition.actualTime           = acquisition.setTime;
  acquisition.trigger              = 2;
  
  disp.hPixels                     = tft.width() - 2;
  disp.pointsPerPixel              = acquisition.nPoints / disp.hPixels;
  disp.firstPoint                  = 0;
  calcDependentDisplayParams();
  
  tft.drawRoundRect(0, 0, tft.width(), tft.height(), 4, HX8357_YELLOW);
  tft.setCursor(8,8);
  tft.setTextColor(HX8357_YELLOW);  tft.setTextSize(2);
  if (adcMode)
    tft.print  ("                   S4: analog recorder");
  else
    if (i2cMode)
      tft.print("S1: i2c             S4: logic recorder");
    else
      tft.print("S1: normal          S4: logic recorder");
  tft.setTextSize(1);
  
  mainCursor.pixel = 1.0;
  secondCursor.pixel = 1.0;
  mainCursor.value = disp.start + ((mainCursor.pixel - 1.0) * disp.time / float(disp.hPixels));
  mainCursor.point = (int)((mainCursor.pixel - 1.0) * (float)disp.pointsPerPixel) + disp.firstPoint;
  secondCursor.value = disp.start + ((secondCursor.pixel - 1.0) * disp.time / float(disp.hPixels));
  secondCursor.point = (int)((secondCursor.pixel - 1.0) * (float)disp.pointsPerPixel) + disp.firstPoint;

  // clear ybar buffer
  if (adcMode)
    for (int i = 1; i <= disp.hPixels; i++) {
      ybarBuffer[i] = 255;
      ybarBuffer2[i] = 0;
    }
  else
    for (int i = 1; i <= disp.hPixels; i++)
      ybarBuffer[i] = 0x55;

  for (int i = 0; i < NUMPARAMS; i++)
    displayParameter(i, true);
  
  for (int b = 0; b < NUMBUTTONS; b++) {
    buttonState[b] = false;
    displayButton(b, buttonState[b], true);
  }
}

/////////////////////////////////
void calcDependentDisplayParams()
/////////////////////////////////
{
  disp.microsPerPixel = acquisition.actualMicrosPerPoint * disp.pointsPerPixel;
  disp.time         = (float) disp.hPixels * (float) disp.microsPerPixel /  float (1000);
  disp.start        = (float) disp.firstPoint * acquisition.actualMicrosPerPoint / (float) 1000;
}

/////////////////////////////////////////////////////////////////////
void displayButton(int buttonNumber, boolean pushed, boolean newDraw)
/////////////////////////////////////////////////////////////////////
{
  int bgcolor, fgcolor, maxafterpoint;
  float value;
  char strbuf[16];
  
  maxafterpoint = 3;
  
  if      (buttonNumber == 0)
    value = 0.0;
  else if (buttonNumber == 1) {
    value = acquisition.actualMicrosPerPoint;
    if (acquisition.actualMicrosPerPoint < 100.0)
      maxafterpoint = 2;
    else
      maxafterpoint = 1;
  }
  else if (buttonNumber == 2)
    value = acquisition.setTime;
  else if (buttonNumber == 3)
    value = disp.time;
  else if (buttonNumber == 4)
    value = disp.start;
  else return;
  
  if (pushed) {
    bgcolor = tft.color565(127, 191, 255);
    fgcolor = HX8357_BLACK;
  }
  else {
    bgcolor = tft.color565(31,63,127);
    fgcolor = HX8357_WHITE;
  }
  tft.setTextColor(fgcolor); tft.setTextSize(1);
  if (newDraw) {
    tft.fillRoundRect(XBOFFSET + buttonNumber * (XBSIZE + 5), YBOFFSET, XBSIZE, YBSIZE, 5, bgcolor);
    tft.drawRoundRect(XBOFFSET + buttonNumber * (XBSIZE + 5) + 1, YBOFFSET + 1,XBSIZE - 2, YBSIZE - 2, 5, fgcolor);  
    tft.setCursor(XBOFFSET + buttonNumber * (XBSIZE + 5) + (XBSIZE / 2) - (strlen(buttonLabels[buttonNumber]) * 3), YBOFFSET + 8);
    tft.print(buttonLabels[buttonNumber]);
  }
    else {  // erase only value display area
    tft.fillRect(XBOFFSET + buttonNumber * (XBSIZE + 5) + 5, YBOFFSET + 18, XBSIZE - 10, 8, bgcolor);
  }
  if (buttonNumber == 0) {
    strcpy(strbuf, triggerLabels[acquisition.trigger]);
  }
  else {
    floatToStr(value, strbuf, maxafterpoint);
    strcat(strbuf, buttonUnits[buttonNumber]);
  }
  tft.setCursor(XBOFFSET + buttonNumber * (XBSIZE + 5) + (XBSIZE / 2) - (strlen(strbuf) * 3), YBOFFSET + 18);
  tft.print(strbuf);
}

/////////////////////////////////////////////////////
void displayParameter(int boxNumber, boolean newDraw)
/////////////////////////////////////////////////////
{
  int bgcolor, fgcolor;
  char strbuf[16];
  float value;
  if ((!adcMode) && (boxNumber > 3))
    return;
  if (boxNumber == 0)
    value = mainCursor.value;
  else if (boxNumber == 1)
    value = secondCursor.value;
  else if (boxNumber == 2)
    value = abs(mainCursor.value - secondCursor.value);
  else if (boxNumber == 3)
    value = 1000.0 / acquisition.actualMicrosPerPoint;
  else if (boxNumber == 4)
    value =  (float)buffer[mainCursor.point] * VOLTat255 / 255.0;
  bgcolor = tft.color565(191, 0, 0);
  fgcolor = HX8357_WHITE;
  if (newDraw) {
    tft.fillRoundRect(XBOFFSET + boxNumber * (XBSIZE + 5), YPOFFSET, XBSIZE, YBSIZE, 5, bgcolor);
    tft.drawRoundRect(XBOFFSET + boxNumber * (XBSIZE + 5) + 1, YPOFFSET + 1,XBSIZE - 2, YBSIZE - 2, 5, fgcolor);  
    tft.setTextColor(fgcolor); tft.setTextSize(1);
    tft.setCursor(XBOFFSET + boxNumber * (XBSIZE + 5) + (XBSIZE / 2) - (strlen(paramLabels[boxNumber]) * 3), YPOFFSET + 8);
    tft.print(paramLabels[boxNumber]);
  }
  else {    // erase only value display area
    tft.fillRect(XBOFFSET + boxNumber * (XBSIZE + 5) + 5, YPOFFSET + 18, XBSIZE - 10, 8, bgcolor);
  }
  if (boxNumber == 4)
    floatToStr(value, strbuf, 2);
  else
    floatToStr(value, strbuf, 3);
  strcat(strbuf, paramUnits[boxNumber]);
  tft.setTextColor(fgcolor); tft.setTextSize(1);
  tft.setCursor(XBOFFSET + boxNumber * (XBSIZE + 5) + (XBSIZE / 2) - (strlen(strbuf) * 3), YPOFFSET + 18);
  tft.print(strbuf);
}

////////////////////////
void printHexByte(int b)
////////////////////////
// Startdard HEX printing does not print leading 0
{
  // tft.print("0x");
  tft.print((b >> 4) & 0xF, HEX);
  tft.print(b & 0xF, HEX);
}


//////////////////////
void display_i2c(void)
//////////////////////
{
  int p, k, sda, scl, lastscl, sclStart, pos, n_one, n_zero, bitcount, data, state, lastData;
  Serial.println("display_i2c:");
  tft.fillRect(1, YGOFFSET + 2 * YGOFFSET + 12, tft.width() - 2, YGSIZE, HX8357_BLACK);
  p = 0;
  lastscl = 1;
  bitcount = 0;
  data = 0;
  state = 0;
  while (p < 2 * BUFFSIZE) {
      k = p / 2;
      if (p % 2) {
        sda = bitRead(buffer[k], 3);
        scl = bitRead(buffer[k], 2);
      }
      else {
        sda = bitRead(buffer[k], 7);
        scl = bitRead(buffer[k], 6);
      }
    if ((lastscl == 0) && (scl == 1)) {            // scl high begins
      lastscl = 1;
      sclStart = p;
      pos = (sclStart - disp.firstPoint) / disp.pointsPerPixel;
      n_one = 0;                                   // reset sda value
      n_zero = 0;
    }
    if (scl == 1) {                                // measure sda state                
      if (sda == 1)
        n_one++;
      else
        n_zero++;
    }
    if ((lastscl == 1) && (scl == 0)) {            // scl high ends
      lastscl = 0;
      // Serial.print("scl low: start = "); Serial.print(sclStart); Serial.print(", end = "); Serial.print(p - 1);
      // Serial.print(" pos = "); Serial.println(pos);
      
      if (pos < (tft.width() - 24)) {
        tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 14);
        tft.setTextSize(1);
        if ((n_zero == 0) && (n_one > 0)) {
          tft.setTextColor(HX8357_GREEN);
          if (pos > 0)
            tft.print('1');
          data = (data << 1) + 1;
          bitcount++;
        }
        else if ((n_one == 0) && (n_zero > 0)) {
          tft.setTextColor(HX8357_GREEN);
          if (pos > 0)
            tft.print('0');
          data = (data << 1);
          bitcount++;
        }
        else { 
          state = 0;
          bitcount = 0;
          data = 0;
        }
        if (state == 0) {                  // device number
          if (bitcount == 7) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 24);
            if (pos > 0)
              tft.setTextColor(HX8357_GREEN);
            printHexByte(data);
            state++;
            bitcount = 0;
            data = 0;
          }
        }
        else if (state == 1) {             // read or write
          if (bitcount == 1) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 34);
            tft.setTextColor(HX8357_GREEN);
            if (data) {
              if (pos > 0)
                tft.print('R');
            }
            else {
              if (pos > 0)
                tft.print('W');
            }
            state++;
            bitcount = 0;
            data = 0;
          }
        }
        else if (state == 2) {             // acqnowledge
          if (bitcount == 1) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 44);
            if (data) {
              tft.setTextColor(HX8357_RED);
              if (pos > 0)
                tft.print('A');   
            }
            else {
              tft.setTextColor(HX8357_GREEN);
              if (pos > 0)
                tft.print('A');   
            }            
            state++;
            bitcount = 0;
            data = 0;
          }
        }
        else if (state == 3) {              // address / data
          if (bitcount == 8) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 24);
            tft.setTextColor(HX8357_YELLOW);
            if (pos > 0)
              printHexByte(data);
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 34);
            if (pos > 0)
              tft.print(data);
            state++;
            bitcount = 0;
            lastData = data;
            data = 0;
          }
        }
        else if (state == 4) {             // acqnowledge
          if (bitcount == 1) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 44);
            if (data) {
              tft.setTextColor(HX8357_RED);
              if (pos > 0)
                tft.print('A');   
            }
            else {
              tft.setTextColor(HX8357_GREEN);
              if (pos > 0)
                tft.print('A');   
            }            
            state++;
            bitcount = 0;
            data = 0;
          }
        }
        else if (state == 5) {              // data 2
          if (bitcount == 8) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 24);
            tft.setTextColor(HX8357_YELLOW);
            if (pos > 0)
              printHexByte(data);
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 34);
            tft.setTextColor(HX8357_WHITE);
            if (pos > 0)
              tft.print((data << 8) | lastData);
            state++;
            bitcount = 0;
            data = 0;
          }
        }
        else if (state == 6) {             // acqnowledge
          if (bitcount == 1) {
            tft.setCursor(pos + 2, YGOFFSET + 2 * YGOFFSET + 44);
            if (data) {
              tft.setTextColor(HX8357_RED);
              if (pos > 0)
                tft.print('A');   
            }
            else {
              tft.setTextColor(HX8357_GREEN);
              if (pos > 0)
                tft.print('A');   
            }            
            state = 3;
            bitcount = 0;
            data = 0;
          }
        }
      }
    }
  p++;      
  }
}

/////////////////////////////////////
void display_digital(int graphNumber)
/////////////////////////////////////
{
  int pix, pPix, k, m, bitNum;
  boolean value, lastminvalue, lastmaxvalue;
  boolean minvalue = true;
  boolean maxvalue = false;
  int ymin = YGSTART + (graphNumber * YGOFFSET);
  
  bitNum = 3 - graphNumber;
  
  for (pix = 0; pix < tft.width() - 2; pix++) {
    for (pPix = 0; pPix < disp.pointsPerPixel; pPix++) {
      m = disp.firstPoint + (pix * disp.pointsPerPixel) + pPix;
      k = m / 2;
      if (m % 2)
        value = bitRead(buffer[k], bitNum);
      else
        value = bitRead(buffer[k], bitNum + 4);
      if (value == 0) minvalue = 0;
      else maxvalue = 1;
    }
    
    // erase last graph
    
    lastminvalue = bitRead(ybarBuffer[pix + 1], 2 * graphNumber);
    lastmaxvalue = bitRead(ybarBuffer[pix + 1], (2 * graphNumber) + 1);
    if (lastminvalue <= lastmaxvalue)
      tft.drawFastVLine(pix + 1, ymin + ((1 - lastmaxvalue) * YGSIZE), YGSIZE * (lastmaxvalue - lastminvalue) + 1, HX8357_BLACK);
    
    // draw new graph    
    if (minvalue <= maxvalue)
      tft.drawFastVLine(pix + 1, ymin + ((1 - maxvalue) * YGSIZE), YGSIZE * (maxvalue - minvalue) + 1, HX8357_GREEN);

    bitWrite(ybarBuffer[pix + 1], 2 * graphNumber, minvalue);          // store the graph data for erase/rewrite
    bitWrite(ybarBuffer[pix + 1], 2 * graphNumber + 1, maxvalue);

    minvalue = value;
    maxvalue = value;
  }
}

/////////////////////////
void display_analog(void)
/////////////////////////
{
  int pix, pPix, m;
  byte value, lastminvalue, lastmaxvalue;
  byte minvalue = 255;
  byte maxvalue = 0;
  int ymin = YGSTART;
  
  for (pix = 0; pix < tft.width() - 2; pix++) {
    for (pPix = 0; pPix < disp.pointsPerPixel; pPix++) {
      m = disp.firstPoint + (pix * disp.pointsPerPixel) + pPix;
      value = buffer[m];
      if (value < minvalue)
        minvalue = value;
      else if (value > maxvalue)
        maxvalue = value;
    }
    
    // erase last graph
    
    lastminvalue = ybarBuffer[pix + 1];
    lastmaxvalue = ybarBuffer2[pix + 1];
    if (lastminvalue <= lastmaxvalue)
      tft.drawFastVLine(pix + 1, ymin + ((2 * (255 - lastmaxvalue)) / 3), (2 * (lastmaxvalue - lastminvalue)) / 3 + 1, HX8357_BLACK);
    
    // draw new graph    
    if (minvalue <= maxvalue)
      tft.drawFastVLine(pix + 1, ymin + ((2 * (255 - maxvalue)) / 3), (2 * (maxvalue - minvalue)) / 3 + 1, HX8357_GREEN);

    ybarBuffer[pix + 1] = minvalue;          // store the graph data for erase/rewrite
    ybarBuffer2[pix + 1] = maxvalue;

    minvalue = value;
    maxvalue = value;
  }
}

////////////////////
void displayResult()
////////////////////
{
  if (adcMode)
    display_analog();
  else
    if (i2cMode) {
      for (int g = 0; g < (CHANNELS - 1); g++)
        display_digital(g);
      display_i2c();
    }
    else
      for (int g = 0; g < CHANNELS; g++)
        display_digital(g);
}

////////////////////////////
unsigned long acquire_data()
////////////////////////////
//
// Pins are hard coded for speed reason, change for trigger and actual acquisition
{
  unsigned long starttime, time;
  unsigned int v1, v2, v3;
  boolean p1val;
  if (adcMode) {
    if (acquisition.setMicrosPerPoint < 35.0) {
      ADCSRA &= ~PS_128;         // remove previously set bits on prescaler
      ADCSRA |= PS_16;           // set new bits on prescaler
    }

    else if (acquisition.setMicrosPerPoint < 65.0) {
      ADCSRA &= ~PS_128;         // remove previously set bits on prescaler
      ADCSRA |= PS_32;           // set new bits on prescaler
    }
    else if (acquisition.setMicrosPerPoint < 130.0) {
      ADCSRA &= ~PS_128;         // remove previously set bits on prescaler
      ADCSRA |= PS_64;           // set new bits on prescaler
    } 
  }
  if (acquisition.trigger == 1) {
    p1val = TRIGGER;
    while (TRIGGER == p1val);                // wait until TRIGGER PIN changes
  }
  else if (acquisition.trigger == 2) {
    while (!TRIGGER);                        // first wait until trigger is HIGH
    while (TRIGGER);                         // then wait until trigger goes from HIGH to LOW
  }
  else if (acquisition.trigger == 3) {
    while (TRIGGER);                         // first wait until trigger is LOW
    while (!TRIGGER);                        // then wait until trigger goes from LOW to HIGH
  }
  starttime = micros();
  time = starttime;
  int k = 0;
  if (adcMode) {
    while (k < BUFFSIZE) {
      buffer[k++] = analogRead(ANALOG_PIN) >> 2;   // use only 8 bit resolution !
      time += acquisition.setMicrosPerPoint;
      while (micros() < time);
    }
  }
  else {
    if (acquisition.setMicrosPerPoint >= 5) {
      while (k < BUFFSIZE) {                     // Port C bit 7 - 4 are pin 30 - 33 on Arduino Mega 2560
        buffer[k] = (PINC & 0xF0);               // read bits 4 - 7 of Port C, store them in bits 4 - 7 of buffer[k]
        time += acquisition.setMicrosPerPoint;
        while (micros() < time);
        buffer[k++] |= (PINC >> 4);              // read bits 4 - 7 of Port C, store them in bits 0 - 3 of buffer[k]
        time += acquisition.setMicrosPerPoint;
        while (micros() < time);
      }
    }
    else {
      while (k < BUFFSIZE) {                     // Port C bit 7 - 4 are pin 30 - 33 on Arduino Mega 2560
        buffer[k] = (PINC & 0xF0);               // read bits 4 - 7 of Port C, store them in bits 4 - 7 of buffer[k]
        buffer[k++] |= (PINC >> 4);              // read bits 4 - 7 of Port C, store them in bits 0 - 3 of buffer[k]
      }
    }
  }
  // calculate the actual acquisition parameter
  time = micros() - starttime;
  acquisition.actualTime           = (float) time / (float) 1000;
  acquisition.actualMicrosPerPoint = (float) time / (float) acquisition.nPoints;
  // Serial.print("ActualMicrosPerPoint = "); Serial.println(acquisition.actualMicrosPerPoint);
  
  if (adcMode && (acquisition.setMicrosPerPoint < 130.0))
    ADCSRA |= PS_128;        // set ADC prescaler back to normal
  
  // display the actual acquisition parameters
  acquisition.setTime            = acquisition.actualTime;
  acquisition.setMicrosPerPoint  = (float)acquisition.actualMicrosPerPoint;
  displayButton(1, false, false);     // display actual time per point
  displayButton(2, true, false);      // display actual scan time
  displayParameter(3, false);         // display actual acquisition frequecy
  disp.pointsPerPixel = acquisition.nPoints / disp.hPixels;
  disp.firstPoint     = 0;
  calcDependentDisplayParams();
}

////////////////////////////////////////
void displaySecondCursor(boolean redraw)
////////////////////////////////////////
{
  int nchannels;
  // erase current position of second cursor, if it was on screen and is not identical with first cursor
  if (((int) mainCursor.pixel != (int) secondCursor.pixel) && ((int) secondCursor.pixel >= 1) && ((int) secondCursor.pixel <= disp.hPixels)) {
  if (adcMode)
    eraseAnalogCursor(secondCursor.pixel);
  else
    eraseDigitalCursor(secondCursor.pixel); 
  }
  if (i2cMode)
    nchannels = CHANNELS - 1;
  else
    nchannels = CHANNELS;
  if (redraw) {
    secondCursor.pixel = (((secondCursor.value - disp.start) * (float) disp.hPixels) / disp.time) + 1.0;
    if (((int) secondCursor.pixel >= 1) && ((int) secondCursor.pixel <= disp.hPixels))  // if on screen
      tft.drawFastVLine((int) secondCursor.pixel, YGSTART - 10, YGSIZE + ((nchannels - 1) * YGOFFSET) + 20, HX8357_RED);
  }
  else // second cursor does not need to be drawn, because main cursor is at the same position
    secondCursor.pixel = mainCursor.pixel;
  secondCursor.value = disp.start + ((secondCursor.pixel - 1.0) * disp.time / float(disp.hPixels));
  secondCursor.point = (int)((secondCursor.pixel - 1.0) * (float)disp.pointsPerPixel) + disp.firstPoint;  displayParameter(1, false);
  displayParameter(2, false);
 
}

//////////////////////////////////
void eraseDigitalCursor(float pix)
//////////////////////////////////
{
    int nchannels;
    if ((int) mainCursor.pixel != (int) secondCursor.pixel) {
      if (i2cMode)
        nchannels = CHANNELS - 1;
      else
        nchannels = CHANNELS;
      tft.drawFastVLine((int) pix, YGSTART - 10, YGSIZE + ((nchannels - 1) * YGOFFSET) + 20, HX8357_BLACK);
      for (int i = 0; i < nchannels; i++) {
        int ymin = YGSTART + (i * YGOFFSET);
        boolean minvalue = bitRead(ybarBuffer[(int) pix], 2 * i);
        boolean maxvalue = bitRead(ybarBuffer[(int) pix], (2 * i) + 1);
        if (minvalue <= maxvalue)
          tft.drawFastVLine((int) pix, ymin + ((1 - maxvalue) * YGSIZE), YGSIZE * (maxvalue - minvalue) + 1, HX8357_GREEN);
      }
    }    
}

/////////////////////////////////
void eraseAnalogCursor(float pix)
/////////////////////////////////
{
    if ((int) mainCursor.pixel != (int) secondCursor.pixel) {
      tft.drawFastVLine((int) pix, YGSTART - 10, YGSIZE + ((CHANNELS - 1) * YGOFFSET) + 20, HX8357_BLACK);
      int ymin = YGSTART;
      byte minvalue = ybarBuffer[(int) pix];
      byte maxvalue = ybarBuffer2[(int) pix];
      if (minvalue <= maxvalue)
        tft.drawFastVLine((int) pix, ymin + ((2 * (255 - maxvalue)) / 3), (2 * (maxvalue - minvalue)) / 3 + 1, HX8357_GREEN);
      tft.drawPixel(pix, YGSTART - 1, HX8357_YELLOW);
      tft.drawPixel(pix, YGSTART + (512 / 3) + 1, HX8357_YELLOW);
    }    
}

//////////////////////////////////////
void displayMainCursor(boolean redraw)
//////////////////////////////////////
{
  int nchannels;
  int rotary = getAcceleratedRotaryChange();
  if (rotary || redraw) {
    // erase current cursor position
    if (adcMode)
      eraseAnalogCursor(mainCursor.pixel);
    else
      eraseDigitalCursor(mainCursor.pixel);
    // draw new cursor position
    if (redraw) {
      mainCursor.pixel = constrain((((mainCursor.value - disp.start) * (float) disp.hPixels) / disp.time) + 1.0, 1.0, (float)disp.hPixels);
      // Serial.print("New cursor pos = "); Serial.println((int) mainCursor.pixel);
    } 
    else
      mainCursor.pixel = constrain(mainCursor.pixel + rotary, 1.0, disp.hPixels);
    if (i2cMode)
        nchannels = CHANNELS - 1;
      else
        nchannels = CHANNELS;    
    tft.drawFastVLine((int) mainCursor.pixel, YGSTART - 10, YGSIZE + ((nchannels - 1) * YGOFFSET) + 20, HX8357_RED);
    mainCursor.value = disp.start + ((mainCursor.pixel - 1.0) * disp.time / float(disp.hPixels));
    mainCursor.point = (int)((mainCursor.pixel - 1.0) * (float)disp.pointsPerPixel) + disp.firstPoint;
    displayParameter(0, false);
    displayParameter(2, false);
    displayParameter(4, false);  
  }
}

//////////////////////////////////
void changeButtonState(int button)
//////////////////////////////////
{
   // if this button is turned on, all other buttons must be turned off
   if (!buttonState[button])
     for (int b = 0; b < NUMBUTTONS; b++)
       if ((b != button) && (buttonState[b])) {
         
         buttonState[b] = false;
         displayButton(b, buttonState[b], true);
       }
   // change the state of this button
   buttonState[button] = !buttonState[button];
   displayButton(button, buttonState[button], true);
}

////////////////////////////////
int getAcceleratedRotaryChange()
////////////////////////////////
{
#define AccTime 60
  static uint32_t lastTime;                   // last time the rotary encoder was changed
  int rotary = rotary_getchange();
  if (rotary) {
    int newTime = millis();
    
    if ((newTime - lastTime) < AccTime)
      rotary *= 2;
    if ((newTime - lastTime) < (14 * AccTime / 10))
      rotary *= 2;
    if ((newTime - lastTime) < (20 * AccTime / 10))
      rotary *= 2;
    if ((newTime - lastTime) < (30 * AccTime /10))
      rotary *= 2;
      
    // Serial.print("Rotary interval = "); Serial.print(newTime - lastTime); Serial.print(" msec, rotary = "); Serial.println(rotary);    
    lastTime = newTime;    
    return rotary;
  }
}

#ifdef DUMPDATA
////////////////
void dump_data()
////////////////
{
  Serial.println(BUFFSIZE);
  for (int i = 0; i < BUFFSIZE; i++) {
    Serial.println(buffer[i]);
  }
}
#endif

///////////
void loop()
///////////
{
  int button;
  int rotary;
  int pointsOnDisplay;
  
  if (buttonState[0]) {                          // FIRST BUTTON SET (TRIGGER TYPE)
    rotary = rotary_getchange();
    if (rotary) {
        acquisition.trigger = (acquisition.trigger + rotary) & 3;
      displayButton(0, true, false);
    }
  }
  
  else if (buttonState[1]) {                     // SECOND BUTTON SET (TIME PER POINT)
    rotary = rotary_getchange();
    if (rotary) {
      if (rotary > 0) {
        acquisition.setMicrosPerPoint = constrain(2 * acquisition.setMicrosPerPoint, minMicrosPerPoint, maxMicrosPerPoint);
        if (acquisition.setMicrosPerPoint < 5)     // no values between 1 and 5 (acquisition below 5 at full speed possible)
          acquisition.setMicrosPerPoint = 5;
      }
      else {
        acquisition.setMicrosPerPoint = constrain(acquisition.setMicrosPerPoint / 2, minMicrosPerPoint, maxMicrosPerPoint);
          if (acquisition.setMicrosPerPoint < 5)
          acquisition.setMicrosPerPoint = 1;       // no values between 1 and 5 (acquisition below 5 at full speed possible)
      }
      acquisition.setTime = (float)acquisition.setMicrosPerPoint * (float)acquisition.nPoints / 1000.0;
      acquisition.actualMicrosPerPoint = (float)acquisition.setMicrosPerPoint;
      displayButton(1, true, false);
      displayButton(2, false, false);
      displayParameter(3, false);
      disp.firstPoint = 0;
      disp.time = acquisition.setTime;
      calcDependentDisplayParams();
      displayButton(3, false, false);
      displayButton(4, false, false);
    }
  }
  
  else if (buttonState[3]) {                     // FOURTH BUTTON SET (DISPLAY TIME)
    rotary = rotary_getchange();
    if (rotary) {
      if (rotary > 0)
        disp.pointsPerPixel = ((3 * disp.pointsPerPixel) + 1) / 2;
      else
        disp.pointsPerPixel = (2 * disp.pointsPerPixel) / 3;
      if (adcMode)
        disp.pointsPerPixel = constrain(disp.pointsPerPixel, 1, BUFFSIZE / disp.hPixels);
      else
        disp.pointsPerPixel = constrain(disp.pointsPerPixel, 1, (BUFFSIZE * 2) / disp.hPixels);
      pointsOnDisplay = disp.pointsPerPixel * disp.hPixels;
      if (rotary < 0) {
        // try to expand around cursor position
        disp.firstPoint = mainCursor.point - (pointsOnDisplay / 2);
      }
      disp.firstPoint = constrain(disp.firstPoint, 0, acquisition.nPoints - pointsOnDisplay);
      calcDependentDisplayParams();     
      displayButton(3, true, false);
      displayButton(4, false, false);
      displayResult();
      displayMainCursor(true); // redraw cursors at old value, if within limits
      displaySecondCursor(true);
    }
  }
  
  else if (buttonState[4]) {                       // FIFTH BUTTON SET (DISPLAY START)
    rotary = rotary_getchange();
    pointsOnDisplay = disp.pointsPerPixel * disp.hPixels;
    if (rotary) {
      disp.firstPoint = constrain( disp.firstPoint + (pointsOnDisplay * rotary / 10), 0, acquisition.nPoints - pointsOnDisplay);
      calcDependentDisplayParams();
      displayButton(4, true, false);
      displayResult();
      displayMainCursor(true); // redraw cursors at old value, if within limits
      displaySecondCursor(true);
    }
  }
  else                                            // NO BUTTON SET, ROTARY DECODER MOVES CURSOR
    displayMainCursor(false);                     // Check for cursor movement and redraw cursor at new value
  button = checkButtonTap();
  
  if (button >= 0) {                              // Check for touched buttons
    if (button == 2) {                            // SCAN button
      changeButtonState(2);
      Serial.println("\nStart acquisition");
      acquire_data();   
      Serial.println("Acquisition complete");
#ifdef DUMPDATA
      dump_data();
#endif    
      changeButtonState(2);
      displayResult();
      displaySecondCursor(true);    
    }
    else {
      changeButtonState(button);
      delay(300);    
    }
  }
  if (!rotary_switch(0)) {
    // Serial.println("Rotary button pressed");
    displaySecondCursor(false);
  }
  if (adcMode != rotary_switch(4))
    setup();
  if (!adcMode) {
    if (i2cMode != rotary_switch(1))
      setup();
  }
}
