// ***************************************************************
// Rotary.ino
// Reads the Rotary Encoder
// Reads the rotary switch and 4 other switches
// Software debouncing, no interrups
// ***************************************************************

// rricharz 2016

// Arduino pins used
const int SWITCH[]  =    {27, 26, 24, 22, 28};
#define ENCODER_CHA       23
#define ENCODER_CHB       25

int prevA;
int prevResult;

// *******************************************************************

void rotary_setup() {
  
  for (int i = 0; i < (sizeof(SWITCH)/sizeof(int)); i++)
    pinMode(SWITCH[i], INPUT_PULLUP);
    
  pinMode(ENCODER_CHA, INPUT_PULLUP);
  pinMode(ENCODER_CHB, INPUT_PULLUP);
  prevA = LOW;
}

// ********************************************************************

long rotary_getchange(void) {
  int result = 0;
  int a = 1;
  int b = 1;
  uint32_t startTime = millis();
  while (millis() < startTime + 5) {            // software debounce during 5 msec
    a = a & digitalRead(ENCODER_CHA);
    b = b & digitalRead(ENCODER_CHB);
  }
  if ((prevA == HIGH) && (a == LOW)) {
    if (b == HIGH)
      result = 1;
    else
      result = -1;
  }
  if ((result * prevResult) < 0)                // debouncing: no direction change without pause
      result = 0;
  prevA = a;
  prevResult = result;
  // if (result) { Serial.print("Rotary encoder result = "); Serial.println(result); }
  return result;
}

// ********************************************************************

boolean rotary_switch(int switchNumber) {
  if ((switchNumber >= 0) && (switchNumber < (sizeof(SWITCH)/sizeof(int))))
    return digitalRead(SWITCH[switchNumber]);
  else
    return false;
}

// ********************************************************************



