# Logic and Analog recorder using an Arduino Mega 2560

How fast can you record data with an Arduino Mega 2560 ?

This sketch uses direct port read and bit manipulations to record either
- 4 digital channels at a maximum speed of approx. 900 kHz (7644 data points) 
- 1 analog channel (8 bits) at a maximum speed of 40 kHz (3822 data points)

The data is displayed and manipulated on a touch tft screen.

![Alt text](box.jpg?raw=true)

*An example of i2c communication (A = SDA, B = SCL, C = not used)*

The following hardware is used:
- Arduino Mega 2560 board
- Adafruit 3.5" 320x480 Color TFT Touchscreen breakout
- Snootlab Rotary Encoder Kit with switch
- One additional switch

The following pins on the Arduino Mega 2560 are used
- Pins 30 - 33 for digital input. Pin 30 is used for the trigger, Pin 33 is recorded
but not dispayed. The program sets these pins to INPUT_PULLUP.
- Pin A5 is used for analog input.
- TFT dispay uses pin 9 for TFT_DC, pin 10 for TFT_CS, pin 46 for TFT_LITE (dimm),
and pin 4 for CARD_CS
- The Touch pannel uses pin A2 for YP, pin A3 for XM, pin 7 for YM and pin 8 for XP
- The rotary encoder uses pin 23 for CHA, 25 for CHB and 27 for the switch
- 4 switches hooked to pin 26, 24, 22, 28 

Using the sketch:
- Set the switch to analog or digial mode.
- There are 4 red fields which display values for the data display
- There are 5 blue touch buttons
- Touch **TRIGGER TYPE** and select *change*, *falling*, *rising* or *none* with the rotary encoder
- Touch **TIME PER POINT** and select a time with the rotary encoder, the total acquisition time is
calculated and displayed
- Touch **START SCAN** to start acquiring data. The data acquisition is started as soon as
the trigger condition is met.
- Once the data has been acquired, you can change **DISPAY TIME** and **DISPLAY START**
- A cursor is displayed to measure a time **CURSOR TIME**.
- With a push of the rotary button, a second cursor can be displayed and moved around
(**2nd CURSOR** and **DIFFERENCE**).

Please help to improve this program by tweeting to
**http://twitter.com/r_richarz** or opening an issue on this repository
if you have any problem or suggestion.