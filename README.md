# rp2040suntracker
Sun Tracking System
Setting the RP2040zSunTracker.ino file, IDE and libraries:
get the Arduino IDE1 **v1.8.19** (NOT THE IDE2): 
https://www.arduino.cc/en/software
get and install board earlephilhower pico library:
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
install Raspberry Pi RP2040 boards(v3.7.2) and Select Raspberry Pi Pico (firth line).
get and install libraries:  
DS3231 : https://github.com/NorthernWidget/DS3231
Adafruit_NeoPixel : https://github.com/adafruit/Adafruit_NeoPixel
Adafruit-GFX : https://github.com/adafruit/Adafruit-GFX-Library
Adafruit-ST7735 : https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp
AccelStepper : http://www.airspayce.com/mikem/arduino/AccelStepper/
AT24C256 eeprom : https://github.com/dantudose/AT24C256
DM556 parameters (1.8°, 200 steps/rev, Pulse +, Direction +, Enable +, neg - common gnd.)
µsteps : 25600 pulse/rev (SW5 off, SW6 off, SW7 off, SW8 on). 
Current Amps 3.2 amps : (SW1 off, SW2 off, SW3 on). Half Current on : (SW4 off).  
firth set your Latitude and Longitude end on line 336 of RP2040zSunTracker.ino file.
all other set can be via BLE.
RP2040zSuntracker v1.2 Board jpeg
![image](https://github.com/user-attachments/assets/3cad70b5-b918-4e9c-ba21-e61c066d7ed4)
