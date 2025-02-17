/*(c) Mechanical Moon LLC, All rights reserved.
  Two axis Azimut/Elevation Rx/Tx Suntracker with GPS.
  (RP2040-Zero) Joy jan 2022. updated jan 2025.
  Thanks to libraries used:
  earlephilhower pico library: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
  Helios : https://os.mbed.com/users/acracan/code/Helios/
  Adafruit_NeoPixel : https://github.com/adafruit/Adafruit_NeoPixel
  Adafruit-GFX : https://github.com/adafruit/Adafruit-GFX-Library
  Adafruit-ST7735 : https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp
  AccelStepper : http://www.airspayce.com/mikem/arduino/AccelStepper/
  AT24C256 eeprom : https://github.com/dantudose/AT24C256
  GPS : https://github.com/mikalhart/TinyGPSPlus
  Two Stepper axis NMRV050 1/100 WormReducers Azimut/Elevation Suntracker control with Helios Algorithm.
  DM556 Nema34 (1.8° 200 steps/rev) stepper driver (Pulse +, Direction +, Enable +, neg - common gnd.)
  Settings on the DM556 : µsteps : 25600 pulse/rev (SW5 off, SW6 off, SW7 off, SW8 on).
  Current Amps 3.2 amps : (SW1 off, SW2 off, SW3 on). Half Current on : (SW4 off).
  RP2040-Zero controller(Pico) on PicozSuntracker Board v1.2.*/
#include <Arduino.h>
#include <Wire.h>
#include "PioSPI.h"
PioSPI spiBus(4, 3, 2, 5, SPI_MODE3, 10000000); //MOSI, MISO, SCK, CS, SPI_MODE, FREQUENCY
#include "at24c256.h"
at24c256 eeprom(0x50);
#define STEPPER1_STEP_PIN 26  // X Pulse
#define STEPPER1_DIR_PIN  11  // X Direction
#define STEPPER2_STEP_PIN 28  // Y Pulse 
#define STEPPER2_DIR_PIN   7  // Y Direction
const int ENABLE1 = 27;       // X Enable
const int ENABLE2 = 29;       // Y Enable
const int SWX = 15;           // X Limit switch
const int SWY = 14;           // Y Limit switch
#include <AccelStepper.h>
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, 16, NEO_GRB + NEO_KHZ800);
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#define BK       3
#define TFT_CS   5
#define TFT_RST 10
#define TFT_DC   6
#define TFT_MOSI 4
#define TFT_SCLK 2
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#include <Helios.h>
Helios helios;                // sun/earth positioning calculator
double dAzimuth;              // Azimut angle
double dElevation;            // Elevation angle
#include <TinyGPSPlus.h>      // GPS
TinyGPSPlus gps;
int satellites;
bool enableRefreshDisplay = true;
uint32_t refresh_previousTime = 0;
uint32_t refresh_currentTime = 0;
unsigned int refresh_sleepInterval = 1500; // value in ms between display refresh
unsigned int AzOffset;        // Azimuth Offset
unsigned int ElOffset;        // Elevation Offset
unsigned int Ang;             // Start/End elevation angle
unsigned int HoHour;          // Daily homing Hour
unsigned int xReach;
unsigned int yReach;
unsigned int lastx = 0;
unsigned int lasty = 0;
int Status = 0;          // Day/Night
int StatusPos = 0;     // Run,Home, Vertical, Wind position (0,1,2,3)
char blop;
String input = "";
void setup() {
  Serial.begin(9600);   // usb
  Serial1.begin(9600);  // GPS
  Serial2.begin(9600);  // BLE
  pinMode(STEPPER1_STEP_PIN, OUTPUT);
  pinMode(STEPPER1_DIR_PIN, OUTPUT);
  pinMode(STEPPER2_STEP_PIN, OUTPUT);
  pinMode(STEPPER2_DIR_PIN, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  pinMode(SWX, INPUT);
  pinMode(SWY, INPUT);
  pinMode(BK, OUTPUT);
  digitalWrite(TFT_RST, HIGH);
  delay(1);
  digitalWrite(TFT_RST, LOW);
  delay(1);
  digitalWrite(TFT_RST, HIGH);
  delay(1);
  digitalWrite(BK, HIGH);
  spiBus.begin();
  spiBus.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
  spiBus.endTransaction();
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.setClock(400000);
  Wire.begin();
  eeprom.init();
  AzOffset = eeprom.read(30000);
  ElOffset = eeprom.read(30001);
  Ang = eeprom.read(30004);
  HoHour = eeprom.read(30006);
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(1, 0, 0));
  pixels.show();
  tft.initR(INITR_MINI160x80_PLUGIN);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 5);
  tft.setTextSize(1 );
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(1, 0);
  tft.print("PicoSunTracker GPS");
  delay(2000);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(1, 20);
  tft.print("By Joy.");
  delay(2000);
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(50, 50, 50));
  pixels.show();
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(1, 30);
  tft.setTextSize(1 );
  tft.print("Homing...");
  delay(1000);
  stepper1.setMaxSpeed(50000.0);
  stepper1.setSpeed(50000.0);
  stepper1.setAcceleration(5000.0);
  stepper2.setMaxSpeed(50000.0);
  stepper2.setSpeed(50000.0);
  stepper2.setAcceleration(5000.0);
  delay(100);
  digitalWrite(ENABLE1, LOW);
  digitalWrite(ENABLE2, LOW);
  delay(100);
  XHoming();                      // Homing...
  delay(1000);
  YHoming();
  checkGPS();
  delay(60000);
  tft.fillScreen(ST77XX_BLACK);
}
void checkGPS() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    satellites = gps.satellites.value();
  }
}
void refreshDisplay() {
  if (!enableRefreshDisplay) {
    return;
  }
  refresh_currentTime = millis();
  if (refresh_currentTime - refresh_previousTime >= refresh_sleepInterval) {
    checkComms();
    displayTime();
    displayNow();
    track();
    refresh_previousTime = millis();
  }
}
void XHoming() {
  stepper2.setAcceleration(5000.0);
  stepper2.moveTo(192000);       // Elevation 45°
  stepper2.runToPosition();
  stepper1.setAcceleration(50000.0);
  stepper1.moveTo(-3000000);
  while (digitalRead(SWX) == HIGH)
    stepper1.run();
  stepper1.stop();
  stepper1.runToPosition();
  stepper1.setCurrentPosition(0);
  stepper1.setAcceleration(5000.0);
  stepper1.moveTo(640000);   // 180° sud
  stepper1.runToPosition();
}
void YHoming() {
  stepper2.setAcceleration(50000.0);
  stepper2.moveTo(-3000000);
  while (digitalRead(SWY) == HIGH)
    stepper2.run();
  stepper2.stop();
  stepper2.runToPosition();
  stepper2.setCurrentPosition(0);
  stepper2.setAcceleration(5000.0);
  stepper2.moveTo(192000);  //  Elevation 45°
  stepper2.runToPosition();
}
void Vertical() {                        // Home position
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(102, 40);
  tft.print("640000");
  tft.setCursor(102, 50);
  tft.print("0");
  stepper1.moveTo(640000);      // Azimut 180°(Sud)
  stepper1.runToPosition();
  stepper2.moveTo(0);       // Elevation 9°
  stepper2.runToPosition();
}
void Home() {                        // Home position
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(102, 40);
  tft.print("640000");
  tft.setCursor(102, 50);
  tft.print("256000");
  stepper1.moveTo(640000);      // Azimut 180°(Sud)
  stepper1.runToPosition();
  stepper2.moveTo(256000);       // Elevation 45°
  stepper2.runToPosition();
}
void wind() {                        // Night Wind position
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(102, 40);
  tft.print("640000");
  tft.setCursor(102, 50);
  tft.print("384000");
  stepper1.moveTo(640000);      // Azimut 180°(Sud)
  stepper1.runToPosition();
  stepper2.moveTo(384000);       // Elevation 67,5°
  stepper2.runToPosition();
}
void track() {
  helios.calcSunPos(gps.date.year() / 100, gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second(), gps.location.lng(), gps.location.lat());
  dAzimuth = helios.dAzimuth;
  dElevation = helios.dElevation;
  float ElReach = ((dElevation * 64000 / 9) + (ElOffset * 1000) - 100000) - 64000; // dElevation * (25600steps/rev * 100ratioreduction) / 360°) + Offsett - Limit minimum elevation (64000 = 9°) = 7111,111 µsteps / degree.
  float AzReach = ((dAzimuth * 64000 / 9) + (AzOffset * 1000) - 100000) - 640000; // dAzimuth * (25600steps/rev * 100ratioreduction) / 360°) + Offsett - Limit minimum azimuth (640000 = 90°) = 7111,111 µsteps / degree.
  if (ElReach >= 512000) {   // Limit maximum Elevation (81°)
    ElReach = 512000;
  }
  if (ElReach <= 0) {        // Limit minimum Elevation (9°)
    ElReach = 0;
  }
  if (AzReach >= 1280000) {  // Limit maximum Azimuth (270°ouest)
    AzReach = 1280000;
  }
  if (AzReach <= 0) {        // Limit maximum Azimuth (270°ouest)
    AzReach = 0;
  }
  if (dElevation >= Ang)  {              // jour
    Status = 1;
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 1, 0));
    pixels.show();
  }
  if (dElevation < Ang)  {              // nuit
    Status = 0;
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(1, 0, 0));
    pixels.show();
  }
  xReach = AzReach - lastx;
  yReach = ElReach - lasty;
  if (Status == 1)
    if (StatusPos == 0) {
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(3, 3, 0));
      pixels.show();
      tft.setTextSize(1);
      tft.setCursor(124, 14);
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      tft.print("Jour");
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(102, 40);
      tft.print(AzReach, 0);
      tft.setCursor(102, 50);;
      tft.print(ElReach, 0);
      stepper1.moveTo(xReach);
      stepper1.runToPosition();
      delay(10);
      lastx == AzReach;
      stepper2.moveTo(yReach);
      stepper2.runToPosition();
      delay(10);
      lasty == ElReach;
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 1, 0));
      pixels.show();
    }
  if (Status == 0)  {
    tft.setCursor(124, 14);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("Nuit");
    if (StatusPos == 0) {
      wind();
    }
  }
}
void displayTime() {
  tft.setTextSize(1);
  tft.setCursor(10, 0);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(gps.date.day());
  tft.print(" ");
  switch (gps.date.month()) {
    case 1:
      tft.print("Janvier ");
      break;
    case 2:
      tft.print("Fevrier ");
      break;
    case 3:
      tft.print("Mars ");
      break;
    case 4:
      tft.print("Avril ");
      break;
    case 5:
      tft.print("Mai ");
      break;
    case 6:
      tft.print("Juin ");
      break;
    case 7:
      tft.print("Juillet ");
      break;
    case 8:
      tft.print("Aout ");
      break;
    case 9:
      tft.print("Septembre ");
      break;
    case 10:
      tft.print("Octobre ");
      break;
    case 11:
      tft.print("Novembre ");
      break;
    case 12:
      tft.print("Decembre ");
      break;
    default: break;
  }
  tft.print(gps.date.year(), DEC);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.setCursor(18, 10);
  if (gps.time.hour() <= 9)  {
    tft.print("0");
  }
  tft.print(gps.time.hour()); // GMT
  tft.print(":");
  if (gps.time.minute() <= 9)  {
    tft.print("0");
  }
  tft.print(gps.time.minute());
  tft.print(":");
  if (gps.time.second() <= 9)  {
    tft.print("0");
  }
  tft.print(gps.time.second());
}
void displayNow() {
  tft.setTextSize(1);
  tft.setCursor(2, 28);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(gps.location.lat(), 5);
  tft.setCursor(52, 28);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("N");
  tft.setCursor(68, 28);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(gps.location.lng(), 5);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(112, 28);
  tft.print("E");
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(118, 0);
  tft.print("Sat ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (satellites < 10) {
    tft.print(" ");
  }
  tft.print(satellites);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(1, 40);
  tft.print("Azim ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(dAzimuth, 2);
  tft.print("  ");
  tft.setCursor(1, 50);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Elev  ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(dElevation, 2);
  tft.print("  ");
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(124, 28);
  tft.print(analogReadTemp(), 1);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" C");
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(60, 60);
  tft.print(" Angle Start ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(Ang);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(1, 60);
  tft.print("AzOf ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(AzOffset);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(1, 70);
  tft.print("ElOf ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(ElOffset);
  tft.setCursor(60, 70);
  tft.setTextColor(ST77XX_ORANGE);
  tft.print("Homing hour ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (HoHour <= 9) {
    tft.print(" ");
  }
  if (HoHour >= 24) {
    tft.print("  no");
  }
  tft.print(HoHour);
}
void checkComms() {
  if (Serial2.available())                   //   BLE control
    while (Serial2.available() > 0) {
      char inChar = (char)Serial2.read();
      input += inChar;
      pixels.clear();
      for (int i = 0; i < 1; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 100));
        pixels.show();
      }
    }
  if (input.length() >= 1) {
    {
      blop = Serial2.read();
    }
    if (input == "olon") {                   // Oled on
      digitalWrite(BK, HIGH);
      Serial2.println("");
      Serial2.println("LCD ON");
    }
    if (input == "olof") {                   // Oled off
      digitalWrite(BK, LOW);
      Serial2.println("");
      Serial2.println("LCD OFF");
    }
    if (input == "wind") {                  // Homing
      StatusPos = 1;
      Serial2.println("");
      Serial2.print("wind...");
      wind();
      Serial2.println("");
      Serial2.print("Wind position.");
    }
    if (input == "home") {                  // Homing
      StatusPos = 1;
      Serial2.println("");
      Serial2.println("Homing...");
      XHoming();
      delay(1000);
      YHoming();
      delay(1000);
      Home();
      Serial2.println("");
      Serial2.print("Home position.");
      Serial2.println("");
    }
    if (input == "vert") {                  // Homing
      StatusPos = 1;
      Serial2.println("");
      Serial2.println("Vertical...");
      Vertical();
      Serial2.println("");
      Serial2.print("Vertical position.");
      Serial2.println("");
    }
    if (input == "star") {                  // Restart
      StatusPos = 0;
      Serial2.println("");
 /*   stepper2.setAcceleration(5000.0);
      stepper2.moveTo(192000);       // Elevation 45°
      stepper2.runToPosition();*/
      Serial2.println("Restart...");
      delay(1000);
      Serial2.println("");
      Serial2.print("Running.");
      Serial2.println("");
    }
    if (input == "rsto") {                 // Reset setting
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("azoffset = ");
      AzOffset = 100;
      eeprom.write(30000, 100);
      Serial2.print(AzOffset, DEC);
      Serial2.println("");
      Serial2.print("eloffset = ");
      ElOffset = 100;
      eeprom.write(30001, 100);
      Serial2.print(ElOffset, DEC);
      Serial2.println("");
      Serial2.print("Angle = ");
      Ang = 5;
      eeprom.write(30004, 5);
      Serial2.print(Ang, DEC);
      Serial2.println("");
      Serial2.print("Homing Hour: ");
      HoHour = 2;                // monthly homing Hour (2 heures)
      eeprom.write(30006, 2);
      Serial2.print(HoHour, DEC);
      Serial2.println("");
    }
    if (input == "HoH+") {                   // Weekly homing Hour +
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("Homing Hour: ");
      HoHour = HoHour + 1;
      Serial2.println(HoHour, DEC);
      eeprom.write(30006, HoHour);
    }
    if (input == "HoH-") {                   // Weekly homing Hour -
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("Homing Hour: ");
      HoHour = HoHour - 1;
      Serial2.println(HoHour, DEC);
      eeprom.write(30006, HoHour);
    }
    if (input == "azo+") {                   // Azimuth Offset +
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("azoffset = ");
      AzOffset = AzOffset + 1;
      Serial2.println(AzOffset, DEC);
      eeprom.write(30000, AzOffset);
    }
    if (input == "azo-") {                   // Azimuth Offset -
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("azoffset = ");
      AzOffset = AzOffset - 1;
      Serial2.println(AzOffset, DEC);
      eeprom.write(30000, AzOffset);
    }
    if (input == "elo+") {                   // Elevation Offset +
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("eloffset = ");
      ElOffset = ElOffset + 1;
      Serial2.println(ElOffset, DEC);
      eeprom.write(30001, ElOffset);
    }
    if (input == "elo-") {                   // Elevation Offset -
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("eloffset = ");
      ElOffset = ElOffset - 1;
      Serial2.println(ElOffset, DEC);
      eeprom.write(30001, ElOffset);
    }
    if (input == "ang+") {                   // Elevation Angle  +
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("angle = ");
      Ang = Ang + 1;
      Serial2.println(Ang, DEC);
      eeprom.write(30004, Ang);
    }
    if (input == "ang-") {                   // Elevation Angle  -
      tft.fillRect(0, 54, 160, 26, ST77XX_BLACK);
      Serial2.println("");
      Serial2.print("angle = ");
      Ang = Ang - 1;
      Serial2.println(Ang, DEC);
      eeprom.write(30004, Ang);
    }
    if (input == "ctrl") {
      Serial2.println(" ");
      Serial2.print(gps.date.day());
      Serial2.print(" ");
      if (gps.date.month() == 1 ) {
        Serial2.print("janvier ");
      }
      if (gps.date.month() == 2 ) {
        Serial2.print("fevrier ");
      }
      if (gps.date.month() == 3 ) {
        Serial2.print("mars ");
      }
      if (gps.date.month() == 4 ) {
        Serial2.print("avril ");
      }
      if (gps.date.month() == 5 ) {
        Serial2.print("mai ");
      }
      if (gps.date.month() == 6 ) {
        Serial2.print("juin ");
      }
      if (gps.date.month() == 7 ) {
        Serial2.print("juillet ");
      }
      if (gps.date.month() == 8 ) {
        Serial2.print("aout ");
      }
      if (gps.date.month() == 9 ) {
        Serial2.print("septembre ");
      }
      if (gps.date.month() == 10 ) {
        Serial2.print("octobre ");
      }
      if (gps.date.month() == 11 ) {
        Serial2.print("novembre ");
      }
      if (gps.date.month() == 12 ) {
        Serial2.print("decembre ");
      }
      Serial2.print(gps.date.year());
      Serial2.println("");
      Serial2.print(gps.time.hour());
      Serial2.print(":");
      Serial2.print(gps.time.minute());
      Serial2.print(":");
      Serial2.print(gps.time.second());
      Serial2.print(" ut ");
      if (Status == 0)  {
        Serial2.println(" nuit ");
      }
      if (Status == 1)  {
        Serial2.println(" jour ");
      }
      Serial2.print("temperature : ");
      Serial2.print(analogReadTemp(), 1);
      Serial2.println("C");
      Serial2.print("longitude: ");
      Serial2.print(gps.location.lng(), 5);
      Serial2.println(" Est");
      Serial2.print("latitude : ");
      Serial2.print(gps.location.lat(), 5);
      Serial2.println(" Nord");
      Serial2.print("azimuth  : ");
      Serial2.println(dAzimuth, 2);
      Serial2.print("elevation: ");
      Serial2.println(dElevation, 2);
      Serial2.print("azoffset ");
      Serial2.println(AzOffset);
      Serial2.print("eloffset ");
      Serial2.println(ElOffset);
      Serial2.print("angle start: ");
      Serial2.println(Ang);
      Serial2.print("homing hour: ");
      if (HoHour < 24) {
        Serial2.println(HoHour);
      }
      if (HoHour >= 24) {
        Serial2.println(" no");
      }
    }
  }
  input = "";
}
void loop() {
  checkGPS();
  refreshDisplay();
}
