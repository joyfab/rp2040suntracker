/*(c)Joyfab. By Joy. jan 2025.                  
  Thanks to libraries used for:
  earlephilhower pico library: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
  Helios : https://os.mbed.com/users/acracan/code/Helios/
  DS3231 : https://github.com/NorthernWidget/DS3231
  Adafruit_NeoPixel : https://github.com/adafruit/Adafruit_NeoPixel
  Adafruit-GFX : https://github.com/adafruit/Adafruit-GFX-Library
  Adafruit-ST7735 : https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp
  AccelStepper : http://www.airspayce.com/mikem/arduino/AccelStepper/
  AT24C256 eeprom : https://github.com/dantudose/AT24C256 
  Two Stepper axis NMRV050 1/100 WormReducers Azimut/Elevation Suntracker control with Helios Algorithm.
  DM556 Nema34 (1.8° 200 steps/rev) stepper driver (Pulse +, Direction +, Enable +, neg - common gnd.) 
  Settings on the DM556 : µsteps : 25600 pulse/rev (SW5 off, SW6 off, SW7 off, SW8 on). 
  Current Amps 3.2 amps : (SW1 off, SW2 off, SW3 on). Half Current on : (SW4 off). 
  RP2040-Zero controller(Pico) on PicozSuntracker Board v1.2.*/     
#include <Arduino.h>
#include <Wire.h>
#include "PioSPI.h"           // Pin SPI for rp2040z
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
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#define BK       3
#define TFT_CS   5
#define TFT_RST 10
#define TFT_DC   6
#define TFT_MOSI 4
#define TFT_SCLK 2
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, 16, NEO_GRB + NEO_KHZ800);
#include <DS3231.h>           // RTC
DS3231 Clock;
bool Century = false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;
byte year, month, date, DoW, hour, minute, second;
#include <Helios.h>
Helios helios;                // sun/earth positioning calculator
double dAzimuth;              // Azimut angle
double dElevation;            // Elevation angle
unsigned int AzOffset;        // Azimuth Offset
unsigned int ElOffset;        // Elevation Offset
unsigned int Ang;             // Start/End elevation angle
unsigned int HoDow;           // Weekly homing DoW
unsigned int HoHour;          // Weekly homing Hour
unsigned int xReach;
unsigned int yReach;
unsigned int lastx = 0;
unsigned int lasty = 0;
int Status = 0;               // Day/Night
char blop;
String input = "";
void setup() {
  Serial.begin(9600);         // USB
  Serial1.begin(9600);        // GPS
  Serial2.begin(9600);        // BLE
  pinMode(STEPPER1_STEP_PIN, OUTPUT);
  pinMode(STEPPER1_DIR_PIN, OUTPUT);
  pinMode(STEPPER2_STEP_PIN, OUTPUT);
  pinMode(STEPPER2_DIR_PIN, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  pinMode(SWX, INPUT);
  pinMode(SWY, INPUT);
  pinMode(BK, OUTPUT);
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.setClock(400000);
  Wire.begin();
  /* Clock.setSecond(30);   //Set the second
     Clock.setMinute(7);    //Set the minute
     Clock.setHour(11);     //Set the hour
     Clock.setDoW(4);       //Set the day of the week
     Clock.setDate(20);     //Set the date of the month
     Clock.setMonth(11);    //Set the month of the year
     Clock.setYear(24);     //Set the year (Last two digits of the year)
  */
  eeprom.init();
  AzOffset = eeprom.read(30000);
  ElOffset = eeprom.read(30001);
  Ang = eeprom.read(30004);
  HoDow = eeprom.read(30005);
  HoHour = eeprom.read(30006);
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(1, 0, 0));
  pixels.show();
  digitalWrite(TFT_RST, HIGH); // reset tft
  delay(1);
  digitalWrite(TFT_RST, LOW);
  delay(1);
  digitalWrite(TFT_RST, HIGH);
  delay(1);
  digitalWrite(BK, HIGH);
  spiBus.begin();
  spiBus.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
  spiBus.endTransaction();
  tft.initR(INITR_MINI160x80_PLUGIN);  // ST7735160x80 tft spi set
  tft.setRotation(3);                  // tft rotation set
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 5);
  tft.setTextSize(1 );
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(1, 0);
  tft.print("PicozSunTracker");
  delay(3000);
  tft.setCursor(1, 10);
  tft.setTextColor(ST77XX_GREEN);
  tft.print("RTC");
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(1, 20);
  tft.print("By Joy.");
  delay(2000);
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(10, 10, 10));
  pixels.show();
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(1, 30);
  tft.setTextSize(1 );
  tft.print("Homing...");
  delay(1000);
  stepper1.setMaxSpeed(40000.0);
  stepper1.setSpeed(40000.0);
  stepper1.setAcceleration(10000.0);
  stepper2.setMaxSpeed(40000.0);
  stepper2.setSpeed(40000.0);
  stepper2.setAcceleration(10000.0);
  delay(100);
  digitalWrite(ENABLE1, LOW);
  digitalWrite(ENABLE2, LOW);
  delay(100);
  XHoming();                         // Homing at start...
  delay(1000);
  YHoming();
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(1, 12);
  tft.print("Lon: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" 2.34839");             // Paris Longitude Est
  tft.setTextColor(ST77XX_CYAN);
  tft.print(" E");
  tft.setCursor(1, 22);
  tft.print("Lat: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print("48.85349");             // Paris Latitude Nord
  tft.setTextColor(ST77XX_CYAN);
  tft.print(" N");
}
void XHoming() {
  stepper1.setAcceleration(40000.0);
  stepper1.moveTo(-3000000);
  while (digitalRead(SWX) == HIGH)
    stepper1.run();
  stepper1.stop();
  stepper1.runToPosition();
  stepper1.setCurrentPosition(0);
  stepper1.moveTo(1000000);
  while (digitalRead(SWX) == LOW)
    stepper1.run();
  stepper1.stop();
  stepper1.runToPosition();
  stepper1.setCurrentPosition(0);
  stepper1.setAcceleration(10000.0);
  stepper1.moveTo(0);
  stepper1.runToPosition();
}
void YHoming() {
  stepper2.setAcceleration(40000.0);
  stepper2.moveTo(-3000000);
  while (digitalRead(SWY) == HIGH)
    stepper2.run();
  stepper2.stop();
  stepper2.runToPosition();
  stepper2.setCurrentPosition(0);
  stepper2.moveTo(1000000);
  while (digitalRead(SWY) == LOW)
    stepper2.run();
  stepper2.stop();
  stepper2.runToPosition();
  stepper2.setCurrentPosition(0);
  stepper2.setAcceleration(10000.0);
  stepper2.moveTo(0);
  stepper2.runToPosition();
}
void wind() {                        // Night Wind position
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(105, 33);
  tft.print("640000");
  tft.setCursor(105, 43);
  tft.print("320000");
  stepper1.moveTo(640000);          // Azimut 180°(Sud)
  stepper1.runToPosition();
  stepper2.moveTo(320000);           // Elevation 45°
  stepper2.runToPosition();
}
void track() { // for Paris:long 2°20'54.20"E,lat 45°51'12.58"N => 2.34839,48.85349 decimal. keep 5 decimal. see https://www.coordonnees-gps.fr/conversion-coordonnees-gps
  helios.calcSunPos(Clock.getYear(), Clock.getMonth(Century), Clock.getDate(), Clock.getHour(h12, PM), Clock.getMinute(), Clock.getSecond(), 2.34839, 48.85349);
  dAzimuth = helios.dAzimuth;
  dElevation = helios.dElevation;
  float ElReach = ((dElevation * 64000 / 9) + (ElOffset * 1000) - 100000) - 32000; // dElevation * (25600 steps/rev * 100 ratio reduction) / 360°) + Offsett - Limit minimum
  float AzReach = ((dAzimuth * 64000 / 9) + (AzOffset * 1000) - 100000) - 512000 ; // = 7111,1111 µsteps / degree.
  if (ElReach <= 0) {        // Limit neg Elevation (0°)
    ElReach = 0;
  }
  if (ElReach >= 600000) {   // Limit maximum Elevation (85°)
    ElReach = 600000;
  }
  if (AzReach >= 2240000) {  // Limit maximum Azimuth (315°ouest)
    AzReach = 2240000;
  }
  if (dElevation >= Ang)  {              // jour
    Status = 1;
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 1, 0));
    pixels.show();
    tft.setCursor(106, 11);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.print(" Jour");
  }
  if (dElevation < Ang)  {              // nuit
    Status = 0;
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(1, 0, 0));
    pixels.show();
    tft.setCursor(106, 11);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print(" Nuit");
    wind();
  }
  xReach = AzReach - lastx;
  yReach = ElReach - lasty;
  if (Status == 1)  {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(105, 33);
    tft.print(AzReach, 0);
    tft.setCursor(105, 43);
    tft.print(ElReach, 0);
    if (Clock.getSecond() >= 0) {
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 0, 5));
      pixels.show();
      stepper1.moveTo(xReach);
      stepper1.runToPosition();
      delay(20);
      lastx == AzReach;
      stepper2.moveTo(yReach);
      stepper2.runToPosition();
      delay(20);
      lasty == ElReach;
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 1, 0));
      pixels.show();
    }
  }
  if (HoDow == 0)                // Daily Homing
    if (Clock.getHour(h12, PM) == HoHour)
      if (Clock.getMinute() == 0) {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(0, 0, 5));
        pixels.show();
        XHoming();
        delay(1000);
        YHoming();
        delay(60000);
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(1, 0, 0));
        pixels.show();
      }
  if (Clock.getDoW() == HoDow)   // Weekly Homing
    if (Clock.getHour(h12, PM) == HoHour)
      if (Clock.getMinute() == 0) {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(0, 0, 5));
        pixels.show();
        XHoming();
        delay(1000);
        YHoming();
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(2, 2, 0));
        pixels.show();
        delay(60000);
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(1, 0, 0));
        pixels.show();
      }
}
void displayTime() {
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  switch (Clock.getDoW()) {
    case 1:
      tft.print("Dim ");
      break;
    case 2:
      tft.print("Lun ");
      break;
    case 3:
      tft.print("Mar ");
      break;
    case 4:
      tft.print("Mer ");
      break;
    case 5:
      tft.print("Jeu ");
      break;
    case 6:
      tft.print("Ven ");
      break;
    case 7:
      tft.print("Sam ");
      break;
    default: break;
  }
  tft.print(Clock.getDate());
  tft.print(" ");
  switch (Clock.getMonth(Century)) {
    case 1:
      tft.print("Jan ");
      break;
    case 2:
      tft.print("Fev ");
      break;
    case 3:
      tft.print("Mar ");
      break;
    case 4:
      tft.print("Avr ");
      break;
    case 5:
      tft.print("Mai ");
      break;
    case 6:
      tft.print("Jun ");
      break;
    case 7:
      tft.print("Jui ");
      break;
    case 8:
      tft.print("Aou ");
      break;
    case 9:
      tft.print("Sep ");
      break;
    case 10:
      tft.print("Oct ");
      break;
    case 11:
      tft.print("Nov ");
      break;
    case 12:
      tft.print("Dec ");
      break;
    default: break;
  }
  tft.print("20");
  tft.print(Clock.getYear());
  tft.setCursor(100, 2);
  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  if (Clock.getHour(h12, PM) <= 9)  {
    tft.print("0");
  }
  tft.print(Clock.getHour(h12, PM)); // GMT
  tft.print(":");
  if (Clock.getMinute() <= 9)  {
    tft.print("0");
  }
  tft.print(Clock.getMinute());
  tft.print(":");
  if (Clock.getSecond() <= 9)  {
    tft.print("0");
  }
  tft.print(Clock.getSecond());
  tft.print(" ");
}
void displayNow() {
  if (Clock.getSecond() == 4) {
    tft.setCursor(106, 12);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(106, 22);
    tft.print(Clock.getTemperature(), 1);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" C");
  }
  tft.setCursor(1, 53);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.print("  Angle ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(Ang);
  tft.print(" ");
  tft.setCursor(140, 55);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(1, 64);
  tft.print("AzOffset ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(AzOffset);
  tft.setTextColor(ST77XX_ORANGE);
  tft.setCursor(1, 73);
  tft.print("ElOffset ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(ElOffset);
  tft.setTextColor(ST77XX_ORANGE);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(1, 33);
  tft.print("Azim ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (dAzimuth < 100) {
    tft.print(" ");
  }
  if (dAzimuth < 10) {
    tft.print(" ");
  }
  tft.print(dAzimuth, 2);
  tft.print(" ");
  tft.setCursor(1, 43);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Elev ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (dElevation >= 10) {
    tft.print(" ");
  }
  if (dElevation < 10)
    if (dElevation > 0) {
      tft.print("  ");
    }
  if (dElevation <= 0)
    if (dElevation > -10) {
      tft.print(" ");
    }
  if (dElevation <= -10) {
    tft.print("");
  }
  tft.print(dElevation, 2);
  tft.print(" ");
  tft.setTextColor(ST77XX_ORANGE);
  tft.setCursor(96, 55);
  tft.print(" Homing");
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(96, 64);
  tft.print(" DoW ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (HoDow == 0) {
    tft.print(" no");
  }
  if (HoDow == 1) {
    tft.print("Dim");
  }
  if (HoDow == 2) {
    tft.print("Lun");
  }
  if (HoDow == 3) {
    tft.print("Mar");
  }
  if (HoDow == 4) {
    tft.print("Mer");
  }
  if (HoDow == 5) {
    tft.print("Jeu ");
  }
  if (HoDow == 6) {
    tft.print("Ven ");
  }
  if (HoDow == 7) {
    tft.print("Sam ");
  }
  tft.setCursor(96, 73);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print(" Hour");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (HoHour < 9) {
    tft.print(" ");
  }
  if (HoHour >= 24) {
    tft.print("no ");
  }
  tft.print(HoHour);
}
void checkComms() {
  if (Serial2.available())                   //   BLE control
    while (Serial2.available() > 0) {
      char inChar = (char)Serial2.read();
      input += inChar;
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 0, 100));
      pixels.show();
    }
  if (input.length() >= 1) {
    {
      blop = Serial2.read();
    }
    if (input == "sets") {
      Serial2.println("");
      Serial2.print("Reset second");
      Clock.setSecond(32);
    }
    if (input == "min+") {
      Serial2.println("");
      Serial2.print("Set minute+ : ");
      Clock.setMinute(Clock.getMinute() + 1);
      Serial2.print(Clock.getMinute());
    }
    if (input == "min-") {
      Serial2.println("");
      Serial2.print("Set minute- : ");
      Clock.setMinute(Clock.getMinute() - 1);
      Serial2.print(Clock.getMinute());
    }
    if (input == "olon") {                   // TFT on
      digitalWrite(BK, HIGH);
      Serial2.println("");
      Serial2.println("LCD ON");
    }
    if (input == "olof") {                   // TFT off
      digitalWrite(BK, LOW);
      Serial2.println("");
      Serial2.println("LCD OFF");
    }
    if (input == "home") {                   // Homing
      Serial2.println("");
      Serial2.print("Homing...");
      XHoming();
      delay(1000);
      YHoming();
      Serial2.println("");
      Serial2.print("At home...");
      delay(10000);
      Serial2.println("");
      Serial2.print("Restart");
    }
    if (input == "rsto") {                 // Reset settings
      Serial2.println("");
      Serial2.print("eloffset = ");
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
      Serial2.print("Homing DoW: ");
      HoDow = 2;                       // Weekly homing DoW (lundi)
      eeprom.write(30005, 2);
      Serial2.print(HoDow, DEC);
      Serial2.println("");
      Serial2.print("Homing Hour: ");
      HoHour = 2;                      // Weekly homing Hour (2 heures)
      eeprom.write(30006, 2);
      Serial2.print(HoHour, DEC);
      Serial2.println("");
    }
    if (input == "HoD+") {             // Weekly homing DoW +
      Serial2.println("");
      Serial2.print("Jour de la semaine + ");
      HoDow = HoDow + 1;
      Serial2.println(HoDow, DEC);
      eeprom.write(30005, HoDow);
    }
    if (input == "HoD-") {             // Weekly homing DoW -
      Serial2.println("");
      Serial2.print("Jour de la semaine - ");
      HoDow = HoDow - 1;
      Serial2.println(HoDow, DEC);
      eeprom.write(30005, HoDow);
    }
    if (input == "HoH+") {             // Weekly homing Hour +
      Serial2.println("");
      Serial2.print("Heure de la journee + ");
      HoHour = HoHour + 1;
      Serial2.println(HoHour, DEC);
      eeprom.write(30006, HoHour);
    }
    if (input == "HoH-") {             // Weekly homing Hour -
      Serial2.println("");
      Serial2.print("Heure de la journee - ");
      HoHour = HoHour - 1;
      Serial2.println(HoHour, DEC);
      eeprom.write(30006, HoHour);
    }
    if (input == "azo+") {                   // Azimuth Offset +
      Serial2.println("");
      Serial2.print("azoffset = ");
      AzOffset = AzOffset + 1;
      Serial2.println(AzOffset, DEC);
      eeprom.write(30000, AzOffset);
    }
    if (input == "azo-") {                   // Azimuth Offset -
      Serial2.println("");
      Serial2.print("azoffset = ");
      AzOffset = AzOffset - 1;
      Serial2.println(AzOffset, DEC);
      eeprom.write(30000, AzOffset);
    }
    if (input == "elo+") {                   // Elevation Offset +
      Serial2.println("");
      Serial2.print("eloffset = ");
      ElOffset = ElOffset + 1;
      Serial2.println(ElOffset, DEC);
      eeprom.write(30001, ElOffset);
    }
    if (input == "elo-") {                   // Elevation Offset -
      Serial2.println("");
      Serial2.print("eloffset = ");
      ElOffset = ElOffset - 1;
      Serial2.println(ElOffset, DEC);
      eeprom.write(30001, ElOffset);
    }
    if (input == "ang+") {                   // Elevation Angle  +
      Serial2.println("");
      Serial2.print("angle = ");
      Ang = Ang + 1;
      Serial2.println(Ang, DEC);
      eeprom.write(30004, Ang);
    }
    if (input == "ang-") {                   // Elevation Angle  -
      Serial2.println("");
      Serial2.print("angle = ");
      Ang = Ang - 1;
      Serial2.println(Ang, DEC);
      eeprom.write(30004, Ang);
    }
    if (input == "ctrl") {                  // BLE reading control
      Serial2.println(" ");
      if (Clock.getDoW() == 1 ) {
        Serial2.print("dimanche ");
      }
      if (Clock.getDoW() == 2 ) {
        Serial2.print("lundi ");
      }
      if (Clock.getDoW() == 3 ) {
        Serial2.print("mardi ");
      }
      if (Clock.getDoW() == 4 ) {
        Serial2.print("mercredi ");
      }
      if (Clock.getDoW() == 5 ) {
        Serial2.print("jeudi ");
      }
      if (Clock.getDoW() == 6 ) {
        Serial2.print("vendredi ");
      }
      if (Clock.getDoW() == 7 ) {
        Serial2.print("samedi ");
      }
      Serial2.print(Clock.getDate());
      Serial2.print(" ");
      if (Clock.getMonth(Century) == 1 ) {
        Serial2.print("janvier ");
      }
      if (Clock.getMonth(Century) == 2 ) {
        Serial2.print("fevrier ");
      }
      if (Clock.getMonth(Century) == 3 ) {
        Serial2.print("mars ");
      }
      if (Clock.getMonth(Century) == 4 ) {
        Serial2.print("avril ");
      }
      if (Clock.getMonth(Century) == 5 ) {
        Serial2.print("mai ");
      }
      if (Clock.getMonth(Century) == 6 ) {
        Serial2.print("juin ");
      }
      if (Clock.getMonth(Century) == 7 ) {
        Serial2.print("juillet ");
      }
      if (Clock.getMonth(Century) == 8 ) {
        Serial2.print("aout ");
      }
      if (Clock.getMonth(Century) == 9 ) {
        Serial2.print("septembre ");
      }
      if (Clock.getMonth(Century) == 10 ) {
        Serial2.print("octobre ");
      }
      if (Clock.getMonth(Century) == 11 ) {
        Serial2.print("novembre ");
      }
      if (Clock.getMonth(Century) == 12 ) {
        Serial2.print("decembre ");
      }
      Serial2.print(" 20");
      Serial2.print(Clock.getYear());
      Serial2.println("");
      Serial2.print(Clock.getHour(h12, PM));
      Serial2.print(":");
      Serial2.print(Clock.getMinute());
      Serial2.print(":");
      Serial2.print(Clock.getSecond());
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
      Serial2.print("azimuth  : ");
      Serial2.println(dAzimuth, 2);
      Serial2.print("elevation: ");
      Serial2.println(dElevation, 2);
      Serial2.print("azoffset ");
      Serial2.print(AzOffset);
      Serial2.print(" eloffset ");
      Serial2.println(ElOffset);
      Serial2.print("start/end angle : ");
      Serial2.println(Ang);
      Serial2.print("homing day of week: ");
      Serial2.println(HoDow);
      Serial2.print("homing hour: ");
      Serial2.println(HoHour);
    }
    input = "";
  }
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 1, 0));
  pixels.show();
}
void loop() {
  displayTime();
  displayNow();
  checkComms();
  track();
}
