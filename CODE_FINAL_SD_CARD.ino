// -*- coding: utf-8 -*-

// --- Import Libraries ---
#include <SPI.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <TimeLib.h>

// --- Pin Definitions ---
#define AZM_EN_PIN 11
#define AZM_DIR_PIN 10
#define AZM_STEP_PIN 12
#define ELV_EN_PIN 8
#define ELV_DIR_PIN 7
#define ELV_STEP_PIN 6
#define BUTTON_PIN 41
#define LED_RED_PIN 32
#define LED_YELLOW_PIN 31
#define LED_GREEN_PIN 30

// --- Konfigurasi ---
#define AZM_GEAR_RATIO 2.0
#define ELV_GEAR_RATIO 3.0
#define MAX_CABLE_ROTATION 450.0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// --- Variabel Global ---
// Display
enum DisplayMode {
  DISPLAY_MAIN,
  DISPLAY_SIGNAL,
  DISPLAY_GPS,
  DISPLAY_SYSTEM,
  DISPLAY_MOTOR,
  DISPLAY_SENSOR,
  DISPLAY_SDCARD,
  DISPLAY_COUNT
};
DisplayMode currentDisplayMode = DISPLAY_MAIN;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

// Stepper Motors & Button
AccelStepper stepperAzm(AccelStepper::DRIVER, AZM_STEP_PIN, AZM_DIR_PIN);
AccelStepper stepperElv(AccelStepper::DRIVER, ELV_STEP_PIN, ELV_DIR_PIN);
volatile bool buttonPressed = false;
uint32_t lastButtonTime = 0;
const uint32_t BUTTON_DEBOUNCE = 200;

// Variabel SD Card
bool sdCardAvailable = false;
File logFile;
String logFileName = "";
uint32_t logFileNumber = 1;
uint32_t lastSdWrite = 0;
const uint32_t SD_WRITE_INTERVAL = 1000;
String dataBuffer = "";
const uint16_t BUFFER_SIZE = 1024;
String lastReceivedCommand = "";

// Variabel Sinyal
int rssi = 0;
float signalQuality = 0.0;
bool signalLock = false;

// Variabel telemetri lengkap dari Python
float droneLatitude = 0.0;
float droneLongitude = 0.0;
float droneAltitude = 0.0;
float droneSpeed = 0.0;

// Variabel sistem
uint32_t systemUptime = 0;
const uint8_t AS5600_ADDR = 0x36;
enum SystemState { STATE_ERROR, STATE_READY, STATE_TRACKING };
SystemState currentState = STATE_ERROR;
bool azmMagnetError = false, elvMagnetError = false;
bool azmI2CError = false, elvI2CError = false;

// Variabel Azimuth
float azmStartAngle = 0, azmTotalAngle = 0, azmPrevCorrectedAngle = 0;
float azmTargetPosition = 0, azmError = 0;
float azmCableRotation = 0;
bool cableOverLimit = false;

// Variabel Elevasi
float elvStartAngle = 0, elvTotalAngle = 0, elvPrevCorrectedAngle = 0;
float elvTargetPosition = 0, elvError = 0;

// Variabel PID
float azmControlSignal = 0, elvControlSignal = 0;
float azmKp = 30, azmKi = 0.01, azmKd = 0.001;
float elvKp = 15, elvKi = 0.001, elvKd = 0.001;
float azmErrorIntegral = 0, azmLastError = 0;
float elvErrorIntegral = 0, elvLastError = 0;
uint32_t lastPidTime = 0;
const uint16_t PID_INTERVAL = 1;

// Variabel lain
const uint32_t LOG_INTERVAL = 100;
uint32_t lastLogTime = 0;
const float MAX_SPEED = 5000;
String inputBuffer = "";
boolean commandComplete = false;

// --- Deklarasi Fungsi ---
void buttonISR();
void setup();
void loop();
void handleButtonPress();
void readSerialCommand();
void parseCommand(String command);
String getValue(String data, String key);
void setSystemState(SystemState newState);
void updateSystemState();
bool initializeAzmSensor();
bool initializeElvSensor();
bool testAzmSensor();
bool testElvSensor();
void updateOLEDDisplay();
void displayMainScreen();
void displaySignalScreen();
void displayGPSScreen();
void displaySystemScreen();
void displayMotorScreen();
void displaySensorScreen();
void displaySDCardScreen();
void updateAzmPID();
void updateElvPID();
bool checkMagnetPresenceAzm();
void calibrateStartAngleAzm();
float getAzmTotalAngle();
float readRawAngleDegAzm();
bool checkMagnetPresenceElv();
void calibrateStartAngleElv();
float getElvTotalAngle();
float readRawAngleDegElv();
float calculateShortestError(float target, float current);
bool checkCableLimit(float proposedError);
float calculateAlternativePath(float target, float current);
bool initializeSDCard();
void createLogFile();
void logDataToSD();
void appendToBuffer(String data);
void flushSDCardBuffer();
String formatTimestamp();
void closeLogFile();


// =======================================================================
//                         FUNGSI UTAMA (SETUP & LOOP)
// =======================================================================

void buttonISR() {
  buttonPressed = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi Pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  pinMode(AZM_EN_PIN, OUTPUT); digitalWrite(AZM_EN_PIN, LOW);
  pinMode(ELV_EN_PIN, OUTPUT); digitalWrite(ELV_EN_PIN, LOW);
  pinMode(LED_RED_PIN, OUTPUT); digitalWrite(LED_RED_PIN, LOW);
  pinMode(LED_YELLOW_PIN, OUTPUT); digitalWrite(LED_YELLOW_PIN, LOW);
  pinMode(LED_GREEN_PIN, OUTPUT); digitalWrite(LED_GREEN_PIN, LOW);

  // Inisialisasi Stepper
  stepperAzm.setMaxSpeed(MAX_SPEED);
  stepperAzm.setAcceleration(800);
  stepperAzm.enableOutputs();
  stepperElv.setMaxSpeed(MAX_SPEED);
  stepperElv.setAcceleration(800);
  stepperElv.enableOutputs();
  
  // Inisialisasi I2C
  Wire1.begin(); Wire1.setClock(800000L); // I2C untuk Azimuth Sensor
  Wire.begin(); Wire.setClock(800000L);   // I2C untuk Elevation Sensor
  Wire2.begin();                          // I2C untuk OLED
  
  // Inisialisasi OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    setSystemState(STATE_ERROR);
    for(;;); // Hentikan jika gagal
  }
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Antenna Tracker v1.4"));
  display.println(F("Initializing..."));
  display.display();
  delay(1000);

  // Inisialisasi SD Card
  sdCardAvailable = initializeSDCard();
  display.clearDisplay();
  display.setCursor(0, 10);
  if (sdCardAvailable) {
    display.println(F("SD Card: OK"));
    Serial.println("SD Card initialized successfully.");
    createLogFile();
    if (logFile) {
      // Header baru yang mencakup semua data telemetri
      String header = "Timestamp,TargetAzm,TargetElv,ActualAzm,ActualElv,ErrorAzm,ErrorElv,DroneLat,DroneLon,DroneAlt,DroneSpd,RSSI,Signal(%)";
      logFile.println(header);
      logFile.flush();
    }
  } else {
    display.println(F("SD Card: FAILED"));
    Serial.println("SD Card initialization failed!");
  }
  display.display();
  delay(1500);

  // Inisialisasi Sensor & State Awal
  bool azmSensorOk = initializeAzmSensor();
  bool elvSensorOk = initializeElvSensor();
  
  if (azmSensorOk && elvSensorOk) {
    setSystemState(STATE_READY);
    Serial.println("System Ready. Waiting for data from Python script...");
  } else {
    setSystemState(STATE_ERROR);
    if (!azmSensorOk) Serial.println("Azimuth sensor error!");
    if (!elvSensorOk) Serial.println("Elevation sensor error!");
  }
}

void loop() {
  static uint32_t lastDisplay = 0;
  static uint32_t lastRecoveryAttempt = 0;

  handleButtonPress();
  readSerialCommand();
  
  systemUptime = millis() / 1000;
  updateSystemState();
  
  // Coba pulihkan sensor jika dalam keadaan ERROR
  if (currentState == STATE_ERROR && millis() - lastRecoveryAttempt >= 2000) {
    if (initializeAzmSensor() && initializeElvSensor()) {
      setSystemState(STATE_READY);
    }
    lastRecoveryAttempt = millis();
  }

  // Jalankan kontrol PID dan motor jika tidak ERROR
  if (currentState != STATE_ERROR) {
    if (millis() - lastPidTime >= PID_INTERVAL) {
      updateAzmPID();
      updateElvPID();
      lastPidTime = millis();
    }
    stepperAzm.runSpeed();
    stepperElv.runSpeed();
  }

  // Kumpulkan data untuk di-log ke SD Card secara berkala saat tracking
  if (sdCardAvailable && currentState == STATE_TRACKING && millis() - lastLogTime >= LOG_INTERVAL) {
    logDataToSD();
    lastLogTime = millis();
  }

  // Tulis buffer ke SD Card secara berkala untuk efisiensi
  if (sdCardAvailable && millis() - lastSdWrite >= SD_WRITE_INTERVAL) {
    flushSDCardBuffer();
    lastSdWrite = millis();
  }

  // Perbarui display secara berkala
  if (millis() - lastDisplay >= 100) {
    updateOLEDDisplay();
    lastDisplay = millis();
  }
}

// =======================================================================
//                    FUNGSI PARSING & KONTROL STATE
// =======================================================================

void handleButtonPress() {
  if (buttonPressed && millis() - lastButtonTime >= BUTTON_DEBOUNCE) {
    currentDisplayMode = (DisplayMode)((currentDisplayMode + 1) % DISPLAY_COUNT);
    lastButtonTime = millis();
    buttonPressed = false;
  }
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '\n') {
      commandComplete = true;
      break;
    } else {
      inputBuffer += inChar;
    }
  }
  
  if (commandComplete) {
    parseCommand(inputBuffer);
    inputBuffer = "";
    commandComplete = false;
  }
}

String getValue(String data, String key) {
  int keyIndex = data.indexOf(key);
  if (keyIndex == -1) {
    return "";
  }
  int startIndex = keyIndex + key.length();
  int endIndex = data.indexOf(',', startIndex);
  if (endIndex == -1) {
    endIndex = data.length();
  }
  return data.substring(startIndex, endIndex);
}

void parseCommand(String command) {
  command.trim();
  lastReceivedCommand = command;

  azmTargetPosition = getValue(command, "AZM:").toFloat();
  elvTargetPosition = getValue(command, "ELV:").toFloat();
  droneLatitude     = getValue(command, "LAT:").toFloat();
  droneLongitude    = getValue(command, "LON:").toFloat();
  droneAltitude     = getValue(command, "ALT:").toFloat();
  droneSpeed        = getValue(command, "SPD:").toFloat();
  rssi              = getValue(command, "RSSI:").toInt();
  signalQuality     = getValue(command, "SIG:").toFloat();
  
  signalLock = (signalQuality > 30.0);

  if (command.indexOf("AZM:") != -1 && currentState != STATE_ERROR) {
    setSystemState(STATE_TRACKING);
  }
}

void setSystemState(SystemState newState) {
  if (currentState == newState) return;
  currentState = newState;
  
  digitalWrite(LED_RED_PIN, (currentState == STATE_ERROR));
  digitalWrite(LED_YELLOW_PIN, (currentState == STATE_READY));
  digitalWrite(LED_GREEN_PIN, (currentState == STATE_TRACKING));
}

void updateSystemState() {
  static uint32_t lastSensorCheck = 0;
  if (millis() - lastSensorCheck >= 500) {
    if (!testAzmSensor() || !testElvSensor()) {
      setSystemState(STATE_ERROR);
    } else if (currentState == STATE_ERROR) {
      setSystemState(STATE_READY);
    }
    lastSensorCheck = millis();
  }
}

// =======================================================================
//                       FUNGSI TAMPILAN OLED
// =======================================================================

void updateOLEDDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  
  switch (currentDisplayMode) {
    case DISPLAY_MAIN:    displayMainScreen();    break;
    case DISPLAY_SIGNAL:  displaySignalScreen();  break;
    case DISPLAY_GPS:     displayGPSScreen();     break;
    case DISPLAY_SYSTEM:  displaySystemScreen();  break;
    case DISPLAY_MOTOR:   displayMotorScreen();   break;
    case DISPLAY_SENSOR:  displaySensorScreen();  break;
    case DISPLAY_SDCARD:  displaySDCardScreen();  break;
  }
  
  display.display();
}

void displayMainScreen() {
  display.setCursor(0, 0);
  display.print(F("Main [1/")); display.print(DISPLAY_COUNT); display.println(F("]"));
  
  if (currentState == STATE_ERROR) {
    display.setCursor(0, 10);
    if (azmMagnetError) display.println(F("AZM: Magnet Error!"));
    else if (azmI2CError) display.println(F("AZM: I2C Error!"));
    else display.println(F("AZM: OK"));
    display.setCursor(0, 20);
    if (elvMagnetError) display.println(F("ELV: Magnet Error!"));
    else if (elvI2CError) display.println(F("ELV: I2C Error!"));
    else display.println(F("ELV: OK"));
  } else if (currentState == STATE_READY) {
    display.setCursor(0, 12); display.print(F("AZ:")); display.print(azmTotalAngle, 1);
    display.setCursor(0, 22); display.print(F("EL:")); display.print(elvTotalAngle, 1);
    display.setCursor(64, 16); display.print(F("READY"));
  } else if (currentState == STATE_TRACKING) {
    display.setCursor(0, 10);
    display.print(F("AZ:")); display.print(azmTotalAngle, 1); display.print(F("/")); display.print(azmTargetPosition, 1);
    display.setCursor(0, 20);
    display.print(F("EL:")); display.print(elvTotalAngle, 1); display.print(F("/")); display.print(elvTargetPosition, 1);
  }
}

void displaySignalScreen() {
  display.setCursor(0, 0);
  display.print(F("Signal [2/")); display.print(DISPLAY_COUNT); display.println(F("]"));
  
  display.setCursor(0, 10);
  display.print(F("DRONE "));
  display.print(signalLock ? F("LOCK") : F("SCAN"));
  
  display.setCursor(0, 20);
  display.print(F("RSSI:")); display.print(rssi);
  
  int barWidth = 30, barHeight = 4, barX = 95, barY = 15;
  display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
  int fillWidth = (signalQuality / 100.0) * (barWidth - 2);
  if (fillWidth > 0) {
    display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);
  }
  display.setCursor(barX + 5, barY + 6);
  display.print(signalQuality, 0); display.print(F("%"));
}

void displayGPSScreen() {
  display.setCursor(0, 0);
  display.print(F("GPS & Telemetry [3/")); display.print(DISPLAY_COUNT); display.println(F("]"));

  display.setCursor(0, 10);
  display.print(F("Lat:")); display.print(droneLatitude, 4);
  display.setCursor(68, 10);
  display.print(F("Alt:")); display.print(droneAltitude, 1);

  display.setCursor(0, 20);
  display.print(F("Lon:")); display.print(droneLongitude, 4);
  display.setCursor(68, 20);
  display.print(F("Spd:")); display.print(droneSpeed, 1);
}

void displaySystemScreen() {
  display.setCursor(0, 0);
  display.print(F("System [4/")); display.print(DISPLAY_COUNT); display.println(F("]"));
  display.setCursor(0, 10);
  display.print(F("Uptime: "));
  uint32_t h = systemUptime / 3600;
  uint32_t m = (systemUptime % 3600) / 60;
  uint32_t s = systemUptime % 60;
  if (h > 0) { display.print(h); display.print(F("h")); }
  display.print(m); display.print(F("m")); display.print(s); display.print(F("s"));
  
  display.setCursor(0, 20);
  display.print(F("Cable: ")); display.print(azmCableRotation, 0);
}

void displayMotorScreen() {
  display.setCursor(0, 0);
  display.print(F("Motor [5/")); display.print(DISPLAY_COUNT); display.println(F("]"));
  display.setCursor(0, 10);
  display.print(F("AZ Spd:")); display.print(int(stepperAzm.speed()));
  display.setCursor(0, 20);
  display.print(F("EL Spd:")); display.print(int(stepperElv.speed()));
}

void displaySensorScreen() {
  display.setCursor(0, 0);
  display.print(F("Sensor [6/")); display.print(DISPLAY_COUNT); display.println(F("]"));
  display.setCursor(0, 10);
  display.print(F("AZ Raw:")); display.print(readRawAngleDegAzm(), 1);
  display.setCursor(0, 20);
  display.print(F("EL Raw:")); display.print(readRawAngleDegElv(), 1);
}

void displaySDCardScreen() {
  display.setCursor(0, 0);
  display.print(F("SD Card [7/")); display.print(DISPLAY_COUNT); display.println(F("]"));
  display.setCursor(0, 10);
  display.print(F("Status: "));  
  display.print(sdCardAvailable ? F("LOGGING") : F("FAIL"));
  
  if (sdCardAvailable) {
    display.setCursor(0, 20);
    display.print(F("File: ")); display.print(logFileName);
  } else {
    display.setCursor(0, 20);
    display.print(F("Check SD Card"));
  }
}

// =======================================================================
//                       FUNGSI-FUNGSI SD CARD
// =======================================================================

bool initializeSDCard() {
  if (!SD.begin(BUILTIN_SDCARD)) { // Gunakan BUILTIN_SDCARD untuk Teensy
    return false;
  }
  return true;
}

void createLogFile() {
  do {
    logFileName = "LOG_" + String(logFileNumber, DEC) + ".csv";
    logFileNumber++;
  } while (SD.exists(logFileName.c_str()) && logFileNumber < 10000);
  
  logFile = SD.open(logFileName.c_str(), FILE_WRITE);
  if (logFile) {
    Serial.println("Log file created: " + logFileName);
  } else {
    Serial.println("Error creating log file: " + logFileName);
    sdCardAvailable = false;
  }
}

void logDataToSD() {
  if (!sdCardAvailable) return;

  String dataLine = "";
  dataLine += formatTimestamp(); dataLine += ",";
  dataLine += String(azmTargetPosition, 3); dataLine += ",";
  dataLine += String(elvTargetPosition, 3); dataLine += ",";
  dataLine += String(getAzmTotalAngle(), 3); dataLine += ",";
  dataLine += String(getElvTotalAngle(), 3); dataLine += ",";
  dataLine += String(azmError, 3); dataLine += ",";
  dataLine += String(elvError, 3); dataLine += ",";
  dataLine += String(droneLatitude, 6); dataLine += ",";
  dataLine += String(droneLongitude, 6); dataLine += ",";
  dataLine += String(droneAltitude, 2); dataLine += ",";
  dataLine += String(droneSpeed, 2); dataLine += ",";
  dataLine += String(rssi); dataLine += ",";
  dataLine += String(signalQuality, 1);
  
  appendToBuffer(dataLine);
}

void appendToBuffer(String data) {
  if (!sdCardAvailable) return;
  
  dataBuffer += data + "\n";
  
  if (dataBuffer.length() > BUFFER_SIZE) {
    flushSDCardBuffer();
  }
}

void flushSDCardBuffer() {
  if (!sdCardAvailable || dataBuffer.length() == 0) return;
  
  if (logFile) {
    logFile.print(dataBuffer);
    logFile.flush();
    dataBuffer = "";
  } else {
    logFile = SD.open(logFileName.c_str(), FILE_WRITE);
    if (!logFile) {
      Serial.println("Error reopening log file");
      sdCardAvailable = false;
    }
  }
}

String formatTimestamp() {
  unsigned long now = millis();
  unsigned long s = now / 1000;
  unsigned long m = s / 60;
  unsigned long h = m / 60;
  unsigned int ms = now % 1000;

  s %= 60;
  m %= 60;
  h %= 24;

  char timestamp[15];
  sprintf(timestamp, "%02lu:%02lu:%02lu.%03u", h, m, s, ms);
  return String(timestamp);
}

void closeLogFile() {
  if (logFile) {
    flushSDCardBuffer();
    logFile.close();
    Serial.println("Log file closed.");
  }
}

// =======================================================================
//                       FUNGSI KONTROL PID & MOTOR
// =======================================================================

void updateAzmPID() {
  float currentAngle = getAzmTotalAngle();
  static float prevAngle = currentAngle;
  azmError = calculateAlternativePath(azmTargetPosition, currentAngle);
  
  float actualMovement = currentAngle - prevAngle;
  if (actualMovement > 180) actualMovement -= 360;
  else if (actualMovement < -180) actualMovement += 360;
  azmCableRotation += actualMovement;
  prevAngle = currentAngle;
  
  float dt = PID_INTERVAL / 1000.0f;
  
  if (!cableOverLimit) {
    float P = azmKp * azmError;
    azmErrorIntegral += azmError * dt;
    azmErrorIntegral = constrain(azmErrorIntegral, -1000.0, 1000.0);
    float I = azmKi * azmErrorIntegral;
    float D = azmKd * (azmError - azmLastError) / dt;
    azmControlSignal = (P + I + D) * AZM_GEAR_RATIO;
    azmControlSignal = constrain(azmControlSignal, -MAX_SPEED, MAX_SPEED);
  } else {
    azmControlSignal = 0;
    azmErrorIntegral = 0;
  }
  
  stepperAzm.setSpeed(-azmControlSignal);
  azmLastError = azmError;
}

void updateElvPID() {
  float currentAngle = getElvTotalAngle();
  elvError = elvTargetPosition - currentAngle;
  
  float dt = PID_INTERVAL / 1000.0f;
  
  float P = elvKp * elvError;
  elvErrorIntegral += elvError * dt;
  elvErrorIntegral = constrain(elvErrorIntegral, -1000.0, 1000.0);
  float I = elvKi * elvErrorIntegral;
  float D = elvKd * (elvError - elvLastError) / dt;
  
  elvControlSignal = (P + I + D) * ELV_GEAR_RATIO;
  elvControlSignal = constrain(elvControlSignal, -MAX_SPEED, MAX_SPEED);
  
  stepperElv.setSpeed(-elvControlSignal);
  elvLastError = elvError;
}

// =======================================================================
//                       FUNGSI PEMBACAAN SENSOR AS5600
// =======================================================================

bool initializeAzmSensor() {
  Wire1.beginTransmission(AS5600_ADDR);
  if (Wire1.endTransmission() != 0) { azmI2CError = true; return false; }
  if (!checkMagnetPresenceAzm()) { azmMagnetError = true; return false; }
  azmI2CError = false; azmMagnetError = false;
  calibrateStartAngleAzm();
  return true;
}

bool initializeElvSensor() {
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) { elvI2CError = true; return false; }
  if (!checkMagnetPresenceElv()) { elvMagnetError = true; return false; }
  elvI2CError = false; elvMagnetError = false;
  calibrateStartAngleElv();
  return true;
}

bool testAzmSensor() {
  Wire1.beginTransmission(AS5600_ADDR); Wire1.write(0x0B);
  if (Wire1.endTransmission(false) != 0) return false;
  Wire1.requestFrom(AS5600_ADDR, (uint8_t)1);
  if (Wire1.available() != 1) return false;
  return ((Wire1.read() & 0x20) != 0);
}

bool testElvSensor() {
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x0B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
  if (Wire.available() != 1) return false;
  return ((Wire.read() & 0x20) != 0);
}

bool checkMagnetPresenceAzm() { return testAzmSensor(); }
bool checkMagnetPresenceElv() { return testElvSensor(); }

void calibrateStartAngleAzm() {
  azmStartAngle = readRawAngleDegAzm();
  azmPrevCorrectedAngle = 0;
  azmTotalAngle = 0;
}

void calibrateStartAngleElv() {
  elvStartAngle = readRawAngleDegElv();
  elvPrevCorrectedAngle = 0;
  elvTotalAngle = 0;
}

float getAzmTotalAngle() {
  float currentAngle = readRawAngleDegAzm();
  float corrected = currentAngle - azmStartAngle;
  if (corrected < 0) corrected += 360;
  float delta = corrected - azmPrevCorrectedAngle;
  if (delta < -180) delta += 360;
  else if (delta > 180) delta -= 360;
  azmTotalAngle -= delta / AZM_GEAR_RATIO;
  azmPrevCorrectedAngle = corrected;
  return azmTotalAngle;
}

float getElvTotalAngle() {
  float currentAngle = readRawAngleDegElv();
  float corrected = currentAngle - elvStartAngle;
  if (corrected < 0) corrected += 360;
  float delta = corrected - elvPrevCorrectedAngle;
  if (delta < -180) delta += 360;
  else if (delta > 180) delta -= 360;
  elvTotalAngle -= delta / ELV_GEAR_RATIO;
  elvPrevCorrectedAngle = corrected;
  return elvTotalAngle;
}

float readRawAngleDegAzm() {
  Wire1.beginTransmission(AS5600_ADDR); Wire1.write(0x0C);
  if (Wire1.endTransmission(false) != 0) { return 0; }
  Wire1.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire1.available() != 2) { return 0; }
  uint16_t angle = (Wire1.read() & 0x0F) << 8;
  angle |= Wire1.read();
  return angle * 0.087890625;
}

float readRawAngleDegElv() {
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) { return 0; }
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() != 2) { return 0; }
  uint16_t angle = (Wire.read() & 0x0F) << 8;
  angle |= Wire.read();
  return angle * 0.087890625;
}

// =======================================================================
//                       FUNGSI MANAJEMEN JALUR & KABEL
// =======================================================================

float calculateShortestError(float target, float current) {
  float error = fmod(target - current, 360.0);
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  return error;
}

bool checkCableLimit(float proposedError) {
  return (abs(azmCableRotation + proposedError) <= MAX_CABLE_ROTATION);
}

float calculateAlternativePath(float target, float current) {
  float shortestError = calculateShortestError(target, current);
  
  if (!checkCableLimit(shortestError)) {
    float alternativeError = shortestError + (shortestError > 0 ? -360 : 360);
    if (checkCableLimit(alternativeError)) {
      cableOverLimit = false;
      return alternativeError;
    } else {
      cableOverLimit = true;
      return 0; // Kedua jalur diblokir, berhenti
    }
  }
  
  cableOverLimit = false;
  return shortestError;
}
