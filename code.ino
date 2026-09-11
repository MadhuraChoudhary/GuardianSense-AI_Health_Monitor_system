/*
   ESP32 + TCA9548A + MAX30100 + MPU6050 + BMP280 + GPS + GSM + OLED
   COMPLETE HEALTH MONITORING SYSTEM - TEMPERATURE MODE
   
   TEMPERATURE MODE DISPLAY STATES:
   1. Waiting: "PRESS ACTION"
   2. Analyzing: Progress bar with percentage
   3. Results: Temperature value with status
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

// ============================================================
// ========== PIN DEFINITIONS ==========
// ============================================================

#define BUTTON_MODE 15
#define BUTTON_RESET 14
#define BUTTON_ACTION 13
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GSM_RX_PIN 26
#define GSM_TX_PIN 27

// ============================================================
// ========== I2C DEFINITIONS ==========
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define TCA_ADDR      0x70
#define MAX30100_ADDR 0x57
#define OLED_ADDR     0x3C
#define BMP280_ADDR   0x76

#define MAX_CHANNEL  0
#define OLED_CHANNEL 1
#define BMP280_CHANNEL 2

// ============================================================
// ========== OLED SETTINGS ==========
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================
// ========== MAX30100 REGISTERS ==========
// ============================================================

#define REG_INT_STATUS   0x00
#define REG_INT_ENABLE   0x01
#define REG_FIFO_WR_PTR  0x02
#define REG_OVF_COUNTER  0x03
#define REG_FIFO_RD_PTR  0x04
#define REG_FIFO_DATA    0x05
#define REG_MODE_CONFIG  0x06
#define REG_SPO2_CONFIG  0x07
#define REG_LED_CONFIG   0x09
#define REG_REV_ID       0xFE
#define REG_PART_ID      0xFF

// ============================================================
// ========== MAX30100 MEASUREMENT ==========
// ============================================================

#define SAMPLE_RATE_HZ   100
#define ANALYSIS_SECONDS 10
#define TOTAL_SAMPLES    (SAMPLE_RATE_HZ * ANALYSIS_SECONDS)
#define FINGER_THRESHOLD 5000
#define FINGER_REMOVE_THRESHOLD 3500

// ============================================================
// ========== FALL DETECTION CONSTANTS ==========
// ============================================================

#define FALL_X_THRESHOLD 15.0
#define FALL_Y_THRESHOLD 15.0
#define FALL_Z_THRESHOLD 3.0
#define FALL_DEBOUNCE_TIME 500
#define NORMAL_DEBOUNCE_TIME 1000

// ============================================================
// ========== TEMPERATURE CONSTANTS ==========
// ============================================================

#define TEMP_ANALYSIS_DURATION 10000
#define TEMP_BUFFER_SIZE 10

// ============================================================
// ========== SYSTEM MODES ==========
// ============================================================

enum SystemMode {
  MODE_FALL_DETECTION,
  MODE_PULSE_OXIMETER,
  MODE_TEMPERATURE
};

// ============================================================
// ========== FALL STATES ==========
// ============================================================

enum FallStatus {
  FALL_GOOD,
  FALL_DETECTED,
  FALL_CONFIRMED
};

// ============================================================
// ========== GLOBAL VARIABLES ==========
// ============================================================

SystemMode currentMode = MODE_FALL_DETECTION;
unsigned long lastDisplayUpdate = 0;
String modeNames[] = {"FALL", "PULSE", "TEMP"};

// ============================================================
// ========== MODE 1: FALL DETECTION VARIABLES ==========
// ============================================================

Adafruit_MPU6050 mpu;
bool mpuReady = false;
float accelX = 0, accelY = 0, accelZ = 0;
float totalAccel = 0;
float currentPitch = 0, currentRoll = 0;
float normalPitch = 0, normalRoll = 0;
bool referenceSet = false;
FallStatus fallStatus = FALL_GOOD;
bool alertSentForThisFall = false;
bool smsSent = false;
unsigned long fallDetectTime = 0;
unsigned long normalStartTime = 0;
bool fallActive = false;

// ============================================================
// ========== MODE 2: PULSE OXIMETER VARIABLES ==========
// ============================================================

uint16_t irBuffer[TOTAL_SAMPLES];
uint16_t redBuffer[TOTAL_SAMPLES];
int sampleCount = 0;
uint16_t currentIR = 0;
uint16_t currentRED = 0;
float heartRate = 0.0;
float spo2 = 0.0;
bool fingerDetected = false;
bool measuring = false;
bool pulseResultShown = false;
unsigned long measurementStart = 0;
unsigned long lastPulseOLEDUpdate = 0;
bool max30100Ready = false;

// ============================================================
// ========== MODE 3: TEMPERATURE VARIABLES ==========
// ============================================================

bool bmp280Ready = false;
float currentTemp = 0;
float finalTemp = 0;
float tempBuffer[TEMP_BUFFER_SIZE];
int tempIndex = 0;
int tempCount = 0;
bool tempMeasuring = false;
bool tempAnalysisComplete = false;
bool tempDataRecorded = false;
unsigned long tempStartTime = 0;
unsigned long tempProgress = 0;

// BMP280 Calibration Data
struct BMP280Calib {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} bmpCalib;
bool bmpCalibRead = false;
int32_t bmp_t_fine = 0;

// ============================================================
// ========== GPS VARIABLES ==========
// ============================================================

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
double latitude = 0.0;
double longitude = 0.0;
int satellites = 0;
bool gpsFix = false;
unsigned long lastGpsUpdate = 0;
#define GPS_FIX_TIMEOUT 30000

// ============================================================
// ========== GSM VARIABLES ==========
// ============================================================

HardwareSerial gsmSerial(1);
bool gsmReady = false;
#define SOS_PHONE_NUMBER "+919511776950"

// ============================================================
// ========== BUTTON VARIABLES ==========
// ============================================================

bool modeButtonPressed = false, resetButtonPressed = false, actionButtonPressed = false;
bool lastModeButtonState = HIGH, lastResetButtonState = HIGH, lastActionButtonState = HIGH;
unsigned long lastModeDebounce = 0, lastResetDebounce = 0, lastActionDebounce = 0;
#define DEBOUNCE_DELAY 50

// ============================================================
// ========== TCA9548A FUNCTIONS ==========
// ============================================================

bool selectTCA(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(500);
  return true;
}

bool tcaDetected() {
  Wire.beginTransmission(TCA_ADDR);
  return Wire.endTransmission() == 0;
}

// ============================================================
// ========== MAX30100 FUNCTIONS ==========
// ============================================================

bool writeMAX(uint8_t reg, uint8_t value) {
  if (!selectTCA(MAX_CHANNEL)) return false;
  Wire.beginTransmission(MAX30100_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

uint8_t readMAX(uint8_t reg) {
  if (!selectTCA(MAX_CHANNEL)) return 0xFF;
  Wire.beginTransmission(MAX30100_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MAX30100_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

void resetMAX() {
  writeMAX(REG_MODE_CONFIG, 0x40);
  delay(100);
}

void clearFIFO() {
  writeMAX(REG_FIFO_WR_PTR, 0x00);
  writeMAX(REG_OVF_COUNTER, 0x00);
  writeMAX(REG_FIFO_RD_PTR, 0x00);
}

bool initializeMAX30100() {
  if (!selectTCA(MAX_CHANNEL)) return false;
  Wire.beginTransmission(MAX30100_ADDR);
  if (Wire.endTransmission() != 0) return false;
  
  uint8_t partID = readMAX(REG_PART_ID);
  Serial.print("MAX30100 PART ID: 0x");
  Serial.println(partID, HEX);
  
  resetMAX();
  delay(100);
  clearFIFO();
  
  if (!writeMAX(REG_SPO2_CONFIG, 0x47)) return false;
  if (!writeMAX(REG_LED_CONFIG, 0x77)) return false;
  if (!writeMAX(REG_MODE_CONFIG, 0x03)) return false;
  writeMAX(REG_INT_ENABLE, 0x00);
  delay(100);
  clearFIFO();
  
  Serial.println("MAX30100 initialized.");
  return true;
}

bool readFIFO(uint16_t &ir, uint16_t &red) {
  if (!selectTCA(MAX_CHANNEL)) return false;
  
  Wire.beginTransmission(MAX30100_ADDR);
  Wire.write(REG_FIFO_WR_PTR);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAX30100_ADDR, (uint8_t)1) != 1) return false;
  uint8_t writePointer = Wire.read();
  
  Wire.beginTransmission(MAX30100_ADDR);
  Wire.write(REG_FIFO_RD_PTR);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAX30100_ADDR, (uint8_t)1) != 1) return false;
  uint8_t readPointer = Wire.read();
  
  if (writePointer == readPointer) return false;
  
  Wire.beginTransmission(MAX30100_ADDR);
  Wire.write(REG_FIFO_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAX30100_ADDR, (uint8_t)4) != 4) return false;
  
  uint8_t irMSB = Wire.read();
  uint8_t irLSB = Wire.read();
  uint8_t redMSB = Wire.read();
  uint8_t redLSB = Wire.read();
  
  ir = ((uint16_t)irMSB << 8) | irLSB;
  red = ((uint16_t)redMSB << 8) | redLSB;
  
  return true;
}

void resetPulseMeasurement() {
  sampleCount = 0;
  heartRate = 0;
  spo2 = 0;
  pulseResultShown = false;
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }
  clearFIFO();
}

void calculateHeartRate() {
  if (sampleCount < TOTAL_SAMPLES) {
    heartRate = 0;
    return;
  }

  float mean = 0;
  for (int i = 0; i < TOTAL_SAMPLES; i++) mean += irBuffer[i];
  mean /= TOTAL_SAMPLES;

  float variance = 0;
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    float x = irBuffer[i] - mean;
    variance += x * x;
  }
  float sd = sqrt(variance / TOTAL_SAMPLES);

  if (sd < 20) { heartRate = 0; return; }

  float threshold = mean + (sd * 0.30);
  const int MIN_DISTANCE = 30;
  int peaks[40];
  int peakCount = 0;
  int lastPeak = -MIN_DISTANCE;

  for (int i = 2; i < TOTAL_SAMPLES - 2; i++) {
    bool peak = irBuffer[i] > irBuffer[i-1] &&
                irBuffer[i] >= irBuffer[i+1] &&
                irBuffer[i] > irBuffer[i-2] &&
                irBuffer[i] >= irBuffer[i+2] &&
                irBuffer[i] > threshold;

    if (peak && (i - lastPeak >= MIN_DISTANCE)) {
      if (peakCount < 40) {
        peaks[peakCount] = i;
        peakCount++;
        lastPeak = i;
      }
    }
  }

  if (peakCount < 3) { heartRate = 0; return; }

  float intervalSum = 0;
  int intervalCount = 0;
  for (int i = 1; i < peakCount; i++) {
    int diff = peaks[i] - peaks[i-1];
    if (diff >= 33 && diff <= 150) {
      intervalSum += diff;
      intervalCount++;
    }
  }

  if (intervalCount == 0) { heartRate = 0; return; }

  float averageInterval = intervalSum / intervalCount;
  float bpm = (60.0 * SAMPLE_RATE_HZ) / averageInterval;

  if (bpm >= 40 && bpm <= 180) heartRate = bpm;
  else heartRate = 0;
}

void calculateSpO2() {
  if (sampleCount < TOTAL_SAMPLES) { spo2 = 0; return; }

  float irDC = 0, redDC = 0;
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    irDC += irBuffer[i];
    redDC += redBuffer[i];
  }
  irDC /= TOTAL_SAMPLES;
  redDC /= TOTAL_SAMPLES;

  if (irDC <= 0 || redDC <= 0) { spo2 = 0; return; }

  float irACSum = 0, redACSum = 0;
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    float irAC = irBuffer[i] - irDC;
    float redAC = redBuffer[i] - redDC;
    irACSum += irAC * irAC;
    redACSum += redAC * redAC;
  }

  float irAC = sqrt(irACSum / TOTAL_SAMPLES);
  float redAC = sqrt(redACSum / TOTAL_SAMPLES);

  if (irAC < 1 || redAC < 1) { spo2 = 0; return; }

  float R = (redAC / redDC) / (irAC / irDC);
  float calculatedSpO2 = 110.0 - (25.0 * R);

  if (calculatedSpO2 < 70 || calculatedSpO2 > 100) { spo2 = 0; return; }

  spo2 = calculatedSpO2;
}

void printPulseResult() {
  Serial.println();
  Serial.println("==============================");
  Serial.println("10 SECOND MEASUREMENT");
  Serial.println("==============================");

  Serial.print("Heart Rate : ");
  if (heartRate > 0) {
    Serial.print(heartRate, 1);
    Serial.println(" BPM");
  } else {
    Serial.println("Invalid");
  }

  Serial.print("SpO2       : ");
  if (spo2 > 0) {
    Serial.print(spo2, 1);
    Serial.println(" %");
  } else {
    Serial.println("Invalid");
  }

  Serial.println("==============================");
  Serial.println();
}

// ============================================================
// ========== BMP280 FUNCTIONS ==========
// ============================================================

bool initBMP280() {
  if (!selectTCA(BMP280_CHANNEL)) return false;
  delay(50);
  
  Wire.beginTransmission(BMP280_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("BMP280 NOT FOUND at 0x76!");
    return false;
  }
  
  Serial.println("✅ BMP280 detected at 0x76");
  
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xD0);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 1);
  if (Wire.available()) {
    uint8_t chipID = Wire.read();
    Serial.print("BMP280 Chip ID: 0x");
    Serial.println(chipID, HEX);
  }
  
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0x88);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 24);
  
  if (Wire.available() >= 24) {
    bmpCalib.dig_T1 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_T2 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_T3 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P1 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P2 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P3 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P4 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P5 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P6 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P7 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P8 = Wire.read() | (Wire.read() << 8);
    bmpCalib.dig_P9 = Wire.read() | (Wire.read() << 8);
    bmpCalibRead = true;
    Serial.println("✅ BMP280 calibration data read.");
  } else {
    Serial.println("⚠️ Failed to read BMP280 calibration!");
    return false;
  }
  
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(100);
  
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xF4);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(50);
  
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xF5);
  Wire.write(0xA0);
  Wire.endTransmission();
  delay(50);
  
  float testTemp = readBMP280Temperature();
  if (testTemp > 0 && testTemp < 100) {
    Serial.print("BMP280 test reading: ");
    Serial.print(testTemp, 2);
    Serial.println(" °C");
    currentTemp = testTemp;
    return true;
  }
  
  return true;
}

float readBMP280Temperature() {
  if (!bmpCalibRead) return 0;
  if (!selectTCA(BMP280_CHANNEL)) return 0;
  
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xFA);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 3);
  
  if (Wire.available() >= 3) {
    int32_t adc_T = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
    
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmpCalib.dig_T1 << 1))) * 
                    ((int32_t)bmpCalib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmpCalib.dig_T1)) * 
                      ((adc_T >> 4) - ((int32_t)bmpCalib.dig_T1))) >> 12) * 
                    ((int32_t)bmpCalib.dig_T3)) >> 14;
    bmp_t_fine = var1 + var2;
    
    float temp = (bmp_t_fine * 5 + 128) >> 8;
    temp = temp / 100.0;
    return temp;
  }
  return 0;
}

void readTemperature() {
  if (!bmp280Ready) return;
  
  float rawTemp = readBMP280Temperature();
  if (rawTemp > 0 && rawTemp < 100) {
    if (currentTemp == 0) {
      currentTemp = rawTemp;
    } else {
      currentTemp = currentTemp * 0.7 + rawTemp * 0.3;
    }
  }
}

void startTempAnalysis() {
  if (tempMeasuring) return;
  
  tempMeasuring = true;
  tempStartTime = millis();
  finalTemp = 0;
  tempAnalysisComplete = false;
  tempDataRecorded = false;
  tempCount = 0;
  tempIndex = 0;
  
  for (int i = 0; i < TEMP_BUFFER_SIZE; i++) tempBuffer[i] = 0;
  
  Serial.println("🌡️ Temperature Analysis Started!");
  showTempMeasuring();
}

void updateTempAnalysis() {
  if (!tempMeasuring) return;
  
  readTemperature();
  
  if (currentTemp > 0) {
    tempBuffer[tempIndex] = currentTemp;
    tempIndex = (tempIndex + 1) % TEMP_BUFFER_SIZE;
    if (tempCount < TEMP_BUFFER_SIZE) tempCount++;
  }
  
  tempProgress = millis() - tempStartTime;
  
  if (currentMode == MODE_TEMPERATURE) {
    showTempMeasuring();
  }
  
  if (tempProgress >= TEMP_ANALYSIS_DURATION) {
    float sumTemp = 0;
    int valid = 0;
    for (int i = 0; i < tempCount; i++) {
      if (tempBuffer[i] > 0) {
        sumTemp += tempBuffer[i];
        valid++;
      }
    }
    
    if (valid > 5) {
      finalTemp = sumTemp / valid;
      tempAnalysisComplete = true;
      tempDataRecorded = true;
    } else {
      finalTemp = 0;
      tempAnalysisComplete = false;
      tempDataRecorded = false;
      displayMessage("❌ Invalid Data", "Please try again");
    }
    
    tempMeasuring = false;
    
    if (tempAnalysisComplete && finalTemp > 0) {
      Serial.print("🌡️ Temperature Complete! Temp: ");
      Serial.println(finalTemp, 1);
      Serial.println("========================================");
      showTempResults();
      displayMessage("✅ Complete!", "Temp: " + String(finalTemp, 1) + "°C");
      sendTempReport();
    }
  }
}

void resetTempData() {
  tempMeasuring = false;
  tempAnalysisComplete = false;
  tempDataRecorded = false;
  finalTemp = 0;
  tempCount = 0;
  tempIndex = 0;
  for (int i = 0; i < TEMP_BUFFER_SIZE; i++) tempBuffer[i] = 0;
  displayMessage("🔄 Reset", "Temp Data Cleared");
}

// ============================================================
// ========== OLED DISPLAY FUNCTIONS ==========
// ============================================================

bool initializeOLED() {
  if (!selectTCA(OLED_CHANNEL)) return false;
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
  return true;
}

void showSensorError(const char *line1, const char *line2) {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(5, 5);
  display.println("SYSTEM ERROR");
  display.setCursor(5, 25);
  display.println(line1);
  display.setCursor(5, 42);
  display.println(line2);
  display.display();
}

void displayMessage(String line1, String line2) {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(line1);
  display.setCursor(0, 35);
  display.println(line2);
  display.display();
  delay(1500);
}

// ============================================================
// ========== MODE 1: FALL DETECTION DISPLAY ==========
// ============================================================

void displayFallStatus() {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("FALL GUARD");
  
  display.setCursor(85, 0);
  display.print("[FALL]");
  
  display.setCursor(118, 0);
  if (gpsFix) {
    display.fillCircle(122, 4, 3, SSD1306_WHITE);
  } else {
    display.drawCircle(122, 4, 3, SSD1306_WHITE);
  }
  
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  if (fallStatus == FALL_CONFIRMED) {
    display.drawTriangle(10, 20, 4, 38, 16, 38, SSD1306_WHITE);
    display.fillRect(8, 24, 4, 8, SSD1306_WHITE);
    display.fillRect(8, 36, 4, 2, SSD1306_WHITE);
    
    display.setTextSize(3);
    display.setCursor(28, 16);
    display.println("FALL");
    
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.drawLine(0, 46, 128, 46, SSD1306_WHITE);
    display.setCursor(2, 49);
    display.print(alertSentForThisFall ? "SOS: SENT" : "SOS: SENDING...");
    
  } else if (fallStatus == FALL_DETECTED) {
    display.setTextSize(2);
    display.setCursor(10, 18);
    display.println("DETECTED");
    display.setTextSize(1);
    display.setCursor(10, 38);
    display.println("CONFIRMING...");
    
    int progress = ((millis() - fallDetectTime) * 100) / FALL_DEBOUNCE_TIME;
    if (progress > 100) progress = 100;
    display.drawRect(0, 48, 128, 8, SSD1306_WHITE);
    display.fillRect(2, 50, (progress * 124) / 100, 4, SSD1306_WHITE);
    
  } else {
    display.drawLine(10, 20, 25, 35, SSD1306_WHITE);
    display.drawLine(25, 35, 45, 15, SSD1306_WHITE);
    display.drawLine(11, 20, 24, 34, SSD1306_WHITE);
    display.drawLine(24, 34, 44, 16, SSD1306_WHITE);
    
    display.setTextSize(3);
    display.setCursor(55, 16);
    display.println("GOOD");
    
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.drawLine(0, 46, 128, 46, SSD1306_WHITE);
    display.setCursor(2, 49);
    display.print("STATUS: NORMAL");
  }
  
  display.drawLine(0, 55, 128, 55, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 58);
  
  if (fallStatus == FALL_CONFIRMED) {
    display.print("PRESS RESET");
  } else {
    display.print("MONITORING");
    if ((millis() / 500) % 2 == 0) {
      display.fillCircle(100, 60, 2, SSD1306_WHITE);
    }
  }
  
  display.display();
}

// ============================================================
// ========== MODE 2: PULSE OXIMETER DISPLAY ==========
// ============================================================

void showPlaceFinger() {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("PULSE OXIMETER");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(10, 22);
  display.println("PLACE");
  display.setCursor(10, 42);
  display.println("FINGER");
  
  display.drawLine(0, 55, 128, 55, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 58);
  display.print("MODE=PULSE");
  
  display.display();
}

void showPulseMeasuring() {
  selectTCA(OLED_CHANNEL);
  int elapsed = (millis() - measurementStart) / 1000;
  if (elapsed > ANALYSIS_SECONDS) elapsed = ANALYSIS_SECONDS;
  int progress = (elapsed * 100) / ANALYSIS_SECONDS;
  if (progress > 100) progress = 100;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("PULSE OXIMETER");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(10, 16);
  display.print("MEASURING");
  
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.print("Time: ");
  display.print(elapsed);
  display.print("s / ");
  display.print(ANALYSIS_SECONDS);
  display.print("s");
  
  display.drawRect(0, 44, 128, 10, SSD1306_WHITE);
  display.fillRect(2, 46, (progress * 124) / 100, 6, SSD1306_WHITE);
  
  display.setCursor(0, 58);
  display.print("Keep finger steady");
  
  display.display();
}

void showPulseResults() {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("PULSE OXIMETER");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(40, 14);
  display.println("RESULT");
  display.drawLine(0, 22, 128, 22, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(2, 26);
  display.print("HR");
  display.setTextSize(2);
  display.setCursor(2, 36);
  if (heartRate > 0) display.print(heartRate, 0);
  else display.print("--");
  display.setTextSize(1);
  display.print(" BPM");

  display.setTextSize(1);
  display.setCursor(70, 26);
  display.print("SpO2");
  display.setTextSize(2);
  display.setCursor(70, 36);
  if (spo2 > 0) display.print(spo2, 0);
  else display.print("--");
  display.setTextSize(1);
  display.print(" %");
  
  display.drawLine(0, 55, 128, 55, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 58);
  display.print("PRESS ACTION TO SEND");
  
  display.display();
}

// ============================================================
// ========== MODE 3: TEMPERATURE DISPLAY (EXACT LAYOUTS) ==========
// ============================================================

void showTempWaiting() {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Top bar
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("TEMPERATURE");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // "PRESS" text
  display.setTextSize(2);
  display.setCursor(10, 22);
  display.println("PRESS");
  
  // "ACTION" text
  display.setCursor(10, 42);
  display.println("ACTION");
  
  // Bottom bar
  display.drawLine(0, 55, 128, 55, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 58);
  display.print("MODE=TEMP");
  
  display.display();
}

void showTempMeasuring() {
  selectTCA(OLED_CHANNEL);
  int progress = (tempProgress * 100) / TEMP_ANALYSIS_DURATION;
  if (progress > 100) progress = 100;
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Top bar
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("TEMPERATURE");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // "ANALYZING" text
  display.setTextSize(2);
  display.setCursor(10, 16);
  display.print("ANALYZING");
  
  // Progress percentage
  display.setTextSize(1);
  display.setCursor(0, 38);
  display.print("Progress: ");
  display.print(progress);
  display.print("%");
  
  // Progress bar
  display.drawRect(0, 44, 128, 10, SSD1306_WHITE);
  display.fillRect(2, 46, (progress * 124) / 100, 6, SSD1306_WHITE);
  
  display.display();
}

void showTempResults() {
  selectTCA(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // "RESULT" header
  display.setTextSize(1);
  display.setCursor(35, 0);
  display.println("RESULT");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // Temperature value
  display.setTextSize(2);
  display.setCursor(15, 20);
  if (finalTemp > 0) {
    display.print(finalTemp, 1);
  } else {
    display.print("--");
  }
  display.setTextSize(1);
  display.print(" °C");
  
  // Status
  display.setTextSize(1);
  display.setCursor(0, 50);
  if (finalTemp > 0) {
    if (finalTemp > 37.5) {
      display.println("⚠️ WARNING: FEVER!");
    } else if (finalTemp < 35.0) {
      display.println("⚠️ WARNING: LOW TEMP!");
    } else {
      display.println("✅ NORMAL");
    }
  }
  
  // Bottom bar
  display.drawLine(0, 55, 128, 55, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 58);
  display.print("PRESS ACTION TO SEND");
  
  display.display();
}

// ============================================================
// ========== MODE 1: MPU6050 FUNCTIONS ==========
// ============================================================

bool initMPU6050() {
  selectTCA(3);
  delay(50);
  
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND!");
    return false;
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 Connected!");
  return true;
}

void readMPU6050() {
  if (!mpuReady) return;
  
  selectTCA(3);
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  
  currentPitch = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 180.0 / PI;
  currentRoll = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / PI;
  
  if (currentMode == MODE_FALL_DETECTION) {
    processFallDetection();
  }
}

void processFallDetection() {
  bool isUpright = false;
  if (totalAccel > 8.0 && totalAccel < 11.0) {
    if (abs(accelZ) > 7.0 && abs(accelX) < 3.0 && abs(accelY) < 3.0) {
      isUpright = true;
    }
  }
  
  if (!referenceSet && isUpright) {
    normalPitch = currentPitch;
    normalRoll = currentRoll;
    referenceSet = true;
    Serial.println("✅ REFERENCE ORIENTATION SET!");
  }
  
  bool xCondition = abs(accelX) > FALL_X_THRESHOLD;
  bool yCondition = abs(accelY) > FALL_Y_THRESHOLD;
  bool zCondition = accelZ < FALL_Z_THRESHOLD;
  bool fallCondition = xCondition || yCondition || zCondition;
  
  if (fallCondition && fallStatus == FALL_GOOD) {
    fallStatus = FALL_DETECTED;
    fallDetectTime = millis();
    Serial.println("⚠️ FALL CONDITION DETECTED!");
    displayFallStatus();
  }
  
  if (fallStatus == FALL_DETECTED) {
    if (millis() - fallDetectTime >= FALL_DEBOUNCE_TIME) {
      fallStatus = FALL_CONFIRMED;
      fallActive = true;
      Serial.println("🚨 FALL CONFIRMED!");
      Serial.println("========================================");
      Serial.println("🚨 FALL DETECTED!");
      Serial.println("========================================");
      
      if (!alertSentForThisFall) {
        sendFallAlert();
        alertSentForThisFall = true;
        smsSent = true;
      }
      displayFallStatus();
    }
    if (!fallCondition) {
      fallStatus = FALL_GOOD;
      Serial.println("⚠️ False alarm - Resetting");
      displayFallStatus();
    }
  }
  
  if (fallStatus == FALL_CONFIRMED) {
    if (isUpright) {
      if (normalStartTime == 0) {
        normalStartTime = millis();
      } else if (millis() - normalStartTime >= NORMAL_DEBOUNCE_TIME) {
        fallStatus = FALL_GOOD;
        fallActive = false;
        alertSentForThisFall = false;
        smsSent = false;
        normalStartTime = 0;
        Serial.println("✅ BACK TO UPRIGHT POSITION - GOOD!");
        displayFallStatus();
      }
    } else {
      normalStartTime = 0;
    }
  }
  
  displayFallStatus();
}

// ============================================================
// ========== GPS FUNCTIONS ==========
// ============================================================

void initGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Initialized");
}

void updateGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        satellites = gps.satellites.value();
        gpsFix = true;
        lastGpsUpdate = millis();
      }
    }
  }
  
  if (gpsFix && (millis() - lastGpsUpdate > GPS_FIX_TIMEOUT)) {
    gpsFix = false;
  }
}

String getLocationString() {
  if (gpsFix && latitude != 0 && longitude != 0) {
    String latString = String(latitude, 6);
    String lngString = String(longitude, 6);
    return "https://www.google.com/maps?q=" + latString + "," + lngString;
  }
  return "Location not available";
}

// ============================================================
// ========== GSM FUNCTIONS ==========
// ============================================================

void initGSM() {
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(1000);
  
  Serial.println("Initializing GSM...");
  
  for (int i = 0; i < 3; i++) {
    gsmSerial.println("AT");
    delay(500);
    if (gsmSerial.available()) {
      String response = gsmSerial.readString();
      if (response.indexOf("OK") > 0) {
        gsmReady = true;
        Serial.println("GSM Module Responding!");
        break;
      }
    }
  }
  
  if (!gsmReady) {
    Serial.println("GSM Module not responding!");
    return;
  }
  
  gsmSerial.println("AT+CMGF=1");
  delay(500);
  gsmSerial.println("AT+CNMI=2,2,0,0,0");
  delay(500);
  
  Serial.println("GSM Initialized Successfully!");
  sendPowerOnSMS();
}

void sendSMS(String message) {
  if (!gsmReady) {
    Serial.println("GSM Not Ready!");
    return;
  }
  
  Serial.println("📤 Sending SMS...");
  gsmSerial.println("AT+CMGS=\"" + String(SOS_PHONE_NUMBER) + "\"");
  delay(1000);
  gsmSerial.print(message);
  delay(100);
  gsmSerial.write(26);
  delay(5000);
  
  if (gsmSerial.available()) {
    String response = gsmSerial.readString();
    if (response.indexOf("OK") > 0 || response.indexOf("+CMGS") > 0) {
      Serial.println("✅ SMS Sent Successfully!");
      smsSent = true;
    }
  }
}

void sendPowerOnSMS() {
  String smsContent = "System Powered On: Device is now active and tracking.";
  sendSMS(smsContent);
}

void sendFallAlert() {
  String smsContent = "🚨 FALL DETECTED! 🚨\n\n";
  smsContent += "Emergency Alert!\n\n";
  smsContent += "📍 Location:\n" + getLocationString() + "\n\n";
  smsContent += "📊 Fall Details:\n";
  smsContent += "X: " + String(accelX, 1) + " m/s²\n";
  smsContent += "Y: " + String(accelY, 1) + " m/s²\n";
  smsContent += "Z: " + String(accelZ, 1) + " m/s²\n\n";
  smsContent += "🆘 Help is on the way!";
  
  sendSMS(smsContent);
  
  Serial.println("========================================");
  Serial.println("🚨 FALL ALERT SENT!");
  Serial.println("========================================");
  Serial.println(smsContent);
  Serial.println("========================================");
}

void sendPulseReport() {
  String message = "PULSE OXIMETER REPORT\n\n";
  message += "Location:\n" + getLocationString() + "\n\n";
  message += "Heart Rate: " + String((int)heartRate) + " BPM\n";
  message += "SpO2: " + String((int)spo2) + "%\n";
  message += "\nTime: " + String(millis() / 1000) + "s";
  
  sendSMS(message);
}

void sendTempReport() {
  String message = "TEMPERATURE REPORT\n\n";
  message += "Location:\n" + getLocationString() + "\n\n";
  message += "Temperature: " + String(finalTemp, 1) + " C\n";
  message += "\nTime: " + String(millis() / 1000) + "s";
  
  sendSMS(message);
}

// ============================================================
// ========== BUTTON FUNCTIONS ==========
// ============================================================

void checkButtons() {
  bool reading1 = digitalRead(BUTTON_MODE);
  if (reading1 != lastModeButtonState) {
    lastModeDebounce = millis();
  }
  if ((millis() - lastModeDebounce) > DEBOUNCE_DELAY) {
    if (reading1 == LOW && !modeButtonPressed) {
      modeButtonPressed = true;
      handleModeButton();
    }
    if (reading1 == HIGH && modeButtonPressed) {
      modeButtonPressed = false;
    }
  }
  lastModeButtonState = reading1;
  
  bool reading2 = digitalRead(BUTTON_RESET);
  if (reading2 != lastResetButtonState) {
    lastResetDebounce = millis();
  }
  if ((millis() - lastResetDebounce) > DEBOUNCE_DELAY) {
    if (reading2 == LOW && !resetButtonPressed) {
      resetButtonPressed = true;
      handleResetButton();
    }
    if (reading2 == HIGH && resetButtonPressed) {
      resetButtonPressed = false;
    }
  }
  lastResetButtonState = reading2;
  
  bool reading3 = digitalRead(BUTTON_ACTION);
  if (reading3 != lastActionButtonState) {
    lastActionDebounce = millis();
  }
  if ((millis() - lastActionDebounce) > DEBOUNCE_DELAY) {
    if (reading3 == LOW && !actionButtonPressed) {
      actionButtonPressed = true;
      handleActionButton();
    }
    if (reading3 == HIGH && actionButtonPressed) {
      actionButtonPressed = false;
    }
  }
  lastActionButtonState = reading3;
}

void handleModeButton() {
  Serial.println("========================================");
  Serial.println("🔄 MODE Button Pressed!");
  
  if (currentMode == MODE_PULSE_OXIMETER && (measuring || pulseResultShown)) {
    Serial.println("⚠️ Complete pulse measurement first!");
    displayMessage("⏳ Complete", "Measurement first");
    return;
  }
  if (currentMode == MODE_TEMPERATURE && tempMeasuring) {
    Serial.println("⚠️ Complete temperature first!");
    displayMessage("⏳ Complete", "Temperature first");
    return;
  }
  
  if (currentMode == MODE_FALL_DETECTION) {
    currentMode = MODE_PULSE_OXIMETER;
    resetPulseMeasurement();
    fingerDetected = false;
    measuring = false;
    pulseResultShown = false;
    Serial.println("▶️ Mode: PULSE OXIMETER");
    showPlaceFinger();
  } else if (currentMode == MODE_PULSE_OXIMETER) {
    currentMode = MODE_TEMPERATURE;
    tempMeasuring = false;
    tempAnalysisComplete = false;
    tempDataRecorded = false;
    finalTemp = 0;
    tempCount = 0;
    Serial.println("▶️ Mode: TEMPERATURE");
    showTempWaiting();
  } else if (currentMode == MODE_TEMPERATURE) {
    currentMode = MODE_FALL_DETECTION;
    Serial.println("▶️ Mode: FALL DETECTION");
    displayFallStatus();
  }
  
  Serial.print("Current Mode: ");
  Serial.println(modeNames[currentMode]);
  Serial.println("========================================");
}

void handleResetButton() {
  Serial.println("========================================");
  Serial.println("🔄 RESET Button Pressed!");
  
  if (currentMode == MODE_FALL_DETECTION) {
    if (fallStatus == FALL_CONFIRMED) {
      fallStatus = FALL_GOOD;
      fallActive = false;
      alertSentForThisFall = false;
      smsSent = false;
      normalStartTime = 0;
      Serial.println("✅ Fall reset (manual)!");
      displayMessage("✅ Reset", "Fall Cleared");
      displayFallStatus();
    } else {
      Serial.println("No active fall");
      displayMessage("No Active Fall", "System OK");
    }
  } else if (currentMode == MODE_PULSE_OXIMETER) {
    resetPulseMeasurement();
    fingerDetected = false;
    measuring = false;
    pulseResultShown = false;
    Serial.println("✅ Pulse reset!");
    showPlaceFinger();
  } else if (currentMode == MODE_TEMPERATURE) {
    resetTempData();
    Serial.println("✅ Temperature reset!");
    showTempWaiting();
  }
  Serial.println("========================================");
}

void handleActionButton() {
  Serial.println("========================================");
  Serial.println("🎯 ACTION Button Pressed!");
  
  if (currentMode == MODE_FALL_DETECTION) {
    if (fallStatus == FALL_CONFIRMED) {
      if (!alertSentForThisFall) {
        Serial.println("Resending alert...");
        sendFallAlert();
        alertSentForThisFall = true;
      } else {
        Serial.println("⚠️ Alert already sent");
        displayMessage("⚠️ Already Sent", "Alert sent for this fall");
      }
    } else {
      Serial.println("⚠️ Test Fall!");
      fallStatus = FALL_CONFIRMED;
      fallActive = true;
      fallDetectTime = millis();
      if (!alertSentForThisFall) {
        sendFallAlert();
        alertSentForThisFall = true;
      }
      displayFallStatus();
    }
  } else if (currentMode == MODE_PULSE_OXIMETER) {
    if (pulseResultShown && heartRate > 0 && spo2 > 0) {
      Serial.print("HR: ");
      Serial.print(heartRate, 0);
      Serial.print(" | SpO2: ");
      Serial.println(spo2, 0);
      sendPulseReport();
    } else if (!fingerDetected) {
      Serial.println("Place finger");
      displayMessage("Place Finger", "on Sensor");
    }
  } else if (currentMode == MODE_TEMPERATURE) {
    if (tempAnalysisComplete && finalTemp > 0) {
      Serial.print("🌡️ Temp: ");
      Serial.print(finalTemp, 1);
      Serial.println(" °C");
      sendTempReport();
      showTempResults();
    } else if (!tempMeasuring && !tempAnalysisComplete) {
      Serial.println("🌡️ Starting temperature analysis...");
      startTempAnalysis();
    }
  }
  Serial.println("========================================");
}

// ============================================================
// ========== SETUP ==========
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("❤️ HEALTH MONITORING SYSTEM");
  Serial.println("TEMPERATURE MODE DISPLAY");
  Serial.println("========================================");

  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_RESET, INPUT_PULLUP);
  pinMode(BUTTON_ACTION, INPUT_PULLUP);
  Serial.println("Buttons: GPIO15=MODE, GPIO14=RESET, GPIO13=ACTION");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);

  if (!tcaDetected()) {
    Serial.println("❌ ERROR: TCA9548A NOT FOUND!");
    while (1) delay(1000);
  }
  Serial.println("✅ TCA9548A detected.");

  if (!initializeOLED()) {
    Serial.println("❌ ERROR: OLED NOT FOUND!");
    while (1) delay(1000);
  }
  Serial.println("✅ OLED detected.");

  max30100Ready = initializeMAX30100();
  if (!max30100Ready) {
    Serial.println("❌ ERROR: MAX30100 NOT FOUND!");
    showSensorError("MAX30100 NOT FOUND", "Check TCA CH0");
    while (1) delay(1000);
  }
  Serial.println("✅ MAX30100 detected.");

  mpuReady = initMPU6050();
  Serial.println(mpuReady ? "✅ MPU6050 Connected!" : "❌ MPU6050 NOT FOUND!");

  bmp280Ready = initBMP280();
  Serial.println(bmp280Ready ? "✅ BMP280 Connected!" : "❌ BMP280 NOT FOUND!");

  initGPS();
  initGSM();

  resetPulseMeasurement();

  Serial.println();
  Serial.println("========================================");
  Serial.println("✅ SYSTEM READY!");
  Serial.println("Mode: FALL DETECTION");
  Serial.println("========================================\n");
  
  lastDisplayUpdate = millis();
  fallStatus = FALL_GOOD;
  referenceSet = false;
  alertSentForThisFall = false;
  
  displayFallStatus();
}

// ============================================================
// ========== MAIN LOOP ==========
// ============================================================

void loop() {
  checkButtons();
  
  // ===== MODE 1: FALL DETECTION =====
  if (mpuReady && currentMode == MODE_FALL_DETECTION) {
    readMPU6050();
  }
  
  // ===== MODE 2: PULSE OXIMETER =====
  if (max30100Ready && currentMode == MODE_PULSE_OXIMETER) {
    uint16_t ir, red;
    if (readFIFO(ir, red)) {
      currentIR = ir;
      currentRED = red;
      
      bool newFingerDetected;
      if (fingerDetected) {
        newFingerDetected = currentIR > FINGER_REMOVE_THRESHOLD;
      } else {
        newFingerDetected = currentIR > FINGER_THRESHOLD;
      }
      
      if (newFingerDetected && !fingerDetected) {
        fingerDetected = true;
        measuring = true;
        pulseResultShown = false;
        resetPulseMeasurement();
        measurementStart = millis();
        lastPulseOLEDUpdate = 0;
        
        Serial.println();
        Serial.println("==============================");
        Serial.println("FINGER DETECTED");
        Serial.println("STARTING 10 SECOND ANALYSIS");
        Serial.println("==============================");
        showPulseMeasuring();
      }
      
      if (!newFingerDetected && fingerDetected) {
        fingerDetected = false;
        measuring = false;
        pulseResultShown = false;
        resetPulseMeasurement();
        Serial.println("FINGER REMOVED");
        showPlaceFinger();
      }
      
      if (fingerDetected && measuring) {
        if (sampleCount < TOTAL_SAMPLES) {
          irBuffer[sampleCount] = currentIR;
          redBuffer[sampleCount] = currentRED;
          sampleCount++;
        }
        
        if (millis() - lastPulseOLEDUpdate >= 500) {
          lastPulseOLEDUpdate = millis();
          showPulseMeasuring();
          Serial.print("Samples: ");
          Serial.print(sampleCount);
          Serial.print("/");
          Serial.println(TOTAL_SAMPLES);
        }
        
        if (sampleCount >= TOTAL_SAMPLES) {
          measuring = false;
          Serial.println("10 SECOND ANALYSIS COMPLETE");
          
          calculateHeartRate();
          calculateSpO2();
          printPulseResult();
          
          showPulseResults();
          pulseResultShown = true;
          
          if (heartRate > 0 && spo2 > 0) {
            sendPulseReport();
          }
        }
      }
    }
  }
  
  // ===== MODE 3: TEMPERATURE =====
  if (bmp280Ready && currentMode == MODE_TEMPERATURE) {
    readTemperature();
    if (tempMeasuring) {
      updateTempAnalysis();
    }
  }
  
  // ===== GPS =====
  updateGPS();
  
  // ===== DISPLAY UPDATE =====
  if (millis() - lastDisplayUpdate >= 500) {
    if (currentMode == MODE_FALL_DETECTION) {
      displayFallStatus();
    } else if (currentMode == MODE_TEMPERATURE && !tempMeasuring && !tempAnalysisComplete) {
      showTempWaiting();
    }
    lastDisplayUpdate = millis();
  }
  
  // ===== DEBUG =====
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 3000) {
    Serial.print("Mode: ");
    Serial.print(modeNames[currentMode]);
    Serial.print(" | Status: ");
    if (currentMode == MODE_FALL_DETECTION) {
      if (fallStatus == FALL_CONFIRMED) {
        Serial.print("FALL!");
        Serial.print(" | Alert: ");
        Serial.print(alertSentForThisFall ? "SENT" : "PENDING");
      } else if (fallStatus == FALL_DETECTED) {
        Serial.print("DETECTED");
      } else {
        Serial.print("GOOD");
      }
    } else if (currentMode == MODE_PULSE_OXIMETER) {
      Serial.print("HR: ");
      Serial.print(heartRate, 0);
      Serial.print(" | SpO2: ");
      Serial.print(spo2, 0);
    } else if (currentMode == MODE_TEMPERATURE) {
      Serial.print("Temp: ");
      Serial.print(currentTemp, 1);
      Serial.print("C");
    }
    Serial.print(" | GPS: ");
    Serial.print(gpsFix ? "OK" : "NO");
    Serial.println();
    lastDebug = millis();
  }
  
  delay(2);
}
