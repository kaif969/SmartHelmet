/*
ESP32 Smart Helmet - Complete Firmware
Version: 1.1.0 (Enhanced with SOS Alert System)
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_bt.h"
#include "esp_timer.h"
#include "esp_system.h"

// Version Information
#define FIRMWARE_VERSION F("1.1.0")
#define HARDWARE_VERSION F("v1.0")

// Pin Definitions
#define I2C_SDA 21
#define I2C_SCL 22
#define GPS_RX 16
#define GPS_TX 17
#define BATTERY_PIN 34
#define BUTTON_PIN 13
#define LED_PIN 2
#define BUZZER_PIN 4

// Constants
const float IMPACT_THRESHOLD = 15.0;
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.3;
const float LOW_BATTERY_THRESHOLD = 15.0;
const float CRITICAL_BATTERY_THRESHOLD = 5.0;
unsigned long updateRate = 1000;  // Variable update rate
const unsigned long GPS_UPDATE_RATE = 2000;
const unsigned long BATTERY_CHECK_INTERVAL = 60000;
const int MAX_RETRY_ATTEMPTS = 3;
const size_t MAX_LOG_SIZE = 1048576;
const int MAX_ERROR_LOGS = 50;
const unsigned long SOS_BUTTON_HOLD_TIME = 1500; // Time in ms to hold button for SOS
const unsigned long SOS_COOLDOWN_PERIOD = 10000; // Cooldown period between SOS triggers

// Global Objects
HardwareSerial gpsSerial(1);
Adafruit_GPS GPS(&gpsSerial);
Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;
esp_adc_cal_characteristics_t adc_chars;
QueueHandle_t errorQueue;

// Store boot count in RTC memory to persist across reboots
RTC_DATA_ATTR int bootCount = 0;

// System State
struct SystemState {
  bool mpuInitialized = false;
  bool gpsInitialized = false;
  bool spiffsInitialized = false;
  bool bluetoothInitialized = false;
  float lastBatteryLevel = 100.0;
  unsigned long lastGPSUpdate = 0;
  unsigned long lastBatteryCheck = 0;
  unsigned long lastDataLog = 0;
  uint32_t bootCount = 0;
  bool isCharging = false;
  bool lowPowerMode = false;
  int errorCount = 0;
  unsigned long lastSOSTriggered = 0;
  bool sosActive = false;
} state;

// Sensor Data
struct SensorData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
  double latitude;
  double longitude;
  float speed;
  float altitude;
  int satellites;
  bool gpsFix;
  unsigned long timestamp;
  float batteryVoltage;
} sensorData;

// Error Handling
struct ErrorLog {
  char timestamp[16];
  char component[16];
  char message[64];
  int code;
};

// Function declarations
void initErrorHandling();
void logError(const char* component, const char* message, int code);
void handleErrors();
void getMacAddress(char* macStr, size_t len);
void initializeADC();
float readBatteryVoltage();
void playAlertTone();
void checkStorageSpace();
void cleanupOldLogs();
void setupSPIFFS();
void handleCommand(const char* command);
void sendAlert(const char* message);
void sendSOSAlert();
void logData();
void enterLowPowerMode();
void exitLowPowerMode();
void handleSystemFailure();
void updateSensors();
void updateGPS();
void updateBattery();
void updateBluetooth();
void checkImpact();
void checkSOSButton();

// Forward declare setup functions
bool setupMPU();
bool setupGPS();
bool setupBluetooth();

// ─── UTILITY FUNCTIONS ───────────────────────────────────────────────

void getMacAddress(char* macStr, size_t len) {
  const uint8_t* mac = esp_bt_dev_get_address();
  snprintf(macStr, len, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void initializeADC() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

float readBatteryVoltage() {
  const int SAMPLES = 10;
  int sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += adc1_get_raw(ADC1_CHANNEL_6);
    delay(5);
  }
  uint32_t voltage = esp_adc_cal_raw_to_voltage(sum / SAMPLES, &adc_chars);
  return (float)voltage / 1000.0 * 2;  // Voltage divider ratio
}

void playAlertTone() {
  if (!state.lowPowerMode) {
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 2000, 200);
      delay(300);
      noTone(BUZZER_PIN);
      delay(100);
    }
  }
}

void playSOSTone() {
  // Play SOS pattern (... --- ...)
  if (!state.lowPowerMode) {
    // Dot dot dot
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 2000, 150);
      delay(200);
      noTone(BUZZER_PIN);
      delay(100);
    }
    delay(300);
    
    // Dash dash dash
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 2000, 450);
      delay(500);
      noTone(BUZZER_PIN);
      delay(100);
    }
    delay(300);
    
    // Dot dot dot
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 2000, 150);
      delay(200);
      noTone(BUZZER_PIN);
      delay(100);
    }
  }
}

// ─── ERROR HANDLING ───────────────────────────────────────────────────

void initErrorHandling() {
  errorQueue = xQueueCreate(MAX_ERROR_LOGS, sizeof(ErrorLog));
  if (!errorQueue) {
    Serial.println(F("Failed to create error queue!"));
  }
}

void logError(const char* component, const char* message, int code) {
  ErrorLog error;
  snprintf(error.timestamp, sizeof(error.timestamp), "%lu", millis());
  strncpy(error.component, component, sizeof(error.component) - 1);
  strncpy(error.message, message, sizeof(error.message) - 1);
  error.code = code;
  
  if (xQueueSend(errorQueue, &error, 0) != pdTRUE) {
    ErrorLog oldError;
    xQueueReceive(errorQueue, &oldError, 0);
    xQueueSend(errorQueue, &error, 0);
  }
  
  Serial.printf(F("ERROR [%s] %s: %s (Code: %d)\n"),
                error.timestamp,
                error.component,
                error.message,
                error.code);
}

void handleErrors() {
  ErrorLog error;
  while (xQueueReceive(errorQueue, &error, 0) == pdTRUE) {
    if (state.spiffsInitialized) {
      File errorLog = SPIFFS.open("/errors.log", FILE_APPEND);
      if (errorLog) {
        StaticJsonDocument<192> doc;
        doc["ts"] = error.timestamp;
        doc["comp"] = error.component;
        doc["msg"] = error.message;
        doc["code"] = error.code;
        serializeJson(doc, errorLog);
        errorLog.println();
        errorLog.close();
      }
    }
  }
}

// ─── SETUP FUNCTIONS ───────────────────────────────────────────────────

bool setupMPU() {
  Wire.beginTransmission(0x68);
  byte error = Wire.endTransmission();
  if (error == 0) {
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      state.mpuInitialized = true;
      return true;
    }
  }
  state.mpuInitialized = false;
  return false;
}

bool setupGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    if (GPS.available()) {
      state.gpsInitialized = true;
      return true;
    }
    delay(10);
  }
  
  state.gpsInitialized = false;
  return false;
}

bool setupBluetooth() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  if (!btStart()) {
    Serial.println(F("⚠ Bluetooth controller init failed"));
    return false;
  }
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println(F("⚠ Bluetooth stack initialization failed"));
    return false;
  }
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println(F("⚠ Bluetooth stack enable failed"));
    return false;
  }
  
  // Create a unique device name
  char deviceName[24];
  snprintf(deviceName, sizeof(deviceName), "SmartHelmet_%02X", random(0xFF));
  
  if (!SerialBT.begin(deviceName)) {
    Serial.println(F("⚠ SerialBT initialization failed"));
    return false;
  }
  
  esp_bt_cod_t cod;
  cod.major = 0x05;  // Peripheral
  cod.minor = 0x40;  // Wearable
  cod.service = 0x200000;  // Audio
  esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
  
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  digitalWrite(LED_PIN, HIGH);
  state.bluetoothInitialized = true;
  Serial.print(F("✅ Bluetooth Started as: "));
  Serial.println(deviceName);
  return true;
}

// ─── UPDATE FUNCTIONS ──────────────────────────────────────────────────

void updateSensors() {
  if (!state.mpuInitialized) return;
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    logError("MPU6050", "Failed to read sensor data", 203);
    return;
  }
  sensorData.accelX = a.acceleration.x;
  sensorData.accelY = a.acceleration.y;
  sensorData.accelZ = a.acceleration.z;
  sensorData.gyroX = g.gyro.x;
  sensorData.gyroY = g.gyro.y;
  sensorData.gyroZ = g.gyro.z;
  sensorData.temperature = temp.temperature;
  sensorData.timestamp = millis();
}

void updateGPS() {
  if (!state.gpsInitialized) return;
  while (gpsSerial.available()) {
    char c = GPS.read();
    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
      sensorData.gpsFix = GPS.fix;
      if (GPS.fix) {
        sensorData.latitude = GPS.latitudeDegrees;
        sensorData.longitude = GPS.longitudeDegrees;
        sensorData.speed = GPS.speed * 1.852;  // knots -> km/h
        sensorData.altitude = GPS.altitude;
        sensorData.satellites = GPS.satellites;
      }
    }
  }
}

void updateBattery() {
  float voltage = readBatteryVoltage();
  sensorData.batteryVoltage = voltage;
  float percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  state.lastBatteryLevel = constrain(percentage, 0, 100);
  
  bool previousCharging = state.isCharging;
  state.isCharging = (voltage > BATTERY_MAX_VOLTAGE);
  
  if (state.lastBatteryLevel < CRITICAL_BATTERY_THRESHOLD) {
    sendAlert("CRITICAL BATTERY LEVEL!");
    enterLowPowerMode();
  } else if (state.lastBatteryLevel < LOW_BATTERY_THRESHOLD) {
    char buf[32];
    snprintf(buf, sizeof(buf), "LOW BATTERY: %.1f%%", state.lastBatteryLevel);
    sendAlert(buf);
  }
  
  if (state.isCharging != previousCharging) {
    sendAlert(state.isCharging ? "Charging Started" : "Charging Stopped");
  }
}

void updateBluetooth() {
  if (!state.bluetoothInitialized) return;
  
  // Send sensor JSON only if a client is connected.
  if (SerialBT.hasClient()) {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= updateRate) {
      StaticJsonDocument<192> doc;
      doc["temp"] = sensorData.temperature;
      doc["ax"] = sensorData.accelX;
      doc["ay"] = sensorData.accelY;
      doc["az"] = sensorData.accelZ;
      doc["gx"] = sensorData.gyroX;
      doc["gy"] = sensorData.gyroY;
      doc["gz"] = sensorData.gyroZ;
      doc["bat"] = state.lastBatteryLevel;
      doc["chg"] = state.isCharging;
      if (sensorData.gpsFix) {
        doc["lat"] = sensorData.latitude;
        doc["lon"] = sensorData.longitude;
        doc["spd"] = sensorData.speed;
        doc["alt"] = sensorData.altitude;
        doc["sat"] = sensorData.satellites;
      }
      char jsonBuffer[192];
      size_t n = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
      if (n > 0) {
        SerialBT.println(jsonBuffer);
      }
      lastUpdate = millis();
    }
  }
  
  // Process incoming commands using a fixed buffer
  char cmdBuffer[64];
  while (SerialBT.available()) {
    int len = SerialBT.readBytesUntil('\n', cmdBuffer, sizeof(cmdBuffer) - 1);
    if (len > 0) {
      cmdBuffer[len] = '\0';
      handleCommand(cmdBuffer);
    }
  }
}

void handleCommand(const char* command) {
  // Remove leading/trailing spaces if needed (assume commands are short)
  if (strcmp(command, "DISCONNECT") == 0) {
    digitalWrite(LED_PIN, LOW);
    SerialBT.println(F("ACK:DISCONNECT"));
  }
  else if (strcmp(command, "SOS") == 0) {
    sendSOSAlert();
  }
  else if (strcmp(command, "STATUS") == 0) {
    StaticJsonDocument<192> status;
    char macStr[18];
    getMacAddress(macStr, sizeof(macStr));
    status["dev"] = String("SmartHelmet_") + String(&macStr[9]);  // last part
    status["mac"] = macStr;
    status["fw"] = FIRMWARE_VERSION;
    status["hw"] = F(HARDWARE_VERSION);
    status["mpu"] = state.mpuInitialized;
    status["gps"] = state.gpsInitialized;
    status["bat"] = state.lastBatteryLevel;
    status["chg"] = state.isCharging;
    status["lp"] = state.lowPowerMode;
    status["err"] = state.errorCount;
    status["up"] = millis() / 1000;
    status["sos"] = state.sosActive;
    char statusBuffer[192];
    serializeJson(status, statusBuffer, sizeof(statusBuffer));
    SerialBT.print(F("STATUS:"));
    SerialBT.println(statusBuffer);
  }
  else if (strcmp(command, "RESET") == 0) {
    SerialBT.println(F("ACK:RESET"));
    delay(100);
    ESP.restart();
  }
  else if (strcmp(command, "DEBUG") == 0) {
    File errorLog = SPIFFS.open("/errors.log", FILE_READ);
    if (errorLog) {
        while (errorLog.available()) {
            SerialBT.print(F("DEBUG:"));
            SerialBT.println(errorLog.readStringUntil('\n'));
        }
        errorLog.close();
    }
  }
  else if (strcmp(command, "CANCEL_SOS") == 0) {
    if (state.sosActive) {
      state.sosActive = false;
      sendAlert("SOS ALERT CANCELLED");
      SerialBT.println(F("ACK:SOS_CANCELLED"));
    } else {
      SerialBT.println(F("ERR:NO_ACTIVE_SOS"));
    }
  }
}

void checkImpact() {
  if (!state.mpuInitialized) return;
  float magnitude = sqrt(pow(sensorData.accelX, 2) + pow(sensorData.accelY, 2) + pow(sensorData.accelZ, 2));
  static unsigned long lastImpactTime = 0;
  if (magnitude > IMPACT_THRESHOLD && (millis() - lastImpactTime) > 1000) {
    char alertMsg[64];
    if (sensorData.gpsFix) {
      snprintf(alertMsg, sizeof(alertMsg), "IMPACT: %.1fg at %.6f,%.6f", magnitude, sensorData.latitude, sensorData.longitude);
    } else {
      snprintf(alertMsg, sizeof(alertMsg), "IMPACT: %.1fg", magnitude);
    }
    sendAlert(alertMsg);
    playAlertTone();
    lastImpactTime = millis();
  }
}

void sendSOSAlert() {
  unsigned long currentTime = millis();
  
  // Check if SOS is in cooldown period
  if (currentTime - state.lastSOSTriggered < SOS_COOLDOWN_PERIOD) {
    return;
  }
  
  state.sosActive = true;
  state.lastSOSTriggered = currentTime;
  
  // Play SOS tone
  playSOSTone();
  
  Serial.println(F("🚨 SOS EMERGENCY ALERT TRIGGERED"));
  
  if (state.bluetoothInitialized) {
    StaticJsonDocument<256> sosAlert;
    sosAlert["type"] = "SOS_EMERGENCY";
    sosAlert["ts"] = currentTime;
    
    // Include location if available
    if (sensorData.gpsFix) {
      sosAlert["lat"] = sensorData.latitude;
      sosAlert["lon"] = sensorData.longitude;
      sosAlert["alt"] = sensorData.altitude;
      sosAlert["spd"] = sensorData.speed;
    }
    
    // Include device info
    char macStr[18];
    getMacAddress(macStr, sizeof(macStr));
    sosAlert["mac"] = macStr;
    sosAlert["bat"] = state.lastBatteryLevel;
    
    // Add command for app to take action
    sosAlert["cmd"] = "SEND_SOS_ALERT";
    sosAlert["target"] = "POLICE,WOMEN_SAFETY,EMERGENCY_CONTACTS";
    
    char sosBuffer[256];
    serializeJson(sosAlert, sosBuffer, sizeof(sosBuffer));
    SerialBT.println(sosBuffer);
    
    // Additional alert with standard format for logging
    sendAlert("SOS EMERGENCY ALERT TRIGGERED");
  } else {
    // If Bluetooth is not available, still log the event
    sendAlert("SOS TRIGGERED - NO CONNECTIVITY!");
  }
  
  // Log the SOS event if storage is available
  if (state.spiffsInitialized) {
    File sosLog = SPIFFS.open("/sos.log", FILE_APPEND);
    if (sosLog) {
      StaticJsonDocument<192> sosRecord;
      sosRecord["ts"] = currentTime;
      sosRecord["type"] = "SOS_BUTTON";
      if (sensorData.gpsFix) {
        sosRecord["lat"] = sensorData.latitude;
        sosRecord["lon"] = sensorData.longitude;
      }
      serializeJson(sosRecord, sosLog);
      sosLog.println();
      sosLog.close();
    }
  }
}

void sendAlert(const char* message) {
  Serial.print(F("🚨 "));
  Serial.println(message);
  if (state.bluetoothInitialized && SerialBT.hasClient()) {
    StaticJsonDocument<192> alert;
    alert["type"] = "ALERT";
    alert["msg"] = message;
    alert["ts"] = millis();
    if (sensorData.gpsFix) {
      alert["lat"] = sensorData.latitude;
      alert["lon"] = sensorData.longitude;
    }
    char alertBuffer[192];
    serializeJson(alert, alertBuffer, sizeof(alertBuffer));
    SerialBT.println(alertBuffer);
  }
}

void logData() {
  if (!state.spiffsInitialized) return;
  StaticJsonDocument<192> doc;
  doc["ts"] = millis();
  doc["ax"] = sensorData.accelX;
  doc["ay"] = sensorData.accelY;
  doc["az"] = sensorData.accelZ;
  doc["gx"] = sensorData.gyroX;
  doc["gy"] = sensorData.gyroY;
  doc["gz"] = sensorData.gyroZ;
  doc["temp"] = sensorData.temperature;
  doc["bat"] = state.lastBatteryLevel;
  doc["chg"] = state.isCharging;
  if (sensorData.gpsFix) {
    doc["lat"] = sensorData.latitude;
    doc["lon"] = sensorData.longitude;
    doc["spd"] = sensorData.speed;
    doc["alt"] = sensorData.altitude;
    doc["sat"] = sensorData.satellites;
  }
  File dataFile = SPIFFS.open("/data.log", FILE_APPEND);
  if (dataFile) {
    serializeJson(doc, dataFile);
    dataFile.println();
    dataFile.close();
    checkStorageSpace();
  } else {
    logError("Storage", "Failed to open data log", 501);
  }
}

void setupSPIFFS() {
  state.spiffsInitialized = SPIFFS.begin(true);
  if (!state.spiffsInitialized) {
    logError("SPIFFS", "Initialization failed", 101);
  }
  checkStorageSpace();
}

void checkStorageSpace() {
  if (state.spiffsInitialized) {
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    float usagePercent = (float)usedBytes / totalBytes * 100;
    if (usagePercent > 90) {
      cleanupOldLogs();
    }
  }
}

void cleanupOldLogs() {
  if (!state.spiffsInitialized) return;
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    String fname = file.name();
    if (!file.isDirectory() && fname.endsWith(".log")) {
      if (file.size() > MAX_LOG_SIZE) {
        File newLog = SPIFFS.open("/temp.log", FILE_WRITE);
        if (newLog) {
          int lines = 0;
          while (file.available() && lines < 1000) {
            String line = file.readStringUntil('\n');
            newLog.println(line);
            lines++;
          }
          newLog.close();
          file.close();
          SPIFFS.remove(fname);
          SPIFFS.rename("/temp.log", fname);
        }
      }
    }
    file = root.openNextFile();
  }
}

void enterLowPowerMode() {
  if (!state.lowPowerMode) {
    state.lowPowerMode = true;
    if (state.gpsInitialized) {
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    }
    if (state.mpuInitialized) {
      mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    }
    updateRate = 5000;
    sendAlert("Entering Low Power Mode");
  }
}

void exitLowPowerMode() {
  if (state.lowPowerMode) {
    state.lowPowerMode = false;
    if (state.gpsInitialized) {
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    }
    if (state.mpuInitialized) {
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    updateRate = 1000;
    sendAlert("Exiting Low Power Mode");
  }
}

void handleSystemFailure() {
  state.errorCount++;
  Serial.print(F("⚠ System Failure Count: "));
  Serial.println(state.errorCount);
  sendAlert("SYSTEM FAILURE DETECTED");
  if (state.errorCount >= MAX_RETRY_ATTEMPTS) {
    Serial.println(F("Maximum retry attempts reached. Attempting recovery..."));
    if (!state.bluetoothInitialized) {
      Serial.println(F("Retrying Bluetooth setup..."));
      setupBluetooth();
    }
    if (!state.mpuInitialized) {
      Serial.println(F("Retrying MPU setup..."));
      setupMPU();
    }
    if (!state.gpsInitialized) {
      Serial.println(F("Retrying GPS setup..."));
      setupGPS();
    }
    state.errorCount = 0;
  }
}

void checkSOSButton() {
  static bool buttonPressed = false;
  static unsigned long buttonPressStartTime = 0;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Button is pressed (active LOW)
  if (currentButtonState == LOW) {
    // If this is the start of a new press
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressStartTime = millis();
      digitalWrite(LED_PIN, LOW); // Visual feedback - turn LED off
    } 
    // If button has been held long enough, trigger SOS
    else if ((millis() - buttonPressStartTime >= SOS_BUTTON_HOLD_TIME) && 
             (millis() - state.lastSOSTriggered >= SOS_COOLDOWN_PERIOD)) {
      sendSOSAlert();
      // Rapid LED blink to indicate SOS triggered
      for (int i = 0; i < 6; i++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
      }
    }
  } 
  // Button is released
  else if (buttonPressed) {
    buttonPressed = false;
    // If Bluetooth is connected, return LED to normal state
    if (state.bluetoothInitialized) {
      if (SerialBT.hasClient()) {
        // Return to blinking state if connected
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
    }
  }
}

// ─── SETUP & LOOP ─────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);
  bootCount++;
  Serial.printf("\n=== Smart Helmet Starting (Boot #%d) ===\n", bootCount);
  
  // Initialize GPIOs
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  // Error handling
  initErrorHandling();
  
  // SPIFFS initialization
  if (!SPIFFS.begin(true)) {
    Serial.println(F("⚠ SPIFFS Mount Failed - continuing without storage"));
  } else {
    state.spiffsInitialized = true;
    Serial.println(F("✅ SPIFFS Mounted"));
  }
  
  // MPU setup
  Serial.println(F("Setting up MPU..."));
  if (setupMPU()) {
    Serial.println(F("✅ MPU6050 initialized"));
  } else {
    Serial.println(F("⚠ MPU6050 failed - continuing without motion detection"));
  }
  
  // GPS setup
  Serial.println(F("Setting up GPS..."));
  if (setupGPS()) {
    Serial.println(F("✅ GPS initialized"));
  } else {
    Serial.println(F("⚠ GPS failed - continuing without location services"));
  }
  
  // ADC initialization for battery monitoring
  initializeADC();
  float batteryVoltage = readBatteryVoltage();
  float batteryPercentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  state.lastBatteryLevel = constrain(batteryPercentage, 0, 100);
  
  // Bluetooth setup (last)
  Serial.println(F("Setting up Bluetooth..."));
  if (setupBluetooth()) {
    Serial.println(F("✅ Bluetooth initialized"));
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println(F("⚠ Bluetooth failed - continuing without connectivity"));
    digitalWrite(LED_PIN, LOW);
  }
  
  // Final status printout
  Serial.println(F("\n=== Initialization Complete ==="));
  Serial.printf(F("- Battery: %.1f%%\n"), state.lastBatteryLevel);
  Serial.printf(F("- Bluetooth: %s\n"), state.bluetoothInitialized ? "OK" : "FAIL");
  Serial.printf(F("- MPU6050: %s\n"), state.mpuInitialized ? "OK" : "FAIL");
  Serial.printf(F("- GPS: %s\n"), state.gpsInitialized ? "OK" : "FAIL");
  Serial.printf(F("- Storage: %s\n"), state.spiffsInitialized ? "OK" : "FAIL");
  Serial.println(F("- SOS Button: ENABLED"));
  Serial.println(F("================================"));
}

void loop() {
  static unsigned long lastStatus = 0;
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  unsigned long currentMillis = millis();
  
  if (currentMillis < lastStatus) {
    lastStatus = currentMillis;
  }
  
  if (state.mpuInitialized) {
    updateSensors();
    checkImpact();
  }
  
  // Check SOS button (replaces the old button check)
  checkSOSButton();
  
  if (state.bluetoothInitialized) {
    updateBluetooth();
    // Only control LED if not in active SOS state
    if (!state.sosActive) {
      if (SerialBT.hasClient()) {
        if (currentMillis - lastBlink >= 1000) {
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState);
          lastBlink = currentMillis;
        }
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
    }
  } else {
    // If no Bluetooth and not in SOS mode
    if (!state.sosActive) {
      digitalWrite(LED_PIN, LOW);
    }
  }
  
  // Blink rapidly if in SOS mode
  if (state.sosActive) {
    if (currentMillis - lastBlink >= 200) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = currentMillis;
    }
  }
  
  if (currentMillis - lastStatus >= 5000) {
    Serial.printf(F("Status - BT:%d GPS:%d MPU:%d BAT:%.1f%% SOS:%d\n"),
                  state.bluetoothInitialized,
                  state.gpsInitialized,
                  state.mpuInitialized,
                  state.lastBatteryLevel,
                  state.sosActive);
    if (state.gpsInitialized && sensorData.gpsFix) {
      Serial.printf(F("GPS - Lat:%.6f Lon:%.6f Alt:%.1fm Spd:%.1fkm/h Sat:%d\n"),
                    sensorData.latitude,
                    sensorData.longitude,
                    sensorData.altitude,
                    sensorData.speed,
                    sensorData.satellites);
    }
    lastStatus = currentMillis;
  }
  
  if (state.gpsInitialized && (currentMillis - state.lastGPSUpdate >= GPS_UPDATE_RATE)) {
    updateGPS();
    state.lastGPSUpdate = currentMillis;
  }
  
  if (currentMillis - state.lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    updateBattery();
    state.lastBatteryCheck = currentMillis;
  }
  
  if (state.spiffsInitialized && (currentMillis - state.lastDataLog >= updateRate)) {
    logData();
    state.lastDataLog = currentMillis;
  }
  
  handleErrors();
  
  if (state.lastBatteryLevel > LOW_BATTERY_THRESHOLD && state.lowPowerMode) {
    exitLowPowerMode();
  }
  
  // If SOS was active but cooldown period has passed, reset the state
  if (state.sosActive && (currentMillis - state.lastSOSTriggered > 60000)) {
    state.sosActive = false;
  }
  
  delay(10);
}
