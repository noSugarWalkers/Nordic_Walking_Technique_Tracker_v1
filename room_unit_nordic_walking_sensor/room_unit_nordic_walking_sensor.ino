/*
 * room_unit_nordic_walking_sensor
 * Nordic Walking Fitness Tracker
 * Hardware: LiLyGo T-Display Bar (ESP32-S3 + BHI260AP + BUZZER)
 *
 * Modular structure (Arduino IDE Tabs):
 * - config.h: Pins and hardware constants
 * - types.h: Structs and enums
 * - power.ino: Power and battery management
 * - imu.ino: BHI260AP sensor handling
 * - logic.ino: Step detection and training logic
 * - storage.ino: SD card and CSV logging
 * - web.ino: WiFi and Web Server
 * - utils.ino: Helper functions and preferences
 */

#include "config.h"
#include "types.h"

#include <DNSServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <SensorBHI260AP.hpp>
#include <SensorLib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>
#define BOSCH_APP30_SHUTTLE_BHI260_FW
#include <BLEDevice.h>
#include <BoschFirmware.h>
#include <GaugeBQ27220.hpp>
#include <SdFat.h>
#include <XPowersLib.h>

// ============================================================
// Core Application State
// ============================================================
AppState appState = STATE_MENU_MAIN;
WiFiMode wifiMode = APP_WIFI_STA; // Default WiFi Mode

// Menu/UI State
int menuSel = 0;
int resultSel = 0;
int resultDetail = 0;

// Hardware control states
bool screenOn = true;
unsigned long lastInteractionTime = 0;
unsigned long btnStopPressStart = 0;
bool btnStopWasPressed = false;
unsigned long btnPwrPressStart = 0;
bool btnPwrWasPressed = false;

// ============================================================
// Core Hardware Instances
// ============================================================
SensorBHI260AP bhi;
bool imuReady = false;
unsigned long lastSettingsActivityMs = 0;
bool isSettingsActive() { return (millis() - lastSettingsActivityMs < 5000); }
SPIClass spiSD(HSPI);
SdFat sd;
bool sdAvailable = false;
SDLogger logger;
WebServer server(80);
DNSServer dnsServer;
Preferences prefs;
XPowersPPM PPM;
GaugeBQ27220 gauge;
bool gaugeEnable = false;
bool ppmEnable = false;
int batteryVoltage = 0;

// Shared buffers/vars
char buf[128];
String fname = "";
String currentFileName = "";
int testProgress = -1; // -1: idle, 0-100: progress
String testPath = "";
String staIP = "";
bool wifiConnected = false;
bool dnsStarted = false;

// Step Detection / Calibration
StepPhase stepPhase = PHASE_IDLE;
float forceThreshold = DEFAULT_FORCE_THRESHOLD;
float forceThresholdSq = forceThreshold * forceThreshold;
float peakImpactAcc = 0, peakLiftAcc = 0;
float strikeAngle = 0, liftAngle = 0;
unsigned long t_impact = 0, t_release = 0, t_prev_impact = 0;

bool isCalibratingForce = false;
unsigned long calForceStartMs = 0;
float calForceBuffer[300];
int calForceIndex = 0;
float calForceResultG = 1.0f;

TrainingData training;

// Preferences / Config (Moved here for visibility)
bool sdRecordEnable = true;
bool rawRecordEnable = false;
uint8_t poleLength = 115;
uint16_t poleWeightGrams = 278;
uint8_t userHeight = 175;
uint16_t sensorFreq = 130;
bool buzzerEnable = false;
float gFactor = 1.0f;
float forceMultiplier = 1.0f;
bool autoTrainingEnable = false;
float cal_pitch_offset = 0.0f;

// ============================================================
// Forward Declarations
// ============================================================
void scanAutoGesturesIMU(float acc, float pitch);
void processDiagnosticsIMU();
void processTrainingIMU(float acc, float pitch);
void setupWifi(WiFiMode wifi);
void setupServer();
void setupBHI();
void setupPPM();
void setupSD();
void checkHW();
void loadPrefs();
void savePrefs();
void startTraining();
void stopTraining();
void powerOff();
float getPitch();
float getLinAccMagSq();
float getAndResetPeakForce();
bool isSettingsActive();
void playMelody(MelodyType type);
void disableUnusedPeripherals();
uint8_t readPMIC(uint8_t reg);
void writePMIC(uint8_t reg, uint8_t val);
int getBatteryPercent();
float getBatteryVoltage();
void onRotationVector(uint8_t sensor_id, const uint8_t *data, uint32_t size,
                      uint64_t *timestamp, void *user_data);
void onLinearAcc(uint8_t sensor_id, const uint8_t *data, uint32_t size,
                 uint64_t *timestamp, void *user_data);

// Test Task for background processing
void testAlgorithmFromSD(String path);
void testTask(void *pvParameters) {
  testAlgorithmFromSD(testPath);
  vTaskDelete(NULL);
}

// Handlers (web.ino)
void handleRoot();
void handleResults();
void handleDownload();
void handleLoad();
void handleSettings();
void handleSettingsApi();
void handleSettingsPost();
void handleCalibrate();
void handleCalibrateForceStart();
void handleCalibrateForceStatus();
void handleDiag();
void handleDelete();
String buildResultsHTML();
String buildSettingsHTML();
String buildResultsJSON();

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  disableUnusedPeripherals();

  // Prefs
  loadPrefs();

  // Physical Buttons
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_PWR, INPUT_PULLUP);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  playMelody(MELODY_START);

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);  // Fast Mode I2C — 4x throughput for BHI260AP FIFO

  setupBHI();
  setupPPM();
  setupSD();

  // Start in STA mode
  setupWifi(APP_WIFI_STA);

  if (wifiMode != APP_WIFI_OFF) {
    setupServer();
  }

  delay(1000);
  playMelody(MELODY_READY);
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  static unsigned long lastHwCheckMs = 0;
  unsigned long now = millis();

  // Hardware check (PMIC/battery) — once per second to free I2C bus
  if (now - lastHwCheckMs > 1000) {
    checkHW();
    lastHwCheckMs = now;
  }

  if (imuReady) {
    bhi.update();

    // Force Calibration handling
    if (isCalibratingForce) {
      if (now - calForceStartMs < 3000) {
        if (calForceIndex < 300) {
          calForceBuffer[calForceIndex++] = getLinAccMagSq();
        }
      } else {
        isCalibratingForce = false;
      }
    }
  }

  checkButtons();

  // WiFi/DNS polling — skip entirely during active training
  if (appState != STATE_TRAINING_ACTIVE) {
    if (dnsStarted)
      dnsServer.processNextRequest();
  }

  if (wifiConnected) server.handleClient();
}
