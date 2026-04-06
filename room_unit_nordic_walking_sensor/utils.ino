#include "config.h"
#include "types.h"

// ============================================================
// Preference Globals (externs from main)
// ============================================================
extern Preferences prefs;
extern bool sdRecordEnable;
extern bool rawRecordEnable;
extern uint8_t poleLength;
extern uint16_t poleWeightGrams;
extern uint8_t userHeight;
extern uint16_t sensorFreq;
extern bool buzzerEnable;
extern float gFactor;
extern float forceMultiplier;

// ============================================================
// Buzzer Functions
// ============================================================

void playTone(int freq, int duration) {
  if (freq == 0) {
    noTone(BUZZER_PIN);
    delay(duration);
  } else {
    tone(BUZZER_PIN, freq, duration);
    delay(duration * 1.3); 
  }
}

void playMelody(MelodyType type) {
  if (!buzzerEnable) return;

  switch (type) {
    case MELODY_START: {
      int notes[] = {659, 784, 988, 1047}; 
      int dur[] = {120, 120, 150, 180};
      for (int i = 0; i < 4; i++) playTone(notes[i], dur[i]);
      break;
    }
    case MELODY_MEASURE_START: {
      int notes[] = {784, 1047}; 
      int dur[] = {100, 120};
      for (int i = 0; i < 2; i++) playTone(notes[i], dur[i]);
      break;
    }
    case MELODY_MEASURE_STOP: {
      int notes[] = {523, 440, 349}; 
      int dur[] = {120, 140, 160};
      for (int i = 0; i < 3; i++) playTone(notes[i], dur[i]);
      break;
    }
    case MELODY_SHUTDOWN: {
      int notes[] = {784, 659, 523, 392}; 
      int dur[] = {140, 140, 180, 220};
      for (int i = 0; i < 4; i++) playTone(notes[i], dur[i]);
      break;
    }
    case MELODY_READY: {
      int notes[] = {988, 1319}; 
      int dur[] = {90, 110};
      for (int i = 0; i < 2; i++) playTone(notes[i], dur[i]);
      break;
    }
    case MELODY_AP: {
      int notes[] = { 988, 1319, 1760 };   // B5, E6, A6
      int dur[]   = { 120, 120, 160 };
      for (int i = 0; i < 3; i++) playTone(notes[i], dur[i]);
      break;
    }
  }
}

// ============================================================
// Hardware Cleanup
// ============================================================

void disableUnusedPeripherals() {
  //Disable BLE
  btStop();

  int unusedPins[] = {1, 21, 6, 7, 8, 5, 40};
  for (int p : unusedPins) {
    pinMode(p, INPUT_PULLDOWN);
  }
  // LCD Backlight Off (PIN 15)
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
}

// ============================================================
// Persistence
// ============================================================

void loadPrefs() {
  prefs.begin("nws", false);
  forceThreshold = prefs.getFloat("threshold", DEFAULT_FORCE_THRESHOLD);
  forceThresholdSq = forceThreshold * forceThreshold;
  cal_pitch_offset = prefs.getFloat("cal_offset", 0.0f);
  sdRecordEnable = prefs.getBool("sd_record", true);
  rawRecordEnable = prefs.getBool("raw_record", false);
  poleLength = prefs.getUChar("pole_len", 115);
  poleWeightGrams = prefs.getUShort("pole_weight", 278);
  userHeight = prefs.getUChar("user_height", 175);
  sensorFreq = prefs.getUShort("sens_freq", 130);
  buzzerEnable = prefs.getBool("buzzer_en", false);
  autoTrainingEnable = prefs.getBool("auto_train", false);
  gFactor = prefs.getFloat("g_factor", 1.0f);
  forceMultiplier = prefs.getFloat("force_mult", 1.0f);
  prefs.end();
}

void savePrefs() {
  prefs.begin("nws", false);
  prefs.putFloat("threshold", forceThreshold);
  prefs.putFloat("cal_offset", cal_pitch_offset);
  prefs.putBool("sd_record", sdRecordEnable);
  prefs.putBool("raw_record", rawRecordEnable);
  prefs.putUChar("pole_len", poleLength);
  prefs.putUShort("pole_weight", poleWeightGrams);
  prefs.putUChar("user_height", userHeight);
  prefs.putUShort("sens_freq", sensorFreq);
  prefs.putBool("buzzer_en", buzzerEnable);
  prefs.putBool("auto_train", autoTrainingEnable);
  prefs.putFloat("g_factor", gFactor);
  prefs.putFloat("force_mult", forceMultiplier);
  prefs.end();
}
