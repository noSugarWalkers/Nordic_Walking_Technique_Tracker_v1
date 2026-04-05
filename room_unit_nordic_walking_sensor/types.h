#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include <SdFat.h>

// ============================================================
// Enums
// ============================================================

enum AppState {
  STATE_MENU_MAIN,
  STATE_TRAINING_IDLE,
  STATE_TRAINING_ACTIVE,
  STATE_TRAINING_RESULTS
};

enum StepPhase { PHASE_IDLE, PHASE_IMPACT, PHASE_PUSH, PHASE_RELEASE };

enum WiFiMode { APP_WIFI_STA, APP_WIFI_STA_AP, APP_WIFI_AP, APP_WIFI_OFF };

enum MelodyType {
  MELODY_START,
  MELODY_MEASURE_START,
  MELODY_MEASURE_STOP,
  MELODY_SHUTDOWN,
  MELODY_READY,
  MELODY_AP
};

// ============================================================
// Structs
// ============================================================

struct Stat {
  float minV = 1e9f;
  float maxV = -1e9f;
  float sum = 0;
  long cnt = 0;

  void reset() {
    minV = 1e9f;
    maxV = -1e9f;
    sum = 0;
    cnt = 0;
  }

  void add(float v) {
    if (v < minV)
      minV = v;
    if (v > maxV)
      maxV = v;
    sum += v;
    cnt++;
  }

  float avg() const { return cnt ? sum / cnt : 0; }
};

struct TrainingData {
  Stat strikeAngleStat;
  Stat liftAngleStat;
  Stat strikeForce;     // Newtons/kgf
  Stat liftForce;       // Newtons/kgf
  Stat groundTime;      // ms
  Stat cycleTime;       // ms
  Stat frequency;       // steps/min
  Stat avgAccHorizStat; // g-units (average horizontal acc during swing phase)
  unsigned long startMs = 0;
  uint32_t totalTimeS = 0;
  long errors = 0;
  bool hasData = false;

  void reset() {
    strikeAngleStat.reset();
    liftAngleStat.reset();
    strikeForce.reset();
    liftForce.reset();
    groundTime.reset();
    cycleTime.reset();
    frequency.reset();
    avgAccHorizStat.reset();
    errors = 0;
    hasData = false;
    totalTimeS = 0;
    startMs = millis();
  }
};

// ============================================================
// Classes
// ============================================================

class SDLogger {
public:
  bool begin();
  void log(const char *line);
  void close();

private:
  SdFile file;
  int c;
};

#endif // TYPES_H
