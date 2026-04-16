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

enum StepPhase { PHASE_IDLE, PHASE_IMPACT, PHASE_GROUND };

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
  Stat rangeAngleStat;
  Stat strikeForce;     // Newtons/kgf
  Stat liftForce;       // Newtons/kgf
  Stat groundTime;      // ms
  Stat cycleTime;       // ms
  Stat frequency;       // steps/min
  Stat avgAccHorizStat; // g-units (average horizontal acc during swing phase)
  Stat impactDurationStat; // ms
  Stat vibrationFreqStat;  // calculated as 60000 / duration / peaks
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
    impactDurationStat.reset();
    vibrationFreqStat.reset();
    errors = 0;
    hasData = false;
    totalTimeS = 0;
    startMs = millis();
  }
};

struct AccWindow {
  float buffer[30];
  int index = 0;

  void push(float val) {
    buffer[index] = val;
    index = (index + 1) % 30;
  }

  float getMax() const {
    float mx = -1e9f;
    for (int i = 0; i < 30; i++) {
      if (buffer[i] > mx)
        mx = buffer[i];
    }
    return mx;
  }
};

// ============================================================
// RAW Sample buffer
// ============================================================
#define RAW_BUF_SIZE 32 // Flush every 32 samples (~160ms at 200Hz)

struct RawSample {
  float acc;
  float pitch;
  float horizAcc;
  unsigned long timeMs;
};

// ============================================================
// Classes
// ============================================================

class SDLogger {
public:
  bool begin();
  void log(const char *line);
  void close();
  void logRaw(float acc, float pitch, float horizAcc, unsigned long timeMs);
  void flushRawBuffer();

private:
  SdFile file;
  SdFile rawFile;
  int c;
  RawSample rawBuf[RAW_BUF_SIZE];
  uint8_t rawBufHead = 0;
};

#endif // TYPES_H
