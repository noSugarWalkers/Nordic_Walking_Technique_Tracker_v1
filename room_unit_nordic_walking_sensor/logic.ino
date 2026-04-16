#include "config.h"
#include "types.h"

// ============================================================
// Step Detection Globals (externs from main)
// ============================================================
extern StepPhase stepPhase;
extern float forceThreshold;
extern float forceThresholdSq;
extern TrainingData training;
extern bool autoTrainingEnable;
extern AppState appState;
extern float gFactor;
extern bool sdAvailable;
extern char buf[128];
extern SDLogger logger;
extern bool rawRecordEnable;

extern float peakImpactAcc;
extern float peakLiftAcc;
extern float strikeAngle;
extern float liftAngle;
extern unsigned long t_impact;
extern unsigned long t_release;
extern unsigned long t_prev_impact;

// Step calculation variables
float accHorizSumStep = 0; // sum of horiz acc during current swing phase (g)
int accHorizCountStep = 0; // sample count during current swing phase
int autoStartHitsCount = 0;
int autoStopHitsCount = 0;
unsigned long autoGestureStartMs = 0;
bool gestureStrikeActive = false;
unsigned long t_vibe_start = 0;
int vibe_peaks = 0;
float vibe_duration = 0;
bool vibe_active = false;
float last_vibe_acc = 0;
bool vibe_acc_increasing = false;

// Step Detection variables
AccWindow stepAccWindow;
float localMaxPitch = -1e9f;
float localMinPitch = 1e9f;
bool isTestingSession = false;

/**
 * @brief Scans for automatic start/stop gestures
 */
void scanAutoGesturesIMU(float acc, float pitch) {
  unsigned long now = millis();

  if (acc >= forceThresholdSq) {
    if (!gestureStrikeActive) {
      gestureStrikeActive = true;

      if (pitch >= 83.0f) {
        if (appState != STATE_TRAINING_ACTIVE) {
          if (autoStartHitsCount == 0 || (now - autoGestureStartMs > 2000)) {
            autoGestureStartMs = now;
            autoStartHitsCount = 1;
          } else {
            autoStartHitsCount++;
          }
          autoStopHitsCount = 0;

          if (autoStartHitsCount >= 2) {
            startTraining();
            autoStartHitsCount = 0;
            autoStopHitsCount = 0;
          }
        } else {
          if (autoStopHitsCount == 0 || (now - autoGestureStartMs > 3000)) {
            autoGestureStartMs = now;
            autoStopHitsCount = 1;
          } else {
            autoStopHitsCount++;
          }
          autoStartHitsCount = 0;

          if (autoStopHitsCount >= 3) {
            stopTraining();
            autoStartHitsCount = 0;
            autoStopHitsCount = 0;
          }
        }
      } else {
        autoStartHitsCount = 0;
        autoStopHitsCount = 0;
      }
    }
  } else if (acc < forceThresholdSq * 0.5f) {
    // Hysteresis to reset strike active flag
    gestureStrikeActive = false;
  }
}

/**
 * @brief Processes IMU data for diagnostics (Settings page)
 */
void processDiagnosticsIMU() {
  // getLinAccMagSq() already updates peakForceAccumulator when called in
  // onLinearAcc. This is a placeholder for future diagnostic processing.
}

/**
 * @brief Main IMU processing loop for step detection during active training
 */
void processTrainingIMU(float acc, float pitch, unsigned long now) {
  // Фільтр: ігноруємо всі дані та не детектуємо кроки, якщо палиця майже
  // вертикальна (>= 83 градусів)
  if (pitch >= 83.0f) {
    return;
  }

  float horizAcc = getHorizontalAcc();

  if (rawRecordEnable && !isTestingSession) {
    unsigned long timeFromStart = now - training.startMs;
    logger.logRaw(acc, pitch, horizAcc, timeFromStart);
  }

  stepAccWindow.push(acc);

  switch (stepPhase) {
  case PHASE_IDLE:
    if (acc >= forceThresholdSq) {
      if (t_impact > 0 && t_release > 0) {
        unsigned long groundTimeMs = t_release - t_impact;
        unsigned long cycleTimeMs = now - t_impact;
        commitStep((float)groundTimeMs, (float)cycleTimeMs);
      }
      stepPhase = PHASE_IMPACT;
      localMaxPitch = pitch;
      peakImpactAcc = acc;
      t_impact = now;
      accHorizSumStep = 0;
      accHorizCountStep = 0;
    } else if (t_release > 0) {
      accHorizSumStep += horizAcc;
      accHorizCountStep++;
    }
    break;

  case PHASE_IMPACT:
    if (pitch > localMaxPitch) {
      localMaxPitch = pitch;
      peakImpactAcc = acc;
      t_impact = now;
    }

    if (acc < forceThresholdSq || pitch < localMaxPitch - PITCH_HYSTERESIS) {
      stepPhase = PHASE_GROUND;
      strikeAngle = localMaxPitch;
      localMinPitch = pitch;
      // Start vibration tracking
      t_vibe_start = now;
      vibe_peaks = 0;
      vibe_duration = 0;
      vibe_active = true;
      last_vibe_acc = acc;
      vibe_acc_increasing = false;
    }
    break;

  case PHASE_GROUND:
    if (pitch < localMinPitch) {
      localMinPitch = pitch;
    }

    if (pitch > localMinPitch + PITCH_HYSTERESIS) {
      t_release = now;
      liftAngle = localMinPitch;
      peakLiftAcc = stepAccWindow.getMax();
      stepPhase = PHASE_IDLE;
    } else {
      // Analyze vibrations in ground phase
      if (vibe_active) {
        if (acc > last_vibe_acc) {
          vibe_acc_increasing = true;
        } else if (acc < last_vibe_acc && vibe_acc_increasing) {
          // Peak detected at last_vibe_acc
          if (last_vibe_acc >= peakImpactAcc * 0.05f) {
            vibe_peaks++;
            vibe_duration = (float)(now - t_vibe_start);
          } else {
            vibe_active = false;
          }
          vibe_acc_increasing = false;
        }
        last_vibe_acc = acc;
      }
    }
    break;
  }
}

/**
 * @brief Finalize a detected step and update statistics
 */
void commitStep(float groundMs, float cycleMs) {
  //фікс початку відриву. Трошки раніше настає ніж кут мінімальний.
  groundMs = groundMs - 100;

  if (groundMs < MIN_GROUNDTIME_MS || cycleMs < MIN_CYCLETIME_MS) {
    return;
  }
  if (groundMs > MAX_GROUNDTIME_MS)
    groundMs = MAX_GROUNDTIME_MS;
  if (cycleMs > MAX_CYCLETIME_MS)
    cycleMs = MAX_CYCLETIME_MS;

  float strikeFN = accToKgf(sqrtf(peakImpactAcc));
  float liftFN = accToKgf(sqrtf(peakLiftAcc));
  float freq = 60000.0f / cycleMs;

  if (validateStep(strikeAngle, liftAngle, strikeFN)) {
    training.errors++;
  }

  training.strikeAngleStat.add(strikeAngle);
  training.liftAngleStat.add(liftAngle);
  training.rangeAngleStat.add(strikeAngle-liftAngle);
  training.strikeForce.add(strikeFN);
  training.liftForce.add(liftFN);
  training.groundTime.add(groundMs);
  training.cycleTime.add(cycleMs);
  training.frequency.add(freq);

  float avgAccHoriz =
      (accHorizCountStep > 0) ? (accHorizSumStep / accHorizCountStep) : 0;
  training.avgAccHorizStat.add(avgAccHoriz);
  training.impactDurationStat.add(vibe_duration);
  float vFreq = (vibe_duration > 0 && vibe_peaks > 0)
                    ? ((1000.0f * vibe_peaks) / vibe_duration )
                    : 0;
  training.vibrationFreqStat.add(vFreq);
  training.hasData = true;

  // log data to file
  if (sdAvailable && !isTestingSession) {
    uint32_t elapsed = (millis() - training.startMs) * 0.001;
    uint8_t m = elapsed * 0.0167;
    uint8_t s = elapsed % 60;
    sprintf(buf, "%u,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d,%.1f,%.0f,%.1f,%02d:%02d",
            training.strikeAngleStat.cnt, strikeAngle, liftAngle, strikeFN,
            liftFN, avgAccHoriz, (int)groundMs, (int)cycleMs, freq,
            vibe_duration, vFreq, m, s);
    logger.log(buf);
  }
}

void testAlgorithmFromSD(String path) {
  if (!sdAvailable)
    return;
  SdFile file;
  if (!file.open(path.c_str(), O_READ)) {
    Serial.println("Could not open RAW file");
    return;
  }

  stepPhase = PHASE_IDLE;
  t_impact = 0;
  t_release = 0;
  t_prev_impact = 0;
  training.reset();
  isTestingSession = true;
  vibe_peaks = 0;
  vibe_duration = 0;
  vibe_active = false;

  char line[128];
  file.fgets(line, sizeof(line)); // first line 

  while (file.fgets(line, sizeof(line)) > 0) {
    char *tokens[4];
    int tIdx = 0;
    char *p = line;
    tokens[tIdx++] = p;
    while (*p && tIdx < 4) {
      if (*p == ',' || *p == '\r' || *p == '\n') {
        *p = '\0';
        if (tIdx < 4)
          tokens[tIdx++] = p + 1;
      }
      p++;
    }

    if (tIdx >= 4) {
      float acc = atof(tokens[0]);
      float pitch = atof(tokens[1]);
      unsigned long t = atol(tokens[3]);
      if (t == 0 && acc == 0)
        continue;
      processTrainingIMU(acc, pitch, t);
    }
  }
  file.close();
  isTestingSession = false;
  training.hasData = true;
  appState = STATE_TRAINING_RESULTS;
}

/**
 * @brief Validates if a step has technique errors
 */
bool validateStep(float sa, float la, float sf) {
  // 1) strikeAngle >= liftAngle
  if (sa >= la)
    return true;
  // 2) strikeAngle < 35, or strikeAngle > 75.
  if (sa < ERR_SA_MIN || sa > ERR_SA_MAX)
    return true;
  // 3) liftAngle > 75 градусів.
  if (la > ERR_LA_MAX)
    return true;
  // 4) strikeFN < 1 (actually ERR_SF_MIN)
  if (sf < ERR_SF_MIN)
    return true;

  return false;
}

/**
 * @brief Calculates a technique score (0-100%)
 */
float gradeTrain(float sa, float la, float groundTime, float cycleTime) {
  // 1) Оцінка за кут уколу (макс 49)
  const float sa_center = (ERR_SA_MAX + ERR_SA_MIN) / 2.0f; // Тепер 46.5°
  float sa_diff = fabs(sa - sa_center);
  const float sa_max_diff = (ERR_SA_MAX - ERR_SA_MIN) / 2.0f; // 9.5°
  float grade1 = 49.0f * (1.0f - (sa_diff / sa_max_diff));
  if (grade1 < 0)
    grade1 = 0;

  // 2) Оцінка за різницю sa–la (макс 17)
  float diff = fabs(sa - la);
  float grade2 = 0.0f;

  if (diff >= 5 && diff <= 10) {
    grade2 = 17.0f;
  } else if (diff < 5) {
    grade2 = 17.0f * (diff / 5.0f);
  } else {
    float over = diff - 10.0f;
    if (over < 10.0f) {
      grade2 = 17.0f * (1.0f - (over / 10.0f));
    } else {
      grade2 = 0.0f;
    }
  }

  // 3) Оцінка за робочий цикл (макс 34)
  float cycle = (groundTime / cycleTime) * 100.0f;
  float grade3 = 0.0f;

  if (cycle >= 66.0f) {
    grade3 = 34.0f;
  } else if (cycle >= 50.0f) {
    grade3 = 34.0f * ((cycle - 50.0f) / 16.0f);
  } else {
    grade3 = 0.0f;
  }

  float total = (grade1 + grade2 + grade3);
  return total;
}

void startTraining() {
  // Disable WiFi during training for max IMU throughput + power saving
  playMelody(MELODY_MEASURE_START);
  logger.begin(); // Create CSV + RAW files BEFORE any data arrives
  training.reset();
  stepPhase = PHASE_IDLE;
  appState = STATE_TRAINING_ACTIVE;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
  t_release = 0;
  vibe_peaks = 0;
  vibe_duration = 0;
  vibe_active = false;
}

void stopTraining() {
  playMelody(MELODY_MEASURE_STOP);
  logger.close();
  training.totalTimeS = (millis() - training.startMs) / 1000;
  appState = STATE_TRAINING_RESULTS;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
}
