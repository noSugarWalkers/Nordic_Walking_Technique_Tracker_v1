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
extern bool autoWifiEnable;
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
extern float q_w;

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
float qw_at_release = 0; // quaternion qw at the moment of lift-off
bool isTestingSession = false;
bool isOldRawFormat = false;

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
void processTrainingIMU(float acc, float pitch, unsigned long now, float horizAcc) {
  // Фільтр: ігноруємо всі дані та не детектуємо кроки, якщо палиця майже
  // вертикальна (>= 83 градусів)
  if (pitch >= 83.0f) {
    return;
  }

  // horizAcc is now passed as an argument to support replaying from SD

  if (rawRecordEnable && !isTestingSession) {
    unsigned long timeFromStart = now - training.startMs;
    logger.logRaw(acc, pitch, horizAcc, q_w, timeFromStart);
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
      qw_at_release = q_w; // Capture qw at ground exit
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
  // фікс початку відриву. Трошки раніше настає ніж кут мінімальний.
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
  
  // Calculate rotation as diff between lift-off and impact
  float qw_eval = (isTestingSession && isOldRawFormat) ? 0 : abs(q_w - qw_at_release);
  float rotDeg = asinf(qw_eval) * 2.0f * 180.0f / M_PI;

  validateStep(strikeAngle, liftAngle, strikeFN, groundMs, cycleMs, qw_eval, training.techniqueErrors);

  training.strikeAngleStat.add(strikeAngle);
  training.liftAngleStat.add(liftAngle);
  training.rangeAngleStat.add(strikeAngle - liftAngle);
  training.strikeForce.add(strikeFN);
  training.liftForce.add(liftFN);
  training.groundTime.add(groundMs);
  training.cycleTime.add(cycleMs);
  training.frequency.add(freq);
  training.rotationStat.add(rotDeg);

  float avgAccHoriz =
      (accHorizCountStep > 0) ? (accHorizSumStep / accHorizCountStep) : 0;
  training.avgAccHorizStat.add(avgAccHoriz);
  training.impactDurationStat.add(vibe_duration);
  float vFreq = (vibe_duration > 0 && vibe_peaks > 0)
                    ? ((1000.0f * vibe_peaks) / vibe_duration)
                    : 0;
  training.vibrationFreqStat.add(vFreq);
  training.hasData = true;

  // log data to file
  if (sdAvailable && !isTestingSession) {
    uint32_t elapsed = (millis() - training.startMs) * 0.001;
    uint8_t m = elapsed * 0.0167;
    uint8_t s = elapsed % 60;
    sprintf(buf, "%u,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d,%.1f,%.0f,%.1f,%.1f,%02d:%02d",
            training.strikeAngleStat.cnt, strikeAngle, liftAngle, strikeFN,
            liftFN, avgAccHoriz, (int)groundMs, (int)cycleMs, freq,
            vibe_duration, vFreq, rotDeg, m, s);
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
  
  // Reset quaternion to identity before test
  extern float q_x, q_y, q_z;
  q_w = 1.0f; q_x = 0; q_y = 0; q_z = 0;
  qw_at_release = 1.0f;

  char line[128];
  uint32_t fileSize = file.fileSize();
  testProgress = 0;
  int lineCount = 0;
  unsigned long lastT = 0;

  file.fgets(line, sizeof(line)); // first line

  while (file.fgets(line, sizeof(line)) > 0) {
    lineCount++;
    if (lineCount % 20 == 0) vTaskDelay(1);
    testProgress = (file.curPosition() * 100) / fileSize;
    char *tokens[5];
    int tIdx = 0;
    char *p = line;
    tokens[tIdx++] = p;
    while (*p && tIdx < 5) {
      if (*p == ',') {
        *p = '\0';
        tokens[tIdx++] = p + 1;
      } else if (*p == '\r' || *p == '\n') {
        *p = '\0';
        break; // End of line
      }
      p++;
    }

    if (tIdx >= 4) {
      float acc = atof(tokens[0]);
      float pitch = atof(tokens[1]);
      float horizAcc = atof(tokens[2]);

      // Для старих файлів (4 параметри) встановлюємо q_w = 1.0 (identity),
      // інакше всі розрахунки прискорення будуть нульовими.
      if (tIdx >= 5) {
        q_w = atof(tokens[3]);
        isOldRawFormat = false;
      } else {
        q_w = 1.0f; // Identity for math
        isOldRawFormat = true;
      }

      lastT = (tIdx >= 5) ? atol(tokens[4]) : atol(tokens[3]);
      if (lastT == 0 && acc == 0)
        continue;
      
      processTrainingIMU(acc, pitch, lastT, horizAcc);
    }
  }
  file.close();
  training.totalTimeS = lastT / 1000;
  isTestingSession = false;
  training.hasData = true;
  testProgress = 100;
  appState = STATE_TRAINING_RESULTS;
}

/**
 * @brief Validates if a step has technique errors
 */
void validateStep(float sa, float la, float sf, long gms, long cms, float qw, TrainingErrors &error) {
  long liftTime = cms-gms;

  //lowPositionError - strike angle < 36
  if(sa<=LOW_ANGLE){ 
    error.lowPositionError.add();
    return;
  }
  
  //rotateHipError - strike angle > 75 
  if((sa>MAX_ANGLE)&&(liftTime<gms)) {
    error.rotateHipError.add();
    return;
  }
  
  //elbowError - short ground time and angle > MAX
  if((liftTime>=gms)&&(sa>MAX_ANGLE)) {
    error.elbowError.add();
    return;
  }

  //motionRangeError and normal strike angle
  if((liftTime>=gms)&&(sa<MAX_ANGLE)) {
    error.motionRangeError.add();
    return;
  }

  //parallelOperationError - rotation threshold
  if(qw > 0.2f) {
    error.parallelOperationError.add();
    return;
  }

  //pushError - very low lift angle(рука не відпускає палицю) або волочіння палиць, або не синхрон
  if((la<CRITICAL_ANGLE)||(gms==MAX_GROUNDTIME_MS)||(cms==MAX_CYCLETIME_MS)) {
    error.pushError.add();
    return;
  }
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
  qw_at_release = q_w;
  if (autoWifiEnable) setupWifi(APP_WIFI_OFF);
}

void stopTraining() {
  playMelody(MELODY_MEASURE_STOP);
  logger.close();
  training.totalTimeS = (millis() - training.startMs) / 1000;
  appState = STATE_TRAINING_RESULTS;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
  if (autoWifiEnable){
    setupWifi(APP_WIFI_AP);
    delay(2000);
    playMelody(MELODY_READY);
  }
}
