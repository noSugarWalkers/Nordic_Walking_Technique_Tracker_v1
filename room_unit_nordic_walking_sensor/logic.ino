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
void processTrainingIMU(float acc, float pitch) {
  // Фільтр: ігноруємо всі дані та не детектуємо кроки, якщо палиця майже
  // вертикальна (>= 83 градусів)
  if (pitch >= 83.0f) {
    return;
  }

  unsigned long now = millis();

  switch (stepPhase) {
  case PHASE_IDLE:
    if (acc >= forceThresholdSq && (now - t_impact > IMPACT_DEBOUNCE_MS)) {
      stepPhase = PHASE_IMPACT;
      t_prev_impact = t_impact;
      t_impact = now;
      peakImpactAcc = acc;
      strikeAngle = pitch;
    }
    break;

  case PHASE_IMPACT:
    if (acc > peakImpactAcc) {
      peakImpactAcc = acc;
      strikeAngle = pitch; // angle at peak force
    }

    if (acc < forceThresholdSq * IMPACT_FALL_RATIO) {
      stepPhase = PHASE_PUSH;
    }

    // Handle timeout where the stick is held down forever
    if (now - t_impact > MAX_PUSH_MS) {
      commitStep(MAX_PUSH_MS, MAX_SWING_MS);
      stepPhase = PHASE_IDLE;
      t_release = 0;
    }
    break;

  case PHASE_PUSH: {
    unsigned long groundTimeMs = now - t_impact;

    if (groundTimeMs > MAX_PUSH_MS) {
      commitStep(MAX_PUSH_MS, MAX_SWING_MS);
      stepPhase = PHASE_IDLE;
      t_release = 0;
    } else if (acc > LIFT_ACC_THRESHOLD_SQ || getHorizontalAcc() > LIFT_HORIZ_THRESH) {
      bool isStrongImpact =
          (peakImpactAcc >= (IMPACT_MIN_PEAK_G * IMPACT_MIN_PEAK_G)) &&
          (groundTimeMs >= MIN_IMPACT_MS);

      if (groundTimeMs >= MIN_PUSH_MS || isStrongImpact) {
        stepPhase = PHASE_RELEASE;
        t_release = now;
        
        peakLiftAcc = acc;
        liftAngle = pitch;
        accHorizSumStep = 0;
        accHorizCountStep = 0;
      } else {
        // Push was way too short, noise. Reset to IDLE.
        stepPhase = PHASE_IDLE;
        t_release = 0;
      }
    }
  } break;

  case PHASE_RELEASE: {
    unsigned long swingMs = now - t_release;

    // Limit the search for the lift-off peak to the first 100ms
    if (swingMs < 100) {
      if (acc > peakLiftAcc) {
        peakLiftAcc = acc;
        liftAngle = pitch;
      }
    }

    // Accumulate horizontal acceleration during swing
    accHorizSumStep += getHorizontalAcc();
    accHorizCountStep++;

    if (swingMs > MAX_SWING_MS) {
      // Timeout during swing
      commitStep((float)(t_release - t_impact), (float)(MAX_SWING_MS + (t_release - t_impact)));
      stepPhase = PHASE_IDLE;
      t_release = 0;
    }
    // Next impact detected → complete step
    else if (acc >= forceThresholdSq && (now - t_release > IMPACT_DEBOUNCE_MS)) {
      unsigned long groundTime = t_release - t_impact;
      unsigned long cycleTime = now - t_impact;
      
      if (cycleTime >= MIN_STEP_PERIOD_MS) {
        commitStep((float)groundTime, (float)cycleTime);
      }

      // Start new impact immediately
      stepPhase = PHASE_IMPACT;
      t_prev_impact = t_impact;
      t_impact = now;
      peakImpactAcc = acc;
      strikeAngle = pitch;
    }
  } break;
  }
}

/**
 * @brief Finalize a detected step and update statistics
 */
void commitStep(float groundMs, float cycleMs) {

  float strikeFN = accToKgf(sqrtf(peakImpactAcc));
  float liftFN = accToKgf(sqrtf(peakLiftAcc));
  float freq = 60000.0f / cycleMs;

  if (validateStep(strikeAngle, liftAngle, strikeFN)) {
    training.errors++;
  }

  training.strikeAngleStat.add(strikeAngle);
  training.liftAngleStat.add(liftAngle);
  training.strikeForce.add(strikeFN);
  training.liftForce.add(liftFN);
  training.groundTime.add(groundMs);
  training.cycleTime.add(cycleMs);
  training.frequency.add(freq);

  float avgAccHoriz =
      (accHorizCountStep > 0) ? (accHorizSumStep / accHorizCountStep) : 0;
  training.avgAccHorizStat.add(avgAccHoriz);
  training.hasData = true;

  // log data to file
  if (sdAvailable) {
    uint32_t elapsed = (millis() - training.startMs) * 0.001;
    uint8_t m = elapsed * 0.0167;
    uint8_t s = elapsed % 60;
    sprintf(buf, "%u,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d,%.1f,%02d:%02d",
            training.strikeAngleStat.cnt, strikeAngle, liftAngle, strikeFN,
            liftFN, avgAccHoriz, (int)groundMs, (int)cycleMs, freq, m, s);
    logger.log(buf);
  }
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
  playMelody(MELODY_MEASURE_START);
  //logger.begin();
  training.reset();
  stepPhase = PHASE_IDLE;
  appState = STATE_TRAINING_ACTIVE;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
  t_release = 0;
}

void stopTraining() {
  playMelody(MELODY_MEASURE_STOP);
  logger.close();
  training.totalTimeS = (millis() - training.startMs) / 1000;
  appState = STATE_TRAINING_RESULTS;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
}
