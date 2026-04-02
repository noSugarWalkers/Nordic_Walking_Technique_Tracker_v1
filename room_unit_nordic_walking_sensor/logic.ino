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

/**
 * @brief Main IMU processing loop for step detection
 */
void processIMU() {

  float acc = getLinAccMagSq() * gFactor;
  float pitch = getPitch();

  unsigned long now = millis();

  switch (stepPhase) {
  case PHASE_IDLE:
    if (acc >= forceThresholdSq && (now - t_impact > IMPACT_DEBOUNCE_MS)) {
      stepPhase = PHASE_IMPACT;
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
    // Detect end of impact: acc drops well below threshold
    if (acc < forceThresholdSq * IMPACT_FALL_RATIO) {
      unsigned long pushMs = now - t_impact;
      
      // Check if this was a valid "push" or at least a strong "impact spike"
      bool isStrongImpact = (peakImpactAcc >= (IMPACT_MIN_PEAK_G * IMPACT_MIN_PEAK_G)) && (pushMs >= MIN_IMPACT_MS);
      
      if ((pushMs >= MIN_PUSH_MS || isStrongImpact) && pushMs <= MAX_PUSH_MS) {
        // ... (Gesture Detection logic stays original)
        // Gesture Detection
        if (autoTrainingEnable && !isSettingsActive()) {
          if (strikeAngle > 80.0f) {
            if (appState != STATE_TRAINING_ACTIVE) {
              if (autoStartHitsCount == 0 ||
                  (now - autoGestureStartMs > 2000)) {
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
          } else if (strikeAngle <= 75.0f) {
            autoStartHitsCount = 0;
            autoStopHitsCount = 0;
          }
        }

        stepPhase = PHASE_RELEASE;
        t_release = now;
        peakLiftAcc = 0;
        liftAngle = pitch;
        accHorizSumStep = 0;
        accHorizCountStep = 0;
      } else {
        // Too short or too long — glitch, ignore
        stepPhase = PHASE_IDLE;
      }
    }
    break;

  case PHASE_RELEASE:
    if (acc > peakLiftAcc) {
      peakLiftAcc = acc;
      liftAngle = pitch;
    }

    // Accumulate horizontal acceleration during swing
    accHorizSumStep += getHorizontalAcc();
    accHorizCountStep++;

    // Next impact detected → complete step
    if (acc >= forceThresholdSq && (now - t_impact > IMPACT_DEBOUNCE_MS)) {
      unsigned long swingMs = now - t_release;
      unsigned long pushMs = t_release - t_impact;
      if (swingMs >= MIN_SWING_MS && swingMs <= MAX_SWING_MS &&
          (pushMs + swingMs) >= MIN_STEP_PERIOD_MS) {
        commitStep((float)pushMs, (float)swingMs);
      }
      // Start new impact
      stepPhase = PHASE_IMPACT;
      t_prev_impact = t_impact;
      t_impact = now;
      peakImpactAcc = acc;
      strikeAngle = pitch;
    }
    break;
  }
}

/**
 * @brief Finalize a detected step and update statistics
 */
void commitStep(float pushMs, float swingMs) {
  // Filter: only count "hits" with angle <= 75 degrees
  if (strikeAngle > 79.0f)
    return;

  float strikeFN = accToKgf(sqrtf(peakImpactAcc));
  float liftFN = accToKgf(sqrtf(peakLiftAcc));
  float freq = 60000.0f / (pushMs + swingMs);

  if (validateStep(strikeAngle, liftAngle, strikeFN)) {
    training.errors++;
  }

  training.strikeAngleStat.add(strikeAngle);
  training.liftAngleStat.add(liftAngle);
  training.strikeForce.add(strikeFN);
  training.liftForce.add(liftFN);
  training.pushTime.add(pushMs);
  training.swingTime.add(swingMs);
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
            liftFN, avgAccHoriz, (int)pushMs, (int)swingMs, freq, m, s);
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
float gradeTrain(float sa, float la, float p, float s) {
    // 1) Оцінка за кут уколу (макс 49)
    const float sa_center = (ERR_SA_MAX + ERR_SA_MIN) / 2.0f;   // Тепер 46.5°
    float sa_diff = fabs(sa - sa_center);
    const float sa_max_diff = (ERR_SA_MAX - ERR_SA_MIN) / 2.0f; // 9.5°
    float grade1 = 49.0f * (1.0f - (sa_diff / sa_max_diff));
    if (grade1 < 0) grade1 = 0;

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
    float cycle = (p / s) * 100.0f;
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
  logger.begin();
  training.reset();
  stepPhase = PHASE_IDLE;
  appState = STATE_TRAINING_ACTIVE;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
}

void stopTraining() {
  playMelody(MELODY_MEASURE_STOP);
  logger.close();
  training.totalTimeS = (millis() - training.startMs) / 1000;
  appState = STATE_TRAINING_RESULTS;
  autoStartHitsCount = 0;
  autoStopHitsCount = 0;
}
