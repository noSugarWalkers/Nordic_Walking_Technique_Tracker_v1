#include "config.h"
#include "types.h"

// ============================================================
// IMU Globals
// ============================================================
// IMU State (moved to main for visibility)
extern SensorBHI260AP bhi;
extern bool imuReady;
extern float cal_pitch_offset;
extern float gFactor;
extern float forceMultiplier;
extern uint16_t poleWeightGrams;
extern AppState appState;
extern bool autoTrainingEnable;

float q_w = 1, q_x = 0, q_y = 0, q_z = 0; // Rotation vector quaternion
float la_x = 0, la_y = 0, la_z = 0;       // Linear acceleration (g)
float peakForceAccumulator = 0;           // Max force since last diag check

void setupBHI() {
  // BHI Enable
  pinMode(PIN_BHI_EN, OUTPUT);
  digitalWrite(PIN_BHI_EN, HIGH);

  // BHI260AP initialization
  bhi.setPins(PIN_BHI_RST);
  bhi.setFirmware(bosch_app30_shuttle_bhi260_firmware_image,
                  sizeof(bosch_app30_shuttle_bhi260_firmware_image));

  if (!bhi.begin(Wire, BHI260AP_SLAVE_ADDRESS_L, PIN_SDA, PIN_SCL)) {
    Serial.println("IMU ERROR!");
  } else {
    // Only GameRV (no magnetometer needed) + Linear Acceleration
    // RV removed — was duplicating quaternion data, wasting 33% of I2C
    // bandwidth
    bhi.configure(BHY2_SENSOR_ID_GAMERV, sensorFreq, 0);
    bhi.configure(BHY2_SENSOR_ID_LACC, sensorFreq, 0);

    bhi.onResultEvent(BHY2_SENSOR_ID_GAMERV, onRotationVector);
    bhi.onResultEvent(BHY2_SENSOR_ID_LACC, onLinearAcc);
    imuReady = true;
  }

  // PMIC configuration
  uint8_t reg02 = readPMIC(0x02);
  writePMIC(0x02, reg02 | 0x40);
}

/**
 * @brief Robust Pitch calculation from quaternions
 * @return float Pitch in degrees
 */
float getEulerPitch() {
  float sinp = 2.0f * (q_w * q_y - q_z * q_x);
  if (fabs(sinp) >= 1)
    return copysign(M_PI / 2, sinp) * 180.0f / M_PI;
  return asinf(sinp) * 180.0f / M_PI;
}

/**
 * @brief Robust Roll calculation from quaternions
 * @return float Roll in degrees
 */
float getEulerRoll() {
  float sinr_cosp = 2.0f * (q_w * q_x + q_y * q_z);
  float cosr_cosp = 1.0f - 2.0f * (q_x * q_x + q_y * q_y);
  return atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
}

float getPitch() { return getEulerPitch() - cal_pitch_offset; }

float getRoll() { return getEulerRoll(); }

float getLinAccMagSq() {
  float magSq = (la_x * la_x + la_y * la_y + la_z * la_z);
  if (magSq > peakForceAccumulator)
    peakForceAccumulator = magSq;
  return magSq;
}

float getAndResetPeakForce() {
  float p = sqrtf(peakForceAccumulator);
  peakForceAccumulator = 0;
  return p;
}

/**
 * @brief Calculates horizontal linear acceleration magnitude in g-units
 * Rotates the sensor-frame linear acceleration into the world frame using
 * current orientation
 */
float getHorizontalAcc() {
  float qw2 = q_w * q_w;
  float qx2 = q_x * q_x;
  float qy2 = q_y * q_y;
  float qz2 = q_z * q_z;

  // Use quaternion rotation formula: v_world = q * v_local * q_conj
  // x_world
  float xw = la_x * (qw2 + qx2 - qy2 - qz2) +
             2.0f * la_y * (q_x * q_y - q_w * q_z) +
             2.0f * la_z * (q_x * q_z + q_w * q_y);
  // y_world
  float yw = 2.0f * la_x * (q_x * q_y + q_w * q_z) +
             la_y * (qw2 - qx2 + qy2 - qz2) +
             2.0f * la_z * (q_y * q_z - q_w * q_x);
  // z_world (vertical)
  float zw = la_x * (2.0f * q_x * q_z - 2.0f * q_w * q_y) +
             la_y * (2.0f * q_y * q_z + 2.0f * q_w * q_x) +
             la_z * (qw2 - qx2 - qy2 + qz2);

  float hAcc = sqrtf(xw * xw + yw * yw);

  return hAcc;
}

/**
 * @brief Convert linear acceleration (g) to Newtons/Kgf using pole mass
 */
float accToKgf(float acc_g) {
  // Use gFactor if needed, but here we expect raw g-units
  // forceMultiplier is a user-defined coefficient for sensitivity correction
  return acc_g * 9.81f * (poleWeightGrams / 1000.0f) * 0.10197 *
         forceMultiplier;
}

// ============================================================
// IMU Callbacks
// ============================================================

void onRotationVector(uint8_t sensor_id, const uint8_t *data, uint32_t size,
                      uint64_t *timestamp, void *user_data) {
  if (size >= 10) {
    int16_t raw_x = (int16_t)(data[0] | (data[1] << 8));
    int16_t raw_y = (int16_t)(data[2] | (data[3] << 8));
    int16_t raw_z = (int16_t)(data[4] | (data[5] << 8));
    int16_t raw_w = (int16_t)(data[6] | (data[7] << 8));
    q_x = raw_x / 16384.0f;
    q_y = raw_y / 16384.0f;
    q_z = raw_z / 16384.0f;
    q_w = raw_w / 16384.0f;
  }
}

void onLinearAcc(uint8_t sensor_id, const uint8_t *data, uint32_t size,
                 uint64_t *timestamp, void *user_data) {
  if (size >= 6) {
    int16_t raw_x = (int16_t)(data[0] | (data[1] << 8));
    int16_t raw_y = (int16_t)(data[2] | (data[3] << 8));
    int16_t raw_z = (int16_t)(data[4] | (data[5] << 8));
    // BHI260AP linear acc scale: 1/2048 g per LSB at 16g range (approx 4096
    // LSB/g)
    la_x = raw_x / 4096.0f;
    la_y = raw_y / 4096.0f;
    la_z = raw_z / 4096.0f;

    if (imuReady) {

      float accSq = getLinAccMagSq();
      float acc = accSq * gFactor;
      float pitch = getPitch();

      if (isSettingsActive()) {
        processDiagnosticsIMU();
      } else {
        if (autoTrainingEnable) {
          scanAutoGesturesIMU(acc, pitch);
        }
        if (appState == STATE_TRAINING_ACTIVE) {
          processTrainingIMU(acc, pitch, millis(), getHorizontalAcc());
        }
      }
    }
  }
}
