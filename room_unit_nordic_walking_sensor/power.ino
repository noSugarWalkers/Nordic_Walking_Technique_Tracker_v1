#include "config.h"
#include "types.h"

// ============================================================
// Power Globals (externs from main)
// ============================================================
extern XPowersPPM PPM;
extern GaugeBQ27220 gauge;
extern bool gaugeEnable;
extern bool ppmEnable;
extern int batteryVoltage;

unsigned long lastBatUpdate = 0;
int batteryPercent = 0;
int bu = 0;

// ============================================================
// PMIC (BQ25896) Helpers
// ============================================================
void writePMIC(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PMIC_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readPMIC(uint8_t reg) {
  Wire.beginTransmission(PMIC_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(PMIC_ADDR, (uint8_t)1);
  return Wire.read();
}

void setupPPM() {
  /*********Power Management**********/
  bool result = PPM.init(Wire, PIN_SDA, PIN_SCL, BQ25896_SLAVE_ADDRESS);
  if (result == false) {
    Serial.println("PPM is not online...");
    return;
  }

  ppmEnable = true;

  if (!gauge.begin(Wire, PIN_SDA, PIN_SCL)) {
    Serial.println("gauge is not online...");
    return;
  }
  delay(1000);
  if (gauge.refresh()) {
    BatteryStatus batteryStatus = gauge.getBatteryStatus();
    if (!batteryStatus.isBatteryPresent()) {
      PPM.disableCharge();
      Serial.println("UPS!Battery not found");
      return;
    }
  }

  gaugeEnable = true;

  // init
  uint16_t newDesignCapacity = 120;
  uint16_t newFullChargeCapacity = 120;
  gauge.setNewCapacity(newDesignCapacity, newFullChargeCapacity);

  PPM.setSysPowerDownVoltage(3200);
  PPM.setChargeTargetVoltage(4208);
  PPM.setPrechargeCurr(128);
  PPM.setChargerConstantCurr(256);
  PPM.disableCharge();
}

void checkHW() {
  // Enable charging
  if (ppmEnable) {
    if (PPM.getVbusVoltage() > 2800) {
      if (!PPM.isEnableCharge()) {
        PPM.enableCharge();
      } else {
        if (PPM.isEnableCharge())
          PPM.disableCharge();
      }
    }
  }

  // update every 60s
  if (bu <= 0) {
    // Power OFF
    batteryVoltage = getBatteryVoltage();
    if (batteryVoltage < 3150)
      powerOff();

    // WiFi reconect
    if (wifiConnected && WiFi.status() != WL_CONNECTED)
      WiFi.reconnect();

    bu = 3000;
  } else {
    bu--;
  }
}

void checkButtons(){

  // --- Physical Button: STOP/BACK (IO38) ---
  bool stopDown = (digitalRead(PIN_BTN_STOP) == LOW);
  if (stopDown && !btnStopWasPressed) {
    btnStopPressStart = millis();
    btnStopWasPressed = true;
  } else if (!stopDown && btnStopWasPressed) {
    btnStopWasPressed = false;
    //Short press
    if (millis() - btnStopPressStart < 4000) {
      if (appState == STATE_TRAINING_ACTIVE)
        stopTraining();
      else
        startTraining();
    }
  }

   //Long press
  if (stopDown && btnStopWasPressed) {
    if (millis() - btnStopPressStart > 4000) {
      playMelody(MELODY_AP);
      if(wifiConnected){
        setupWifi(APP_WIFI_OFF);
      }else{
        setupWifi(APP_WIFI_STA_AP);
        playMelody(MELODY_READY);
      }
      //Pause for user reaction
      delay(2000);
    }
  }

  // --- Physical Button: POWER (IO0) ---
  bool pwrDown = (digitalRead(PIN_BTN_PWR) == LOW);
  if (pwrDown && !btnPwrWasPressed) {
    btnPwrPressStart = millis();
    btnPwrWasPressed = true;
  } else if (!pwrDown && btnPwrWasPressed) {
    btnPwrWasPressed = false;
  }

  if (pwrDown && btnPwrWasPressed) {
    if (millis() - btnPwrPressStart > 4000) {
      powerOff();
    }
  }
}

float getBatteryVoltage() {
  if (!gaugeEnable)
    return 3600;
  if (gauge.refresh())
    return gauge.getVoltage();
  return 0;
}

int getBatteryPercent() {
  if (!gaugeEnable)
    return 0;
  if (gauge.refresh())
    return gauge.getStateOfCharge();
  return 0;
}

void powerOff() {
  playMelody(MELODY_SHUTDOWN);
  logger.close();
  delay(1500);
  // REG09 Bit 5 = BATFET_DIS (Ship Mode)
  uint8_t val = readPMIC(0x09);
  writePMIC(0x09, val | 0x20);
}
