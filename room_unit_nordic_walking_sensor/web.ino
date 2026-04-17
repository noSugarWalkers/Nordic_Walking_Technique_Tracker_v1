#include "config.h"
#include "types.h"

// ============================================================
// WiFi & Web Globals (externs from main)
// ============================================================
extern bool wifiConnected;
extern String staIP;
extern WebServer server;
extern DNSServer dnsServer;
extern bool dnsStarted;
extern unsigned long lastSettingsActivityMs;
extern AppState appState;
extern WiFiMode wifiMode;
extern TrainingData training;

// Calibration Force state (externs from main)
extern bool isCalibratingForce;
extern unsigned long calForceStartMs;
extern float calForceBuffer[300];
extern int calForceIndex;
extern float calForceResultG;

// Hardware & Config (externs from main)
extern GaugeBQ27220 gauge;
extern XPowersPPM PPM;
extern bool gaugeEnable;
extern bool sdAvailable;
extern SdFat sd;

extern float forceThreshold;
extern float forceThresholdSq;
extern float cal_pitch_offset;
extern bool sdRecordEnable;
extern bool rawRecordEnable;
extern uint8_t poleLength;
extern uint16_t poleWeightGrams;
extern uint8_t userHeight;
extern uint16_t sensorFreq;
extern bool buzzerEnable;
extern bool autoTrainingEnable;
extern float gFactor;
extern float forceMultiplier;
extern float accToKgf(float acc_g);
void validateStep(float sa, float la, float sf, long gms, long cms, TrainingErrors& error);

/**
 * @brief Initialize WiFi (Station or AP)
 */
void setupWifi(WiFiMode wifi) {
  WiFi.disconnect(true);
  delay(100);

  if (wifi == APP_WIFI_STA || wifi == APP_WIFI_STA_AP) {
    WiFi.mode(WIFI_STA);
    IPAddress ip(CONFIG_IP);
    IPAddress gw(CONFIG_GATEWAY);
    IPAddress sn(CONFIG_SUBNET);
    WiFi.config(ip, gw, sn);
    WiFi.persistent(false);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting STA");
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) {
      delay(300);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      staIP = WiFi.localIP().toString();
      Serial.println("\nSTA IP: " + staIP);

      WiFi.setSleep(true);
      esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
      WiFi.setTxPower(WIFI_POWER_8_5dBm);
      return;
    } else {
      if (wifi == APP_WIFI_STA_AP) {
        wifiConnected = true;
        WiFi.mode(WIFI_AP);
        IPAddress apIP(WIFI_AP_IP);
        IPAddress apGW(WIFI_AP_GATEWAY);
        IPAddress apSN(WIFI_AP_SUBNET);
        WiFi.softAPConfig(apIP, apGW, apSN);
        WiFi.softAP(WIFI_AP_SSID); // open, no password
        Serial.println("AP started: " + String(WIFI_AP_SSID));
        dnsServer.start(53, "*", apIP);
        dnsStarted = true;
        // AP mode: reduce TX power
        WiFi.setTxPower(WIFI_POWER_8_5dBm);
        return;
      } else {
        wifiConnected = false;
        Serial.println("\nSTA connect failed");
        WiFi.mode(WIFI_OFF);
        Serial.println("WiFi OFF");
        return;
      }
    }
  } else if (wifi == APP_WIFI_AP) {
    wifiConnected = true;
    WiFi.mode(WIFI_AP);
    IPAddress apIP(WIFI_AP_IP);
    IPAddress apGW(WIFI_AP_GATEWAY);
    IPAddress apSN(WIFI_AP_SUBNET);
    WiFi.softAPConfig(apIP, apGW, apSN);
    WiFi.softAP(WIFI_AP_SSID); // open, no password
    Serial.println("AP started: " + String(WIFI_AP_SSID));
    dnsServer.start(53, "*", apIP);
    dnsStarted = true;
    // AP mode: reduce TX power
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    return;
  } else if (wifi == APP_WIFI_OFF) {
    wifiConnected = false;
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi OFF");
  }
  return;
}

/**
 * @brief Configure Web Server routes
 */
void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/results", HTTP_GET, handleResults);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/load", HTTP_GET, handleLoad);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/api/settings", HTTP_GET, handleSettingsApi);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/cal_force_start", HTTP_POST, handleCalibrateForceStart);
  server.on("/api/cal_force_status", HTTP_GET, handleCalibrateForceStatus);
  server.on("/api/diag", HTTP_GET, handleDiag);
  server.on("/delete", HTTP_GET, handleDelete);
  server.on("/rename", handleRename);
  server.on("/test", HTTP_GET, handleTest);
  server.on("/api/test_status", HTTP_GET, []() {
    server.send(200, "application/json", "{\"progress\":" + String(testProgress) + "}");
  });

  server.on("/poweroff", HTTP_GET, []() {
    server.send(200, "text/html",
                "<h2 "
                "style='color:#fff;text-align:center;margin-top:40vh;font-"
                "family:sans-serif'>Power Off...</h2>");
    delay(500);
    powerOff();
  });

  server.on("/start", HTTP_GET, []() {
    startTraining();
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.on("/stop", HTTP_GET, []() {
    stopTraining();
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.println("Web server started");
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", buildResultsHTML());
}

void handleResults() {
  server.send(200, "application/json", buildResultsJSON());
}

String buildResultsJSON() {
  if (!training.hasData)
    return "{\"hasData\":false}";

  String j = "{\"hasData\":true,";
  j += "\"strikeAngle\":{\"avg\":" + String(training.strikeAngleStat.avg(), 1) +
       ",\"min\":" + String(training.strikeAngleStat.minV, 1) +
       ",\"max\":" + String(training.strikeAngleStat.maxV, 1) + "},";
  j += "\"liftAngle\":{\"avg\":" + String(training.liftAngleStat.avg(), 1) +
       ",\"min\":" + String(training.liftAngleStat.minV, 1) +
       ",\"max\":" + String(training.liftAngleStat.maxV, 1) + "},";
  j += "\"RangeAngle\":{\"avg\":" + String(training.rangeAngleStat.avg(), 1) +
       ",\"min\":" + String(training.rangeAngleStat.minV, 1) +
       ",\"max\":" + String(training.rangeAngleStat.maxV, 1) + "},";
  j += "\"strikeForce\":{\"avg\":" + String(training.strikeForce.avg(), 2) +
       ",\"min\":" + String(training.strikeForce.minV, 2) +
       ",\"max\":" + String(training.strikeForce.maxV, 2) + "},";
  j += "\"liftForce\":{\"avg\":" + String(training.liftForce.avg(), 2) +
       ",\"min\":" + String(training.liftForce.minV, 2) +
       ",\"max\":" + String(training.liftForce.maxV, 2) + "},";
  j += "\"groundTime\":{\"avg\":" + String(training.groundTime.avg(), 0) +
       ",\"min\":" + String(training.groundTime.minV, 0) +
       ",\"max\":" + String(training.groundTime.maxV, 0) + "},";
  j += "\"cycleTime\":{\"avg\":" + String(training.cycleTime.avg(), 0) +
       ",\"min\":" + String(training.cycleTime.minV, 0) +
       ",\"max\":" + String(training.cycleTime.maxV, 0) + "},";
  j += "\"frequency\":{\"avg\":" + String(training.frequency.avg(), 1) +
       ",\"min\":" + String(training.frequency.minV, 1) +
       ",\"max\":" + String(training.frequency.maxV, 1) + "},";
  j += "\"avgAccHoriz\":{\"avg\":" + String(training.avgAccHorizStat.avg(), 2) +
       ",\"min\":" + String(training.avgAccHorizStat.minV, 2) +
       ",\"max\":" + String(training.avgAccHorizStat.maxV, 2) + "},";
  j += "\"impactDuration\":{\"avg\":" + String(training.impactDurationStat.avg(), 0) +
       ",\"min\":" + String(training.impactDurationStat.minV, 0) +
       ",\"max\":" + String(training.impactDurationStat.maxV, 0) + "},";
  j += "\"vibrationFreq\":{\"avg\":" + String(training.vibrationFreqStat.avg(), 1) +
       ",\"min\":" + String(training.vibrationFreqStat.minV, 1) +
       ",\"max\":" + String(training.vibrationFreqStat.maxV, 1) + "},";
  j += "\"duration\":" + String(training.totalTimeS) + ",";
  j += "\"battery\":{\"percent\":" + String(getBatteryPercent()) +
       ",\"voltage\":" + String(getBatteryVoltage() / 1000.0, 2) + "},";
  j += "\"state\":" + String(appState) + ",";
  j += "\"steps\":" + String(training.strikeAngleStat.cnt) + ",";

  int totalErrs = training.techniqueErrors.total();
  float purity = 100.0f;
  if (training.strikeAngleStat.cnt > 0) {
    purity = 100.0f -
             (((float)totalErrs / (float)training.strikeAngleStat.cnt) *
              100.0f);
  }
  j += "\"errors\":" + String(totalErrs) + ",";
  j += "\"techniqueErrors\":{";
  j += "\"lowPosition\":" + String(training.techniqueErrors.lowPositionError.count) + ",";
  j += "\"rotateHip\":" + String(training.techniqueErrors.rotateHipError.count) + ",";
  j += "\"elbow\":" + String(training.techniqueErrors.elbowError.count) + ",";
  j += "\"motionRange\":" + String(training.techniqueErrors.motionRangeError.count) + ",";
  j += "\"parallelOperation\":" + String(training.techniqueErrors.parallelOperationError.count) + ",";
  j += "\"push\":" + String(training.techniqueErrors.pushError.count);
  j += "},";
  j += "\"testProgress\":" + String(testProgress) + ",";
  j += "\"techniquePurity\":" + String(purity, 1) + "}";
  return j;
}

void handleDownload() {
  if (!server.hasArg("f")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String path = server.arg("f");
  if (!path.startsWith("/"))
    path = "/" + path;

  SdFile file;
  if (sdAvailable && file.open(path.c_str(), O_READ)) {
    server.sendHeader("Content-Disposition",
                      "attachment; filename=" +
                          path.substring(path.lastIndexOf('/') + 1));
    server.setContentLength(file.fileSize());
    server.send(200, "text/csv", "");

    uint8_t bt[256];
    while (file.available()) {
      size_t n = file.read(bt, sizeof(bt));
      server.client().write(bt, n);
    }
    file.close();
  } else {
    server.send(404, "text/plain", "File not found");
  }
}

void handleLoad() {
  if (!server.hasArg("f")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String path = server.arg("f");
  if (!path.startsWith("/"))
    path = "/" + path;

  SdFile file;
  if (!sdAvailable || !file.open(path.c_str(), O_READ)) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  training.reset();
  currentFileName = path;

  char line[128];
  bool first = true;
  while (file.fgets(line, sizeof(line)) > 0) {
    if (first) {
      first = false;
      continue;
    }

    char *p = line;
    char *tokens[14];
    int tIdx = 0;
    tokens[tIdx++] = p;
    while (*p && tIdx < 14) {
      if (*p == ',' || *p == '\r' || *p == '\n') {
        *p = '\0';
        if (tIdx < 14)
          tokens[tIdx++] = p + 1;
      }
      p++;
    }

    if (tIdx >= 10) {
      float sa = atof(tokens[1]);
      float la = atof(tokens[2]);
      float sf = atof(tokens[3]);
      long gt = atof(tokens[6]);
      long ct = atof(tokens[7]);

      float rotDeg = 0;
      float qw = 0;
      if (tIdx >= 13) {
        rotDeg = atof(tokens[11]);
        // Reconstruct qw for validation threshold (approximate)
        qw = sinf(rotDeg * M_PI / 360.0f);
        training.rotationStat.add(rotDeg);
      }

      validateStep(sa, la, sf, gt, ct, qw, training.techniqueErrors);

      training.strikeAngleStat.add(sa);
      training.liftAngleStat.add(la);
      training.rangeAngleStat.add(sa - la);
      training.strikeForce.add(sf);
      training.liftForce.add(atof(tokens[4]));
      training.avgAccHorizStat.add(atof(tokens[5]));
      training.groundTime.add(atof(tokens[6]));
      training.cycleTime.add(atof(tokens[7]));
      training.frequency.add(atof(tokens[8]));

      if (tIdx >= 13) { // New format with Rotation
        training.impactDurationStat.add(atof(tokens[9]));
        training.vibrationFreqStat.add(atof(tokens[10]));
        char *timeStr = tokens[12];
        char *colon = strchr(timeStr, ':');
        if (colon) {
          *colon = '\0';
          training.totalTimeS = atoi(timeStr) * 60 + atoi(colon + 1);
        }
      } else if (tIdx == 12) { // Old format with Vibration but no Rotation
        training.impactDurationStat.add(atof(tokens[9]));
        training.vibrationFreqStat.add(atof(tokens[10]));
        char *timeStr = tokens[11];
        char *colon = strchr(timeStr, ':');
        if (colon) {
          *colon = '\0';
          training.totalTimeS = atoi(timeStr) * 60 + atoi(colon + 1);
        }
      } else { // Very old format
        char *timeStr = tokens[9];
        char *colon = strchr(timeStr, ':');
        if (colon) {
          *colon = '\0';
          training.totalTimeS = atoi(timeStr) * 60 + atoi(colon + 1);
        }
      }
    }
  }
  file.close();

  training.hasData = true;
  appState = STATE_TRAINING_RESULTS;

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleTest() {
  if (!server.hasArg("f")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String path = server.arg("f");
  if (!path.startsWith("/"))
    path = "/" + path;

  testPath = path;
  currentFileName = path;
  testProgress = 0;
  xTaskCreate(testTask, "testTask", 8192, NULL, 1, NULL);

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleDelete() {
  if (!server.hasArg("f")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String path = server.arg("f");
  if (!path.startsWith("/"))
    path = "/" + path;

  if (sdAvailable && sd.remove(path.c_str())) {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  } else {
    server.send(500, "text/plain", "Delete failed");
  }
}

void handleRename() {
  if (!server.hasArg("f") || !server.hasArg("n")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String oldPath = server.arg("f");
  if (!oldPath.startsWith("/"))
    oldPath = "/" + oldPath;

  String newName = server.arg("n");
  String checkName = newName;
  checkName.toLowerCase();
  if (!checkName.endsWith(".csv")) {
    newName += ".csv";
  }

  String newPath = "/";
  if (oldPath.lastIndexOf('/') > 0) {
    newPath = oldPath.substring(0, oldPath.lastIndexOf('/') + 1);
  }
  newPath += newName;

  if (sdAvailable && sd.rename(oldPath.c_str(), newPath.c_str())) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "text/plain", "Rename failed");
  }
}

String buildResultsHTML() {
  //reset timer
  timerSleep = millis();

  String h = F(
      "<!DOCTYPE html><html lang='uk'><head>"
      "<meta charset='UTF-8'><meta name='viewport' "
      "content='width=device-width,initial-scale=1'>"
      "<title>NW Technique Tracker</title><style>"
      "*{margin:0;padding:0;box-sizing:border-box}"
      "body{font-family:-apple-system,system-ui,sans-serif;background:#0f172a;"
      "color:#f8fafc;padding:16px;line-height:1.5;min-height:100vh}"
      ".card{background:rgba(30,41,59,0.7);backdrop-filter:blur(12px);border-"
      "radius:16px;padding:20px;margin-bottom:20px;border:1px solid "
      "rgba(255,255,255,0.1);box-shadow:0 4px 6px -1px rgba(0,0,0,0.1)}"
      "h1{text-align:center;font-size:22px;font-weight:700;letter-spacing:0."
      "5px;margin-bottom:8px;color:#38bdf8}"
      ".header-bar{display:flex;justify-content:space-between;align-items:"
      "center;margin-bottom:20px}"
      ".bat-info{font-size:13px;background:rgba(56,189,248,0.1);padding:8px "
      "12px;border-radius:20px;cursor:pointer;border:1px solid "
      "rgba(56,189,248,0.2);color:#38bdf8;font-weight:600}"
      ".row{display:flex;justify-content:space-between;align-items:center;"
      "padding:10px 0;border-bottom:1px solid rgba(255,255,255,0.05)}"
      ".row:last-child{border-bottom:none}"
      ".label{font-size:12px;color:#94a3b8;text-transform:uppercase;letter-"
      "spacing:0.05em}"
      ".val{font-size:18px;font-weight:700;color:#38bdf8}"
      ".minmax{font-size:11px;color:#64748b;text-align:right}"
      ".no-data{text-align:center;padding:40px;color:#64748b;font-size:14px}"
      ".actions{display:flex;gap:12px;margin-bottom:20px}"
      ".action-btn{flex:1;text-align:center;padding:14px;background:linear-"
      "gradient(135deg,#10b981,#059669);border-radius:12px;text-decoration:"
      "none;color:#0f172a;font-weight:700;font-size:16px;border:none;cursor:"
      "pointer;transition:transform 0.2s}"
      ".action-btn:active{transform:scale(0.98)}"
      ".btn-stop{background:linear-gradient(135deg,#ef4444,#dc2626);color:#fff}"
      ".file-link{display:block;padding:12px;margin-bottom:10px;background:"
      "rgba(30,41,59,0.5);border-radius:10px;text-decoration:none;color:#"
      "38bdf8;font-family:monospace;border:1px solid "
      "rgba(255,255,255,0.05);transition:background 0.2s}"
      ".file-link:hover{background:rgba(30,41,59,0.8)}"
      "dialog::backdrop{background:rgba(0,0,0,0.8);backdrop-filter:blur(4px)}"
      ".err-list{font-size:11px;color:#94a3b8;margin-top:4px;display:flex;flex-wrap:wrap;gap:8px}"
      ".err-item{text-decoration:underline dotted;cursor:pointer;transition:color 0.2s}"
      ".err-item:hover{color:#38bdf8}"
      ".progress-container{width:100%;background:rgba(255,255,255,0.1);border-radius:10px;margin:20px 0;height:20px;overflow:hidden}"
      ".progress-bar{width:0%;height:100%;background:linear-gradient(90deg,#38bdf8,#818cf8);transition:width 0.3s}"
      "</style></head><body>");

  String shortName = "";
  if (currentFileName != "") {
    shortName = currentFileName.substring(currentFileName.lastIndexOf('/') + 1);
  }
  h += "<div class='header-bar'><h1>🥾 "+ String(DEVICE_NAME) + " v."+ String(FW_VERSION) + (shortName != "" ? " (" + shortName + ")" : "") + "</h1>";
  h += "<div style='display:flex;gap:8px;align-items:center'>";
  if (gaugeEnable) {
    h += "<button class='bat-info' "
         "onclick='document.getElementById(\"batModal\").showModal()'>🔋 " +
         String(getBatteryPercent()) + "% (" + String(getBatteryVoltage()) +
         "mV)</button>";
  }
  h += "<a href='/settings' "
       "style='font-size:22px;text-decoration:none;background:rgba(56,189,248,"
       "0.1);width:40px;height:40px;display:flex;align-items:center;justify-"
       "content:center;border-radius:50%;border:1px solid "
       "rgba(56,189,248,0.2);color:#38bdf8'>⚙️</a>";
  h += "<a href='#' onclick=\"if(confirm('Power "
       "OFF?'))location.href='/poweroff';return false;\" "
       "style='font-size:18px;text-decoration:none;background:rgba(239,68,68,0."
       "1);width:40px;height:40px;display:flex;align-items:center;justify-"
       "content:center;border-radius:50%;border:1px solid "
       "rgba(239,68,68,0.2);color:#ef4444'>⏻</a></div></div>";

  h += "<div class='actions'>";
  if (appState == STATE_TRAINING_ACTIVE) {
    h += "<a href='/stop' class='action-btn btn-stop'>⏹ Stop Training</a>";
    h += "<a href='/' class='action-btn' "
         "style='background:linear-gradient(135deg,#6366f1,#4f46e5);color:#fff'"
         ">↻ Refresh</a>";
  } else {
    h += "<a href='/start' class='action-btn'>▶ Start Training</a>";
  }
  h += "</div>";

  if (!training.hasData) {
    h += "<div class='card'><div class='no-data'>Немає даних "
         "тренування.<br>Запустіть тренування.</div></div>";
  } else {
    float sa = training.strikeAngleStat.avg();
    float la = training.liftAngleStat.avg();
    float sFn = training.strikeForce.avg();
    float lFn = training.liftForce.avg();
    float safeSa = constrain(sa, 10.0f, 89.0f);
    float safeLa = constrain(la, 10.0f, 89.0f);
    int frontTouchX = 190 - (int)(120.0f / tan(safeSa * 3.14159f / 180.0f));
    int backTouchX = 110 - (int)(120.0f / tan(safeLa * 3.14159f / 180.0f));
    if(backTouchX < 0 ) backTouchX = 0;

    h += "<svg viewBox='0 0 300 280' width='100%' "
         "style='background:rgba(0,0,0,0.2); border-radius:16px; "
         "margin-bottom:12px; border:1px solid rgba(255,255,255,0.05); shadow: "
         "0 4px 6px rgba(0,0,0,0.1);'>";
    h += "<clipPath id='gClip'><rect x='0' y='0' width='300' "
         "height='250'/></clipPath>";
    h += "<line x1='0' y1='250' x2='300' y2='250' stroke='#fff' "
         "stroke-opacity='0.2' stroke-width='2' />";
    h += "<g fill='none' stroke='#fff' stroke-width='6' stroke-linecap='round' "
         "stroke-linejoin='round'>";
    h += "<circle cx='150' cy='50' r='14' fill='#fff' /><path d='M150,64 "
         "C160,100 145,140 140,150' />";

    h += "<path d='M140,150 L170,200 L180,250' stroke='#888' /><path d='M180,250 L195,245' stroke='#888' stroke-width='4' />";
    h += "<path d='M140,150 L115,190 L75,245' /><path d='M75,245 L90,250' stroke-width='4' /></g>";

    h += "<g fill='none' stroke-linecap='round' stroke-linejoin='round'>";
    h += "<path d='M150,80 L120,110 L110,140' stroke='#64748b' "
         "stroke-width='5' />";
    h += "<path d='M150,80 L165,105 L190,130' stroke='#f8fafc' "
         "stroke-width='5' /></g>";
    h += "<g clip-path='url(#gClip)'>";
    h += "<line x1='190' y1='130' x2='" + String(frontTouchX) +
         "' y2='250' stroke='#38bdf8' stroke-width='4' stroke-linecap='round' "
         "/>";
    h += "<line x1='110' y1='140' x2='" + String(backTouchX) +
         "' y2='250' stroke='#f43f5e' stroke-width='4' stroke-linecap='round' "
         "/></g>";
    h += "<circle cx='190' cy='130' r='5' fill='#38bdf8' /><circle cx='110' "
         "cy='140' r='5' fill='#f43f5e' />";
    h += "<text x='230' y='115' fill='#38bdf8' font-size='16' "
         "font-weight='bold' text-anchor='middle'>" +
         String(sFn, 1) + "kgf</text>";
    h += "<text x='70' y='115' fill='#f43f5e' font-size='16' "
         "font-weight='bold' text-anchor='middle'>" +
         String(lFn, 1) + "kgf</text>";
    h += "<text x='" + String(frontTouchX + 10) +
         "' y='240' fill='#38bdf8' font-size='14' font-weight='bold' "
         "text-anchor='start'>" +
         String(sa, 1) + "&deg;</text>";
    h += "<text x='" + String(backTouchX + 25) +
         "' y='240' fill='#f43f5e' font-size='14' font-weight='bold' "
         "text-anchor='start'>" +
         String(la, 1) + "&deg;</text></svg>";
    
    h += "<div class='card'><div class='row'><div><div "
         "class='label'>Кроків</div><div class='val'>" +
         String(training.strikeAngleStat.cnt) + "</div></div>";
    h += "<div><div class='label'>Час</div><div class='val'>" +
         String(training.totalTimeS / 60) + ":" +
         (training.totalTimeS % 60 < 10 ? "0" : "") +
         String(training.totalTimeS % 60) + "</div></div></div>";

    auto row = [&](const String &lbl, const String &unit, const Stat &s,
                   int dec) {
      h += "<div class='row'><div><div class='label'>" + lbl +
           "</div><div class='val'>" + String(s.avg(), dec) + " " + unit +
           "</div></div>"
           "<div class='minmax'>min " +
           String(s.minV, dec) + "<br>max " + String(s.maxV, dec) +
           "</div></div>";
    };

    row("Кут удару", "°", training.strikeAngleStat, 1);
    row("Кут відриву", "°", training.liftAngleStat, 1);
    row("Діапазон", "°", training.rangeAngleStat, 1);
    row("Сила удару", "кгс", training.strikeForce, 2);
    row("Сила відриву", "кгс", training.liftForce, 2);
    row("Час на землі", "мс", training.groundTime, 0);
    row("Час циклу", "мс", training.cycleTime, 0);
    row("Частота", "уд/хв", training.frequency, 1);
    row("Прискорення тіла", "g", training.avgAccHorizStat, 2);
    row("Обертання тіла", "°", training.rotationStat, 1);
    row("Тривалість віддачі", "мс", training.impactDurationStat, 0);
    row("Частота вібрації палиці", "Гц", training.vibrationFreqStat, 1);

    float workCycle =
        (training.cycleTime.avg() > 0)
            ? (training.groundTime.avg() / training.cycleTime.avg()) * 100.0f
            : 0;
    int totalErrs = training.techniqueErrors.total();
    float purity = (training.strikeAngleStat.cnt > 0)
                       ? (((float)totalErrs * 100.0f)/(float)training.strikeAngleStat.cnt)
                       : 0.0f;
    h += "<div class='row'><div><div class='label'>Робочий цикл</div><div "
         "class='val'>" +
         String(workCycle, 1) + " %</div></div></div>";
    h += "<div class='row' style='flex-direction:column; align-items:flex-start;'>";
    h += "<div style='display:flex; justify-content:space-between; width:100%'>";
    h += "<div><div class='label'>Помилки техніки</div><div class='val'>" +
         String(purity,2) + "% (" + String(totalErrs) + " шт)</div></div>";
    h += "</div>";
    h += "<div class='err-list'>";
    if (training.techniqueErrors.lowPositionError.count > 0) h += "<span class='err-item' onclick='showErr(\"lowPositionError\")'>Низька позиція: " + String(training.techniqueErrors.lowPositionError.count) + "</span>";
    if (training.techniqueErrors.rotateHipError.count > 0) h += "<span class='err-item' onclick='showErr(\"rotateHipError\")'>Стегна: " + String(training.techniqueErrors.rotateHipError.count) + "</span>";
    if (training.techniqueErrors.elbowError.count > 0) h += "<span class='err-item' onclick='showErr(\"elbowError\")'>Лікоть: " + String(training.techniqueErrors.elbowError.count) + "</span>";
    if (training.techniqueErrors.motionRangeError.count > 0) h += "<span class='err-item' onclick='showErr(\"motionRangeError\")'>Діапазон: " + String(training.techniqueErrors.motionRangeError.count) + "</span>";
    if (training.techniqueErrors.parallelOperationError.count > 0) h += "<span class='err-item' onclick='showErr(\"parallelOperationError\")'>Паралельність: " + String(training.techniqueErrors.parallelOperationError.count) + "</span>";
    if (training.techniqueErrors.pushError.count > 0) h += "<span class='err-item' onclick='showErr(\"pushError\")'>Поштовх: " + String(training.techniqueErrors.pushError.count) + "</span>";
    h += "</div></div>";
//END
    h += "</div>";
  }

  if (sdAvailable && appState != STATE_TRAINING_ACTIVE) {
    h += "<h2 style='font-size:16px;margin:16px 0 "
         "8px;color:#94a3b8;font-weight:700'>SD Card Files</h2><div "
         "class='card' style='max-height:300px;overflow-y:auto;'>";
    SdFile root;
    if (root.open("/")) {
      int count = 0;
      SdFile f;
      while (f.openNext(&root, O_RDONLY)) {
        char name[64];
        f.getName(name, sizeof(name));
        String sname = String(name);
        if (sname.endsWith(".CSV") || sname.endsWith(".csv")) {
          String dName = sname.startsWith("/") ? sname : "/" + sname;
          h += "<div class='file-link' "
               "style='display:flex;justify-content:space-between;align-items:"
               "center;'><span>📄 " +
               sname + " <small style='color:#64748b'>(" +
               String(f.fileSize()) + "B)</small></span><div style='display:flex;gap:6px;'>";
          if (!sname.endsWith("RAW.csv")) {
            h +=
                "<a href='/load?f=" + dName +
                "' "
                "style='background:linear-gradient(135deg,#38bdf8,#1e40af);"
                "padding:6px "
                "12px;border-radius:8px;text-decoration:none;color:#fff;font-"
                "size:11px;font-weight:bold;'>Load</a>";
          } else {
             h += "<a href='#' onclick='runTest(\"" + dName + "\");return false;' "
                  "style='background:linear-gradient(135deg,#eab308,#ca8a04);"
                  "padding:6px 12px;border-radius:8px;text-decoration:none;"
                  "color:#fff;font-size:11px;font-weight:bold;'>Test</a>";
          }
          h += "<a href='/download?f=" + dName +
               "' style='background:rgba(255,255,255,0.05);padding:6px "
               "12px;border-radius:8px;text-decoration:none;color:#f1f5f9;font-"
               "size:11px;border:1px solid rgba(255,255,255,0.1)'>Save</a>";
          h += "<a href='#' onclick=\"renameFile('" + dName + "','" + sname +
               "');return false;\" "
               "style='background:linear-gradient(135deg,#6366f1,#4f46e5);"
               "padding:6px 10px;border-radius:8px;text-decoration:none;"
               "color:#fff;font-size:11px;'>Rename</a>";
          h += "<a href='#' onclick=\"if(confirm('Видалити " + sname +
               "?'))location.href='/delete?f=" + dName +
               "';return false;\" "
               "style='background:linear-gradient(135deg,#f85032,#e73827);"
               "padding:6px "
               "10px;border-radius:8px;text-decoration:none;color:#fff;font-"
               "size:11px;'>Delete</a></div></div>";
          count++;
        }
        f.close();
      }
      root.close();
      if (count == 0)
        h += "<div class='no-data' style='padding:20px'>No data files "
             "found</div>";
    }
    h += "</div>";
  }

  if (gaugeEnable) {
    gauge.refresh();
    BatteryStatus bs = gauge.getBatteryStatus();
    h += "<dialog id='batModal' "
         "style='padding:24px;border-radius:16px;background:#1e293b;color:#"
         "f1f5f9;border:1px solid "
         "#334155;max-width:90%;margin:auto;box-shadow:0 25px 50px -12px "
         "rgba(0,0,0,0.5)'>";
    h += "<h2 style='margin-bottom:12px;font-size:20px;color:#38bdf8'>Battery "
         "Details</h2><div style='font-size:14px;line-height:1.6'>";
    h += "<b>Temp:</b> " + String(gauge.getTemperature()) +
         "°C<br><b>Voltage:</b> " + String(gauge.getVoltage()) +
         " mV<br><b>Current:</b> <span style='color:#38bdf8'>" +
         String(gauge.getCurrent()) + " mA</span><br>";
    h += "<b>Rem Cap:</b> " + String(gauge.getRemainingCapacity()) +
         " mAh<br><b>Full Cap:</b> " + String(gauge.getFullChargeCapacity()) +
         " mAh<br><b>SoC:</b> <b style='color:#38bdf8'>" +
         String(gauge.getStateOfCharge()) + "%</b><br>";
    h += "<hr style='margin:12px "
         "0;border-color:rgba(255,255,255,0.1)'><b>VBUS:</b> " +
         String(PPM.isVbusIn() ? "Connected" : "Disconnected") + " @ " +
         String(PPM.getVbusVoltage()) + "mV<br>";
    h += "<b>CHG Status:</b> " + String(PPM.getChargeStatusString()) + " (" +
         String(PPM.getChargeCurrent()) + "mA)<br></div>";
    h += "<button onclick='document.getElementById(\"batModal\").close()' "
         "style='margin-top:20px;width:100%;padding:12px;background:#38bdf8;"
         "color:#0f172a;border:none;border-radius:10px;font-weight:bold;cursor:"
         "pointer'>CLOSE</button></dialog>";
    h += F("<dialog id='errModal' style='padding:24px;border-radius:16px;background:#1e293b;color:#f1f5f9;border:1px solid #334155;max-width:90%;margin:auto;box-shadow:0 25px 50px -12px rgba(0,0,0,0.5)'>");
    h += F("<h2 style='margin-bottom:12px;font-size:20px;color:#f43f5e'>Деталі помилки</h2>");
    h += F("<div id='errHintText' style='font-size:15px;line-height:1.6'></div>");
    h += F("<button onclick='document.getElementById(\"errModal\").close()' style='margin-top:20px;width:100%;padding:12px;background:#38bdf8;color:#0f172a;border:none;border-radius:10px;font-weight:bold;cursor:pointer'>ЗРОЗУМІЛО</button></dialog>");
    
    h += F("<dialog id='testModal' style='padding:24px;border-radius:16px;background:#1e293b;color:#f1f5f9;border:1px solid #334155;max-width:90%;margin:auto;box-shadow:0 25px 50px -12px rgba(0,0,0,0.5)'>");
    h += F("<h2 style='margin-bottom:12px;font-size:20px;color:#38bdf8'>Обробка RAW даних</h2>");
    h += F("<div class='progress-container'><div id='testBar' class='progress-bar'></div></div>");
    h += F("<div id='testStatus' style='text-align:center; font-size:14px; color:#94a3b8'>0%</div></dialog>");
    
    h += F("<script>");
    h += F("const errHints = {");
    h += F("lowPositionError: \"Опускання центру ваги або ходьба на напівзігнутих ногах, небезпечне перевантаження колінних суглобів. Або задовга палиця. Кут удару менший 36° \",");
    h += F("rotateHipError: \"\\\"Виляння\\\" стегнами, небезпечне перенапруження попереку. Або закоротка палиця. Не природній рух руки, кут удару > 75° \",");
    h += F("elbowError: \"Робота лише ліктьовим суглобом(рух від ліктя), плече залишається нерухомим. Небезпечне травмування м’язів-розгиначів передпліччя. Короткий час на землі і кут удару більший 75°\",");
    h += F("motionRangeError: \"Рука закінчує рух перед стегном (не перетинає лінію стегна). Горбатість і травмування спини. Короткий час на землі.\",");
    h += F("parallelOperationError: \"Руки рухаються не паралельно, звужуются спереду і розходятся сзаду. Спотикання о палиці, занадто сильна ротація тіла, травмування спини.\",");
    h += F("pushError: \"Відсутність активного поштовху палицею, рука тримає рукоять в момент поштовху, біг з палицями, волочіння палиці, не синхроний рух палицями. Не має жодного єфекту від скандинавської ходьби.\"");
    h += F("};");
    h += F("function showErr(k) { document.getElementById('errHintText').innerText = errHints[k]; document.getElementById('errModal').showModal(); }");
    h += F("function runTest(f) {");
    h += F("  document.getElementById('testBar').style.width = '0%';");
    h += F("  document.getElementById('testStatus').innerText = '0%';");
    h += F("  document.getElementById('testModal').showModal();");
    h += F("  fetch('/test?f=' + f).then(r=>r.json()).then(res=>{");
    h += F("    let itv = setInterval(()=>{");
    h += F("      fetch('/api/test_status').then(r=>r.json()).then(j=>{");
    h += F("        if(j.progress >= 0) {");
    h += F("          document.getElementById('testBar').style.width = j.progress + '%';");
    h += F("          document.getElementById('testStatus').innerText = j.progress + '%';");
    h += F("          if(j.progress >= 100) {");
    h += F("            clearInterval(itv);");
    h += F("            setTimeout(()=>location.reload(), 500);");
    h += F("          }");
    h += F("        }");
    h += F("      }).catch(() => {});");
    h += F("    }, 2500);");
    h += F("  });");
    h += F("}");
    h += F("function renameFile(p, oldName) {");
    h += F("  let n = prompt('Введіть нову назву файлу:', oldName);");
    h += F("  if (n && n !== oldName) {");
    h += F("    fetch('/rename?f=' + p + '&n=' + encodeURIComponent(n))");
    h += F("      .then(r => r.json())");
    h += F("      .then(res => { if (res.ok) location.reload(); else alert('Rename failed'); })");
    h += F("      .catch(() => alert('Network error'));");
    h += F("  }");
    h += F("}");
    h += F("</script>");

  }
  h += "</body></html>";
  return h;
}

void handleSettings() {
  server.send(200, "text/html; charset=utf-8", buildSettingsHTML());
}

void handleSettingsApi() {
  String j =
      "{\"forceThreshold\":" + String(forceThreshold, 1) +
      ",\"cal_pitch_offset\":" + String(cal_pitch_offset, 2) +
      ",\"sdRecordEnable\":" + String(sdRecordEnable ? "true" : "false") +
      ",\"rawRecordEnable\":" + String(rawRecordEnable ? "true" : "false") +
      ",";
  j += "\"poleLength\":" + String(poleLength) +
       ",\"poleWeightGrams\":" + String(poleWeightGrams) +
       ",\"userHeight\":" + String(userHeight) +
       ",\"sensorFreq\":" + String(sensorFreq) + ",";
  j += "\"buzzerEnable\":" + String(buzzerEnable ? "true" : "false") +
       ",\"autoTrainingEnable\":" +
       String(autoTrainingEnable ? "true" : "false") + ",";
  j += "\"forceMultiplier\":" + String(forceMultiplier, 3) + "}";
  server.send(200, "application/json", j);
}

void handleSettingsPost() {
  if (server.hasArg("forceThreshold"))
    forceThreshold = server.arg("forceThreshold").toFloat();
  if (server.hasArg("cal_pitch_offset"))
    cal_pitch_offset = server.arg("cal_pitch_offset").toFloat();
  if (server.hasArg("sdRecordEnable"))
    sdRecordEnable = (server.arg("sdRecordEnable") == "true" ||
                      server.arg("sdRecordEnable") == "1");
  if (server.hasArg("rawRecordEnable"))
    rawRecordEnable = (server.arg("rawRecordEnable") == "true" ||
                       server.arg("rawRecordEnable") == "1");
  if (server.hasArg("poleLength"))
    poleLength = (uint8_t)server.arg("poleLength").toInt();
  if (server.hasArg("poleWeightGrams"))
    poleWeightGrams = (uint16_t)server.arg("poleWeightGrams").toInt();
  if (server.hasArg("userHeight"))
    userHeight = (uint8_t)server.arg("userHeight").toInt();
  if (server.hasArg("sensorFreq"))
    sensorFreq = (uint16_t)server.arg("sensorFreq").toInt();
  if (server.hasArg("buzzerEnable"))
    buzzerEnable = (server.arg("buzzerEnable") == "true" ||
                    server.arg("buzzerEnable") == "1");
  if (server.hasArg("autoTrainingEnable"))
    autoTrainingEnable = (server.arg("autoTrainingEnable") == "true" ||
                          server.arg("autoTrainingEnable") == "1");
  if (server.hasArg("gFactor"))
    gFactor = server.arg("gFactor").toFloat();
  if (server.hasArg("forceMultiplier"))
    forceMultiplier = server.arg("forceMultiplier").toFloat();
  forceThresholdSq = forceThreshold * forceThreshold;
  savePrefs();
  loadPrefs();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrate() {
  cal_pitch_offset = getEulerPitch();
  server.send(200, "application/json",
              "{\"cal_pitch_offset\":" + String(cal_pitch_offset, 2) + "}");
}
void handleCalibrateForceStart() {
  isCalibratingForce = true;
  calForceStartMs = millis();
  calForceIndex = 0;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrateForceStatus() {
  String j = "{";
  bool done = (!isCalibratingForce || (millis() - calForceStartMs >= 3000));
  j += "\"done\":" + String(done ? "true" : "false") + ",";
  if (done && calForceIndex > 20) {
    float maxVal = 0;
    int maxIdx = 0;
    for (int i = 0; i < calForceIndex; i++) {
      if (calForceBuffer[i] > maxVal) {
        maxVal = calForceBuffer[i];
        maxIdx = i;
      }
    }
    float sumVal = 0;
    int countVal = 0;
    int startIdx = maxIdx - 15;
    if (startIdx < 0)
      startIdx = 0;
    int endIdx = maxIdx - 5;
    if (endIdx < 0)
      endIdx = 0;
    for (int i = startIdx; i <= endIdx; i++) {
      if (calForceBuffer[i] > 0.3f && calForceBuffer[i] < 5.0f) {
        sumVal += calForceBuffer[i];
        countVal++;
      }
    }
    if (countVal > 5) {
      gFactor = 1.0f / (sumVal / countVal);
      savePrefs();
      j += "\"gFactor\":" + String(gFactor, 4) + ",\"success\":true";
    } else {
      j += "\"success\":false,\"msg\":\"Error identifying free fall\"";
    }
  } else {
    j += "\"success\":false,\"msg\":\"Calibrating...\"";
  }
  j += "}";
  server.send(200, "application/json", j);
}

void handleDiag() {
  lastSettingsActivityMs = millis();
  float peakG = getAndResetPeakForce();
  String j = "{";
  j += "\"pitch\":" + String(getPitch(), 1) + ",";
  j += "\"peakAcc\":" + String(peakG, 2) + ",";
  j += "\"peakForce\":" + String(accToKgf(peakG) * gFactor, 2) + ",";
  j += "\"Timer\":" + String(TIMER_SLEEP - (millis() - timerSleep)) + ",";

  if (gaugeEnable && gauge.refresh()) {
    j += "\"batVoltage\":" + String(gauge.getVoltage()) + ",";
    j += "\"batPercent\":" + String(gauge.getStateOfCharge()) + ",";
    j += "\"batCurrent\":" + String(gauge.getCurrent()) + ",";
    j += "\"batTemp\":" + String(gauge.getTemperature(), 1) + ",";
    j += "\"vbus\":" + String(PPM.getVbusVoltage()) + ",";
    j += "\"chgStatus\":\"" + String(PPM.getChargeStatusString()) + "\"";
  } else {
    j += "\"batVoltage\":0,\"batPercent\":0,\"batCurrent\":0,\"batTemp\":0,"
         "\"vbus\":0,\"chgStatus\":\"Off\"";
  }
  j += "}";
  server.send(200, "application/json", j);
}

String buildSettingsHTML() {
  //reset timer
  timerSleep = millis();

  String h = F(
      "<!DOCTYPE html><html lang='uk'><head>"
      "<meta charset='UTF-8'><meta name='viewport' "
      "content='width=device-width,initial-scale=1'>"
      "<title>Settings - Walker</title><style>"
      "*{margin:0;padding:0;box-sizing:border-box}"
      "body{font-family:-apple-system,system-ui,sans-serif;background:#0f172a;"
      "color:#f8fafc;padding:16px;line-height:1.5}"
      ".nav{display:flex;align-items:center;margin-bottom:24px;gap:16px}"
      ".back-btn{text-decoration:none;color:#38bdf8;font-size:24px;width:40px;"
      "height:40px;display:flex;align-items:center;justify-content:center;"
      "background:rgba(56,189,248,0.1);border-radius:50%}"
      "h1{font-size:24px;font-weight:700;color:#38bdf8}"
      ".card{background:rgba(30,41,59,0.7);backdrop-filter:blur(12px);border-"
      "radius:16px;padding:20px;margin-bottom:20px;border:1px solid "
      "rgba(255,255,255,0.1);box-shadow:0 4px 6px -1px rgba(0,0,0,0.1)}"
      "h2{font-size:14px;text-transform:uppercase;letter-spacing:0.05em;color:#"
      "94a3b8;margin-bottom:16px;border-bottom:1px solid "
      "rgba(255,255,255,0.05);padding-bottom:8px}"
      ".field{display:flex;justify-content:space-between;align-items:center;"
      "margin-bottom:16px;gap:12px}"
      ".field:last-child{margin-bottom:0}"
      ".label{font-size:15px;font-weight:500}"
      "input[type='number'],input[type='text']{background:#334155;border:1px "
      "solid #475569;color:#fff;padding:8px "
      "12px;border-radius:8px;width:100px;font-size:15px;transition:border-"
      "color 0.2s}"
      "input:focus{outline:none;border-color:#38bdf8;background:#1e293b}"
      "input[type='checkbox']{width:20px;height:20px;accent-color:#38bdf8}"
      ".btn{display:inline-flex;align-items:center;justify-content:center;"
      "padding:10px "
      "20px;border-radius:10px;font-weight:600;font-size:15px;cursor:pointer;"
      "border:none;transition:all 0.2s;gap:8px}"
      ".btn-primary{background:#38bdf8;color:#0f172a;width:100%}"
      ".btn-primary:active{transform:scale(0.98);background:#0ea5e9}"
      ".btn-sec{background:rgba(255,255,255,0.05);color:#fff;border:1px solid "
      "rgba(255,255,255,0.1);padding:6px 12px;font-size:13px}"
      ".btn-sec:hover{background:rgba(255,255,255,0.1)}"
      "dialog{background:#1e293b;color:#f8fafc;border:1px solid "
      "#334155;border-radius:16px;padding:24px;margin:auto;max-width:320px;box-"
      "shadow:0 25px 50px -12px rgba(0,0,0,0.5)}"
      "dialog::backdrop{background:rgba(0,0,0,0.8);backdrop-filter:blur(4px)}"
      ".diag-title{font-size:18px;font-weight:700;margin-bottom:12px;display:"
      "block}"
      ".diag-msg{font-size:14px;color:#94a3b8;margin-bottom:20px;display:block}"
      ".diag-row{display:flex;justify-content:space-between;padding:4px "
      "0;border-bottom:1px solid rgba(255,255,255,0.05)}"
      ".diag-row:last-child{border-bottom:none}"
      ".diag-label{color:#94a3b8;font-size:13px}"
      ".diag-val{font-family:monospace;color:#38bdf8;font-weight:bold}"
      "</style></head><body>");

  h += F("<div class='nav'><a href='/' "
         "class='back-btn'>←</a><h1>Налаштування</h1></div>");

  // Секція 1: Алгоритм
  h += F("<div class='card'><h2>Алгоритм та Чутливість</h2>");
  h += "<div class='field'><span>Поріг удару (G)</span><input type='number' "
       "step='0.1' id='forceThreshold' value='" +
       String(forceThreshold, 1) + "'></div>";
  h += "<div class='field'><span>Зсув Pitch (°)</span><div "
       "style='display:flex;gap:8px;align-items:center'><input type='number' "
       "step='0.01' id='cal_pitch_offset' value='" +
       String(cal_pitch_offset, 2) +
       "'><button class='btn btn-sec' "
       "onclick='doCalibrate()'>🎯</button></div></div>";
  h += "<div class='field'><span>G-Factor</span><div "
       "style='display:flex;gap:8px;align-items:center'><input type='number' "
       "step='0.0001' id='gFactor' value='" +
       String(gFactor, 4) +
       "'><button class='btn btn-sec' "
       "onclick='openForceModal()'>⚖️</button></div></div>";
  h += "<div class='field'><span>Коефіцієнт сили (множник)</span><input "
       "type='number' step='0.001' id='forceMultiplier' value='" +
       String(forceMultiplier, 3) + "'>";
  h += F("</div>");

  // Секція 2: Параметри палиць
  h += F("<div class='card'><h2>Параметри обладнання</h2>");
  h += "<div class='field' style='margin-bottom:8px'><span>Довжина палиці "
       "(см)</span><input type='number' id='poleLength' value='" +
       String(poleLength) + "'></div>";
  h += F("<div style='display:flex;margin-bottom:16px;'><button class='btn "
         "btn-sec' style='width:100%;' "
         "onclick='openPoleAdviseModal()'>Підібрати</button></div>");
  h += "<div class='field'><span>Вага палиці (г)</span><input type='number' "
       "id='poleWeightGrams' value='" +
       String(poleWeightGrams) + "'></div>";
  h += "<div class='field'><span>Зріст користувача (см)</span><input "
       "type='number' id='userHeight' value='" +
       String(userHeight) + "'></div>";
  h += "<div class='field'><span>Частота сенсора (Гц)</span><input "
       "type='number' id='sensorFreq' value='" +
       String(sensorFreq) + "'></div>";
  h += F("</div>");

  // Секція 3: Опції
  h += F("<div class='card'><h2>Додаткові опції</h2>");
  h += "<div class='field'><span>Запис на SD</span><input type='checkbox' "
       "id='sdRecordEnable' " +
       String(sdRecordEnable ? "checked" : "") + "></div>";
  h += "<div class='field'><span>RAW Data</span><input type='checkbox' "
       "id='rawRecordEnable' " +
       String(rawRecordEnable ? "checked" : "") + "></div>";
  h += "<div class='field'><span>Звукові сигнали</span><input type='checkbox' "
       "id='buzzerEnable' " +
       String(buzzerEnable ? "checked" : "") + "></div>";
  h += "<div class='field'><span>Авто-тренування</span><input type='checkbox' "
       "id='autoTrainingEnable' " +
       String(autoTrainingEnable ? "checked" : "") + "></div>";
  h += F("</div>");

  // --- Diagnostics ---
  h += "<div class='card'>";
  h += "<h2>Діагностика <span "
       "style='font-size:10px;color:#555'>(live)</span></h2>";
  h += "<div class='diag-row'><span class='diag-label'>Кут палиці</span><span "
       "class='diag-val' id='dPitch'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Макс. прискорення "
       "(G)</span><span class='diag-val' id='dPeakAcc'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Сила удару "
       "(MAX)</span><span class='diag-val' id='dPeak'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Батарея</span><span "
       "class='diag-val' id='dBat'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Напруга</span><span "
       "class='diag-val' id='dVolt'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Струм</span><span "
       "class='diag-val' id='dCurr'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Температура</span><span "
       "class='diag-val' id='dTemp'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>VBUS</span><span "
       "class='diag-val' id='dVbus'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Зарядка</span><span "
       "class='diag-val' id='dChg'>—</span></div>";
  h += "<div class='diag-row'><span class='diag-label'>Таймер</span><span "
       "class='diag-val' id='dTimer'>—</span></div>";
  h += "</div>";

  h += F("<button class='btn btn-primary' onclick='doSave()'>Зберегти "
         "налаштування</button>");

  // Модалка калібрування сили
  h += F("<dialog id='forceModal'><span class='diag-title'>Калібрування "
         "ваги</span>"
         "<span class='diag-msg'>Натисніть СТАРТ та залиште палицю у стані "
         "вільного падіння на 3 секунди (наприклад, відпустіть її на м'яку "
         "поверхню).</span>"
         "<button class='btn btn-primary' id='forceStartBtn' "
         "onclick='doCalForce()'>Почати (3с)</button>"
         "<div id='forceStatus' "
         "style='display:none;margin-top:16px;text-align:center'><div "
         "style='color:#38bdf8;font-weight:bold'>Вимірювання...</div></div>"
         "<button class='btn btn-sec' style='width:100%;margin-top:12px' "
         "onclick='document.getElementById(\"forceModal\").close()'>Скасувати</"
         "button></dialog>");

  // Модалка підбору довжини палиці
  h += F("<dialog id='poleAdviseModal'><span class='diag-title'>Підбір довжини "
         "палиці</span>"
         "<span class='diag-msg'>Візьміть палицю налаштуйте її на довжину на - "
         "<b id='poleAdviseCalc' style='color:#38bdf8;'></b> "
         "см.<br><br>Станьте на рівну поверхню, відведіть палицю назад і "
         "опустіть руку вздовж стегна.</span>"
         "<div style='text-align:center;margin:20px 0;'>"
         "<div id='advisePitch' "
         "style='font-size:48px;font-weight:bold;transition:color "
         "0.3s;'>--&deg;</div>"
         "<div id='adviseHint' "
         "style='font-size:16px;min-height:24px;margin-top:8px;'></div>"
         "</div>"
         "<button class='btn btn-primary' style='width:100%;' "
         "onclick='location.href=\"/\"'>Ок</button></dialog>");

  h += F(
      "<script>"
      "function doSave(){"
      "  const d = {"
      "    forceThreshold: document.getElementById('forceThreshold').value,"
      "    cal_pitch_offset: document.getElementById('cal_pitch_offset').value,"
      "    gFactor: document.getElementById('gFactor').value,"
      "    forceMultiplier: document.getElementById('forceMultiplier').value,"
      "    poleLength: document.getElementById('poleLength').value,"
      "    poleWeightGrams: document.getElementById('poleWeightGrams').value,"
      "    userHeight: document.getElementById('userHeight').value,"
      "    sensorFreq: document.getElementById('sensorFreq').value,"
      "    sdRecordEnable: document.getElementById('sdRecordEnable').checked,"
      "    rawRecordEnable: document.getElementById('rawRecordEnable').checked,"
      "    buzzerEnable: document.getElementById('buzzerEnable').checked,"
      "    autoTrainingEnable: "
      "document.getElementById('autoTrainingEnable').checked"
      "  };"
      "  fetch('/api/settings', {method:'POST', body: new URLSearchParams(d)})"
      "    .then(r=>r.json()).then(res=>{ if(res.ok) location.href='/'; });"
      "}"
      "function doCalibrate(){"
      "  fetch('/api/calibrate', {method:'POST'}).then(r=>r.json()).then(res=>{"
      "    document.getElementById('cal_pitch_offset').value = "
      "res.cal_pitch_offset;"
      "    alert('Pitch calibrated!');"
      "  });"
      "}"
      "function openForceModal(){ "
      "document.getElementById('forceModal').showModal(); }"
      "function openPoleAdviseModal(){"
      "  const h = parseFloat(document.getElementById('userHeight').value) || "
      "0;"
      "  document.getElementById('poleAdviseCalc').innerText = (h * "
      "0.7).toFixed(1);"
      "  document.getElementById('poleAdviseModal').showModal();"
      "}"
      "function doCalForce(){"
      "  document.getElementById('forceStartBtn').disabled = true;"
      "  document.getElementById('forceStatus').style.display='block';"
      "  fetch('/api/cal_force_start', {method:'POST'}).then(()=>{"
      "    let itv = setInterval(()=>{ "
      "      fetch('/api/cal_force_status').then(r=>r.json()).then(res=>{"
      "        if(res.done){ "
      "          clearInterval(itv); "
      "          if(res.success){ "
      "            document.getElementById('gFactor').value = res.gFactor;"
      "            alert('G-Factor updated to ' + res.gFactor); "
      "            document.getElementById('forceModal').close();"
      "          } else { "
      "            alert('Error: ' + res.msg); "
      "            document.getElementById('forceStartBtn').disabled = false;"
      "            document.getElementById('forceStatus').style.display='none';"
      "          }"
      "        }"
      "      }); "
      "    }, 1000);"
      "  });"
      "}"
      "function updateDiag(){"
      "  fetch('/api/diag').then(r=>r.json()).then(j=>{"
      "    document.getElementById('dPitch').innerText = j.pitch + '°';"
      "    document.getElementById('dPeakAcc').innerText = j.peakAcc + ' G';"
      "    document.getElementById('dPeak').innerText = j.peakForce + ' kgf';"
      "    document.getElementById('dBat').innerText = j.batPercent + ' %';"
      "    document.getElementById('dVolt').innerText = j.batVoltage + ' mV';"
      "    document.getElementById('dCurr').innerText = j.batCurrent + ' mA';"
      "    document.getElementById('dTemp').innerText = j.batTemp + ' °C';"
      "    document.getElementById('dVbus').innerText = j.vbus + ' mV';"
      "    document.getElementById('dChg').innerText = j.chgStatus;"
      "    document.getElementById('dTimer').innerText = j.Timer;"
      "    "
      "    const p = j.pitch;"
      "    const ap = document.getElementById('advisePitch');"
      "    const ah = document.getElementById('adviseHint');"
      "    if(ap && ah) {"
      "      ap.innerHTML = p.toFixed(1) + '&deg;';"
      "      if(p >= 37 && p <= 42) {"
      "        ap.style.color = '#10b981';"
      "        ah.innerText = 'Ідеально';"
      "        ah.style.color = '#94a3b8';"
      "      } else if(p < 37) {"
      "        ap.style.color = '#ef4444';"
      "        ah.innerText = 'Зменшіть довжину палиці';"
      "        ah.style.color = '#ef4444';"
      "      } else {"
      "        ap.style.color = '#ef4444';"
      "        ah.innerText = 'Збільшить довжину палиці';"
      "        ah.style.color = '#ef4444';"
      "      }"
      "    }"
      "  });"
      "}"
      "setInterval(updateDiag, 1000); updateDiag();"
      "</script></body></html>");
  return h;
}
