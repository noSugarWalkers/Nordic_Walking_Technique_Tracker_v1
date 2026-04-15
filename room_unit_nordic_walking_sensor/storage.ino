#include "config.h"
#include "types.h"

// ============================================================
// Storage Globals (externs from main)
// ============================================================
extern SdFat sd;
extern bool sdAvailable;
extern char buf[128];
extern String fname;
extern SDLogger logger;
extern SPIClass spiSD;
extern bool sdRecordEnable;
extern bool rawRecordEnable;

/**
 * @brief Initialize SD card storage
 */
void setupSD() {
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (sd.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(24), &spiSD))) {
    sdAvailable = sdRecordEnable;
    Serial.println("SD initialized OK");
  } else {
    sdAvailable = false;
    Serial.println("⚠️ SD init failed — card missing or wiring error");
  }
}

// ============================================================
// SDLogger Implementations (Class defined in types.h)
// ============================================================

bool SDLogger::begin() {
  if (!sdAvailable)
    return false;

  // Find next file name
  uint16_t maxIndex = 0;
  SdFile root;
  if (!root.open("/")) {
    Serial.println("Cannot open root");
    return false;
  }
  Serial.println("Root OK");

  SdFile f;
  while (f.openNext(&root, O_RDONLY)) {
    char name[64];
    f.getName(name, sizeof(name));
    String sname = String(name);
    if (sname.endsWith(".CSV") || sname.endsWith(".csv")) {
      int idx = sname.substring(0, sname.indexOf('.')).toInt();
      if (idx > maxIndex)
        maxIndex = idx;
    }
    f.close();
  }
  root.close();

  char nbuf[16];
  snprintf(nbuf, sizeof(nbuf), "/%03u.csv", maxIndex + 1);
  fname = String(nbuf);

  if (!file.open(fname.c_str(), O_RDWR | O_CREAT | O_APPEND)) {
    sdAvailable = false;
    return false;
  } else {
    file.println("Step,StrikeAngle,LiftAngle,StrikeForce(kgf),LiftForce(kgf),"
                 "AccHoriz(g),"
                 "GroundTime(ms),"
                 "CycleTime(ms),Freq(s/m),VibeDuration,VibeFreq,TimeLeft");
  }

  if (rawRecordEnable) {
    char rawNbuf[24];
    snprintf(rawNbuf, sizeof(rawNbuf), "/%03u_RAW.csv", maxIndex + 1);
    if (rawFile.open(rawNbuf, O_RDWR | O_CREAT | O_APPEND)) {
      rawFile.println("Acc(g),Pitch(deg),HorizAcc(g),Time(ms)");
    }
  }

  c = 0;
  return true;
}

void SDLogger::log(const char *line) {
  if (file.isOpen()) {
    file.println(line);
    c++;
    if (c > 50) {
      // Flush every 50 steps to prevent data loss. Every 100m
      file.flush();
      c = 0;
    }
  } else {
    logger.begin();
    file.println(line);
    c++;
  }
}

void SDLogger::close() {
  if (!sdAvailable)
    return;
  flushRawBuffer();  // Flush remaining RAW data before closing
  if (file.isOpen())
    file.close();
  if (rawFile.isOpen())
    rawFile.close();
  delay(50);
}

void SDLogger::logRaw(float acc, float pitch, float horizAcc, unsigned long timeMs) {
  if (!rawRecordEnable) return;
  if (rawBufHead >= RAW_BUF_SIZE) {
    flushRawBuffer();
    if (rawBufHead >= RAW_BUF_SIZE) rawBufHead = 0; // Prevent out-of-bounds
  }
  rawBuf[rawBufHead] = {acc, pitch, horizAcc, timeMs};
  rawBufHead++;
}

void SDLogger::flushRawBuffer() {
  if (!rawFile.isOpen() || rawBufHead == 0) {
    rawBufHead = 0;
    return;
  }
  char rb[64];
  for (uint8_t i = 0; i < rawBufHead; i++) {
    snprintf(rb, sizeof(rb), "%.2f,%.2f,%.2f,%lu",
             rawBuf[i].acc, rawBuf[i].pitch, rawBuf[i].horizAcc, rawBuf[i].timeMs);
    rawFile.println(rb);
  }
  rawFile.flush();
  rawBufHead = 0;
}