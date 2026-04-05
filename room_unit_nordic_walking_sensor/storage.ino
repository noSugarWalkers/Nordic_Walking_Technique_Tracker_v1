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
                 "CycleTime(ms),Freq(s/m),TimeLeft");
  }
  c = 0;
  return true;
}

void SDLogger::log(const char *line) {
  if (file.isOpen()) {
    file.println(line);
    c++;
    if (c > 200) {
      // Flush every 200 steps to prevent data loss
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
  if (file.isOpen())
    file.close();
  delay(50);
}