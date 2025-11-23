
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "SD-logger.h"

#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// Single log file for now
static const char *FILE_NAME = "/log.csv";

void setupLogger() {
  // Init SPI for SD
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);

  Serial.println("Mounting SD...");
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    while (1) { delay(1000); }
  }
  Serial.println("SD OK.");

  // Make sure file exists and has header
  File f = SD.open(FILE_NAME, FILE_APPEND);  // create if not exists
  if (!f) {
    Serial.println("Failed to open log file in setupLogger()");
    return;
  }

  if (f.size() == 0) {
    // Adjust header to match what you actually log
    f.println("millis,measuredRPMx100,targetRPMx100,P,I,u,pwmCmd");
    Serial.println("Created log file with header.");
  } else {
    Serial.print("Log file exists, size = ");
    Serial.println(f.size());
  }
  f.close();
}

void logLine(const String &line) {
  File f = SD.open(FILE_NAME, FILE_APPEND);  // APPEND is critical here
  if (f) {
    f.println(line);
    f.close();
  } else {
    Serial.println("Failed to open log file in logLine()");
  }
}

