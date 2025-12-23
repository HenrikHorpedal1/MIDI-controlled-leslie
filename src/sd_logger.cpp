// sd_logger.cpp
#include "sd_logger.h"

#include <SD.h>
#include <SPI.h>

// ---------------- Pin config (from your note) ----------------
#define SD_CS   21
#define SD_SCK  48
#define SD_MISO 47
#define SD_MOSI 38
// -------------------------------------------------------------

static File  logFile;
static bool  sdReady          = false;
static bool  headerWritten    = false;
static uint32_t linesSinceFlush = 0;

bool sdLoggerBegin()
{
    if (sdReady) {
        return true;
    }

    Serial.println("SD: initializing...");

    // Configure the default SPI bus on your custom pins
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    // Use simple SD.begin(csPin) to avoid weirdness between ESP32 variants
    if (!SD.begin(SD_CS)) {
        Serial.println("SD: initialization failed");
        return false;
    }

    // Create a new file: /log_00.csv, /log_01.csv, ...
    char filename[16];
    uint8_t index = 0;
    do {
        snprintf(filename, sizeof(filename), "/log_%02u.csv", index++);
    } while (SD.exists(filename) && index < 100);

    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("SD: failed to open log file");
        return false;
    }

    Serial.print("SD: logging to ");
    Serial.println(filename);

    // CSV header: one line per control iteration
    logFile.println(
        "t_us,loop,mode,"
        "refAngleDeg,refVelRpm,"
        "measAngleDeg,measVelRpm,"
        "error,input,P,I,D"
    );
    logFile.flush();

    headerWritten = true;
    sdReady       = true;

    return true;
}

void sdLoggerLogControllerSample(const char* modeName,
                                 float refAngleDeg,
                                 float refVelRpm,
                                 float measAngleDeg,
                                 float measVelRpm,
                                 float error,
                                 float input,
                                 float P,
                                 float I,
                                 float D,
                                 uint32_t loopCounter)
{
    if (!sdReady || !logFile || !headerWritten || !modeName) {
        return;
    }

    static uint32_t decimCounter = 0;
    constexpr uint32_t LOG_EVERY_N = 5;  // <--- tweak this if you like

    if (++decimCounter < LOG_EVERY_N) {
        return;    // skip this sample
    }
    decimCounter = 0;

    const uint32_t t_us = micros();

    // Write a single CSV line. Keep it simple & fast.
    logFile.print(t_us);
    logFile.print(',');
    logFile.print(loopCounter);
    logFile.print(',');
    logFile.print(modeName);
    logFile.print(',');

    logFile.print(refAngleDeg, 6);
    logFile.print(',');
    logFile.print(refVelRpm, 6);
    logFile.print(',');

    logFile.print(measAngleDeg, 6);
    logFile.print(',');
    logFile.print(measVelRpm, 6);
    logFile.print(',');

    logFile.print(error, 6);
    logFile.print(',');
    logFile.print(input, 6);
    logFile.print(',');
    logFile.print(P, 6);
    logFile.print(',');
    logFile.print(I, 6);
    logFile.print(',');
    logFile.println(D, 6);

    // Flush occasionally to avoid data loss but not every line
    if (++linesSinceFlush >= 200) {
        logFile.flush();
        linesSinceFlush = 0;
    }
}

void sdLoggerFlush()
{
    if (sdReady && logFile) {
        logFile.flush();
    }
}
