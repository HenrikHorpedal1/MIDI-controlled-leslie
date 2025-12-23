// sd_logger.h
#pragma once

#include <Arduino.h>

// Initialize SD card and open a new CSV log file.
// Call this once in setup(). Returns true on success.
bool sdLoggerBegin();

// Log one controller sample (called from controllerTask).
// Always writes one line (no throttling/decimation).
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
                                 uint32_t loopCounter);

// Optionally force a flush (e.g. before reset/shutdown).
void sdLoggerFlush();
