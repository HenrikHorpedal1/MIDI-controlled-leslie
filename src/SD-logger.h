#pragma once
#include <Arduino.h>

void setupLogger();                 // Call once at startup
void logLine(const String &line);   // Call whenever you want to log a CSV line

