#pragma once

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

using Moteus = MoteusController<ACAN2517FD>;

// Base PID gains configured on the moteus at startup.
// Use these as the denominator when computing real-time kp/kd/ilimit scales.
static constexpr double MOTEUS_BASE_KP = 7.0;
static constexpr double MOTEUS_BASE_KI = 0.4;
static constexpr double MOTEUS_BASE_KD = 0.7;

bool configureMoteus(Print& debug = Serial);
Moteus& hornMoteus();
