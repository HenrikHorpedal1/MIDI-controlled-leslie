#pragma once

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

using Moteus = MoteusController<ACAN2517FD>;

bool configureMoteus(Print& debug = Serial);
Moteus& hornMoteus();
Moteus& drumMoteus();

// Shared moteus command/query formats used by the control loop:
//   - position format: float accel_limit + velocity_limit fields
//   - query format: float position/velocity plus encoder 1 (rotor) pos/vel
const mm::PositionMode::Format& lesliePositionFmt();
const mm::Query::Format&        leslieQueryFmt();
