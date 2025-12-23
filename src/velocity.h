#pragma once

#include <Arduino.h>
#include "quadrature-encoder.h"

struct VelocityState {
    float velCountsPerSec;  // signed, filtered
    float velRpm;           // signed, filtered
    bool  valid;            // true once homed (or at least one good sample)
};

void velocityInit();

// Called from encoderTask when a full encoder step (±1 count) occurred.
// edgeMicros should be the timestamp of that step (from the edge ISR buffer).
void velocityPushStep(int8_t stepDir, uint32_t edgeMicros);

// Pure getter: just returns the last state written by the velocity task.
void velocityGetState(VelocityState &out);

