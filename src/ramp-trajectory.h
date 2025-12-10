#pragma once

#include "reference.h"
#include <Arduino.h>
#include <cstdint>

enum class SpeedCommand : uint8_t {
    CHORALE,
    STOP,
    TREMOLO
};

void rampTrajectoryInit();
void rampTrajectoryStartTask(UBaseType_t priority = 3);
void rampTrajectoryCommand(SpeedCommand cmd, RefSource src);
