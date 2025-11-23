#pragma once

#include <Arduino.h>

struct EncoderState {
    int32_t count;        // multi-turn incremental count
    float   relAngleDeg;  // raw angle from count (can be <0 or >360)
    float   absAngleDeg;  // 0–360 absolute angle (valid when homed == true)
    bool    homed;        // true after first successful Z-based home
};
void encoderInit();
void getEncoderState(EncoderState &state);
