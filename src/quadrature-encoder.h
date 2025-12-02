#pragma once

#include <Arduino.h>

struct EncoderState {
    int32_t count;        
    float   relAngleDeg;  
    float   absAngleDeg;  
    bool    homed;        
};
void encoderInit();
void getEncoderState(EncoderState &state);
