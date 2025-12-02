#pragma once

#include <Arduino.h>
#include "quadrature-encoder.h"

struct VelocityState {
    float velCountsPerSec;  
    float velRpm;           
    bool  valid;            
};

void velocityInit();

void velocityUpdate();

void velocityGetState(VelocityState &out);
