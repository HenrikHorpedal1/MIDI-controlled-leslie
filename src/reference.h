// reference.h
#pragma once

#include <Arduino.h>

struct ReferenceState {
    float angleDeg;
    float velRPM;
};

void referenceInit();
void referenceSet(const ReferenceState &ref);
void referenceGet(ReferenceState &out);
