// reference.h
#pragma once

#include <Arduino.h>

enum class Rotor : uint8_t { Horn, Drum };

struct ReferenceState {
    float angleDeg;
    float velRPM;
};

void referenceInit();
void referenceSet(Rotor rotor, const ReferenceState &ref);
void referenceGet(Rotor rotor, ReferenceState &out);
