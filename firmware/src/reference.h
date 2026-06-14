// reference.h
#pragma once

#include <Arduino.h>
#include <cstdint>

// Operator intent (Park is a controller-internal state, not an intent — it is
// the terminal state of a Velocity(0) request once the rotor has slowed).
enum class DriveMode : uint8_t { Velocity, BeatSync };

// A single, coherent snapshot of what the operator wants. Written as a whole by
// exactly one producer (the input handler) and read by the controller.
struct Reference {
    DriveMode mode;
    float     hornRPM;
    float     drumRPM;
};

void referenceInit();
void referenceSet(const Reference &ref);
void referenceGet(Reference &out);
