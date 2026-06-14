#pragma once

#include <Arduino.h>

enum class ControlSource {
    Footswitch,
    ExpressionPedal,
    MidiKeyboard,
    MidiBeatSync,
};

// Returns the currently active control source (thread-safe)
ControlSource modeSelectorGetSource();

void modeSelectorTask(void* pvParameters);
