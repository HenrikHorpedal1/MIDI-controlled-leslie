#pragma once

#include <Arduino.h>

// The control source selected by the rotary switch — i.e. where the operator's
// commands come from. The active source gates which inputs the handler acts on.
enum class InputSource {
    Footswitch,
    ExpressionPedal,
    MidiKeyboard,
    MidiBeatSync,
};

// Reads the rotary selector, debounces it, and pushes a SourceChange InputEvent
// onto the input queue (passed via pvParameters) whenever the position changes
// — including once for the position detected at startup. The source selector is
// a pure event producer; it holds no shared state.
void sourceSelectorTask(void* pvParameters);
