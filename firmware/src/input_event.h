#pragma once

#include <Arduino.h>
#include <cstdint>

#include "source-selector.h"   // InputSource (rotary selection), carried by SourceChange

// Classifies an InputEvent (selects which union member below is valid).
enum class EventType : uint8_t {
    Footswitch,
    ExpPedal,
    MidiButton,
    MidiCC,
    SourceChange   // rotary selector moved to a new position
};

struct FootswitchState {
    bool swA;
    bool swB;
};

struct ExpPedalState {
    float voltage;
};

struct MidiCCEvent {
    uint8_t control;   // CC number (distinguishes e.g. rate vs. subdivision)
    uint8_t value;     // 0..127
};

enum class MidiButtonEvent : uint8_t {
    BUTTON0,
    BUTTON1,
    BUTTON2
};

struct InputEvent {
    EventType type;
    union {
        FootswitchState foot;
        ExpPedalState   exp;
        MidiCCEvent     midiCC;
        MidiButtonEvent midiButton;
        InputSource     source;     // for SourceChange (newly selected source)
    } data;
};
