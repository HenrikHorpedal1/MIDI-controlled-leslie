#pragma once

#include <Arduino.h>

enum class InputSource : uint8_t {
    Footswitch,
    ExpPedal,
    Midi
};

struct FootswitchState {
    bool swA;
    bool swB;
};

struct ExpPedalState {
    float value01;
    float angleDeg;
};

struct MidiCCEvent {
    uint8_t channel;
    uint8_t controller;
    uint8_t value;
};

struct InputEvent {
    InputSource source;
    union {
        FootswitchState foot;
        ExpPedalState   exp;
        MidiCCEvent     midi;
    } data;
};
