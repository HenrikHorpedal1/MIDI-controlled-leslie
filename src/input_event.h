#pragma once

#include <Arduino.h>
#include <cstdint>

enum class InputSource : uint8_t {
    Footswitch,
    ExpPedal,
    MidiButton,
    MidiCC
};

struct FootswitchState {
    bool swA;
    bool swB;
};

struct ExpPedalState {
    float voltage;
};

struct MidiCCEvent {
    uint8_t value;
};

enum class MidiButtonEvent : uint8_t {
    BUTTON0,
    BUTTON1,
    BUTTON2
};

struct InputEvent {
    InputSource source;
    union {
        FootswitchState foot;
        ExpPedalState   exp;
        MidiCCEvent     midiCC;
        MidiButtonEvent midiButton;
    } data;
};
