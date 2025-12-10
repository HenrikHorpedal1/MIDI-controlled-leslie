// reference.h
#pragma once

#include <Arduino.h>

struct ReferenceState {
    float angleDeg;      
    float velRPM;  
};

enum class RefSource : uint8_t {
    Footswitch = 0,
    ExpPedal,
    MidiButton,
    MidiCC
};

void referenceInit();

void referenceSetFrom(RefSource src, const ReferenceState &ref);

void referenceSetMode(RefSource mode);

RefSource referenceGetMode();

void referenceGetActive(ReferenceState &out);

