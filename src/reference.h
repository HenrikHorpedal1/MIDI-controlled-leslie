// reference.h
#pragma once

#include <Arduino.h>

struct ReferenceState {
    float angleDeg;      
    float velRPM;  
    bool  enabled;       
    bool  valid;         
};

enum class RefSource : uint8_t {
    Midi       = 0,
    ExpPedal   = 1,
    FootSwitch = 2
};

struct AllReferences {
    ReferenceState midi;
    ReferenceState expPedal;
    ReferenceState footSwitch;
    RefSource      activeSource;
};

void referenceInit();

void referenceSetFrom(RefSource src, const ReferenceState &ref);

void referenceSetMode(RefSource mode);

RefSource referenceGetMode();

void referenceGetActive(ReferenceState &out);

