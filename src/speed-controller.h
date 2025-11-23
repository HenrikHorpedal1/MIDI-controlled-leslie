#pragma once
#include "ir-encoder.h"


void speedControllerTask(void *pvParameters);

struct GlobalVelocityReference{
    volatile uint64_t velocity_reference_x100;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

extern GlobalVelocityReference globalVelocityReference; //TODO: correct way???

struct GlobalShouldStop{
    volatile bool shouldStop = false;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

//this shoulc/could be defined a different place.
struct SpeedControllerParameters{
    GlobalVelocity* velocity;
    GlobalVelocityReference* reference;
    GlobalShouldStop* stop;
}; 

