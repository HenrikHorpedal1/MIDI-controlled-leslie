#pragma once
#include "ir-encoder.h"
#include "speed-controller.h" //to get GlobalShouldStop TODO: find better solution.

struct GlobalPositionReference{
    volatile uint64_t positionReference;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

struct PositionControllerParameters{
    GlobalVelocity* velocity; //should not have acess, TODO: fix.
    GlobalPosition* position;
    GlobalPositionReference* reference;
    GlobalShouldStop* stop;
}; 

extern GlobalPositionReference globalPositionReference;

void positionControllerTask(void *pvParameters);

