#pragma once
#include "ir-encoder.h"

// struct PID {
//   const float Kp = 0.005f;
//   const float Ki_per_s = 0.003f;
//   const float Kd = 0.0f;
// };
struct PID {
  const float Kp = 3;
  const float Ki_per_s = 0.5;
  const float Kd = 0.0f;
};


const int pwmPin     = 27;

const int dirPin     = 25;

const int freq       = 1000;
const int resolution = 16;


struct GlobalVelocityReference{
    volatile uint64_t velocity_reference_x100;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

extern GlobalVelocityReference globalVelocityReference;


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


void speedControllerTask(void *pvParameters);


