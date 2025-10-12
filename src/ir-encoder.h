// ir-encoder.h
#pragma once
#ifndef IR_ENCODER_H
#define IR_ENCODER_H

#include <Arduino.h>

struct IrSensor;

void IRAM_ATTR onIrEdge(void* pvParameter);
void IrSensorTask(void* pvParameter);
void updateDebounceTime(IrSensor* sensor, int32_t lastPeriodUs);

struct IrSensor {
  volatile uint32_t lastEdgeUs   = 0;
  volatile uint32_t prevEdgeUs   = 0;
  volatile uint32_t lastPeriodUs = 0;
  volatile uint32_t passCount    = 0;
  volatile bool     newestEdgeProcessed = true;
  volatile uint32_t debounceUs   = 0;

  int pin = -1;
  int const marksPerRev = -1;

  // forbid copying (locks should not be duplicated)
  IrSensor() = default;
  IrSensor(const IrSensor&) = delete;
  IrSensor& operator=(const IrSensor&) = delete;

  portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
  IrSensor(uint32_t debounceUs, int pin_, int mpr) : pin(pin_), marksPerRev(mpr) {}
};

struct FusedPosition{
    volatile uint32_t outerEdgesSinceInner = 0; // not a good variable name cuz goes from 0 to marks per rev -1
    volatile bool haveSeenInner = false; //during startup to find absolute position.
    volatile bool resetOnNext = false;

    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

//these are for other files to use:
struct GlobalPosition{ 
    volatile float position_deg = -1; //need think about initialization
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

struct GlobalVelocity{
    volatile uint64_t velocity_rpm_x100 = 0;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

#endif 
