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

#endif 
