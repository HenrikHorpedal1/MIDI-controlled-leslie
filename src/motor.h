// motor.h
#pragma once

#include <Arduino.h>

constexpr int MOTOR_MAX_INPUT_PERCENT = 60;

void motorInit();

void motorSetNormalized(float u);
