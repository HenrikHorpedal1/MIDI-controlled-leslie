// motor.cpp
#include "motor.h"
#include <Arduino.h>
#include <math.h>

namespace {

constexpr int pwmPin     = 6;
constexpr int dirPin     = 5;

constexpr int freq       = 1000;  // Hz
constexpr int resolution = 14;    // bits (0..65535 on ESP32)

constexpr uint32_t MAX_DUTY = (1u << resolution) - 1u;

// Convert 0..100% -> 0..MAX_DUTY (rounded)
constexpr uint32_t pct_to_duty(int pct) {
  if (pct <= 0)   return 0u;
  if (pct >= 100) return MAX_DUTY;
  return (MAX_DUTY * static_cast<uint64_t>(pct) + 50) / 100u;
}

constexpr int MAX_INPUT_PWM =
    static_cast<int>(pct_to_duty(MOTOR_MAX_INPUT_PERCENT));

constexpr int DIR_CHANGE_DEADBAND_PERCENT = 5;
constexpr int DIR_CHANGE_DEADBAND_PWM =
    static_cast<int>(pct_to_duty(DIR_CHANGE_DEADBAND_PERCENT));

enum class Direction : int8_t {
  Forward = +1,
  Reverse = -1
};

Direction currentDirection = Direction::Forward;

void applyDirection(Direction dir) {
  currentDirection = dir;

  pinMode(dirPin, OUTPUT);

  if (dir == Direction::Forward) {
    digitalWrite(dirPin, HIGH);  // transistor ON -> DIR LOW
  } else {
    digitalWrite(dirPin, LOW);   // transistor OFF -> DIR HIGH (released)
  }
}

} // namespace

// =======================
// Public API
// =======================

void motorInit() {
  Serial.println("Initializing motor interface...");

  if (!ledcAttach(pwmPin, freq, resolution)) {
    Serial.println("ERROR: ledcAttach failed!");
  }

  applyDirection(Direction::Forward);
  ledcWrite(pwmPin, 0);   // duty = 0
}

void motorSetNormalized(float u) {
  // Clamp to [-1, 1]
  if (u >  1.0f) u =  1.0f;
  if (u < -1.0f) u = -1.0f;

  float mag = fabsf(u);
  int pwm = static_cast<int>(mag * static_cast<float>(MAX_INPUT_PWM) + 0.5f);

  int sign = 0;
  if (pwm > 0) {
    sign = (u > 0.0f) ? +1 : -1;
  }

  int absPwm = (pwm >= 0) ? pwm : -pwm;

  static int lastSign = 0;

  if (lastSign != 0 && sign != 0 && sign != lastSign && absPwm > DIR_CHANGE_DEADBAND_PWM) {
    Serial.println("motorSetNormalized: large sign flip, forcing zero before direction change");
    sign   = 0;
    absPwm = 0;
    pwm    = 0;
  }

  if (sign > 0 && currentDirection != Direction::Forward) {
    applyDirection(Direction::Forward);
  } else if (sign < 0 && currentDirection != Direction::Reverse) {
    applyDirection(Direction::Reverse);
  }

  uint32_t duty = static_cast<uint32_t>(absPwm);
  if (duty > static_cast<uint32_t>(MAX_INPUT_PWM)) {
    duty = static_cast<uint32_t>(MAX_INPUT_PWM);
  }

  ledcWrite(pwmPin, duty);

  lastSign = sign;
}

