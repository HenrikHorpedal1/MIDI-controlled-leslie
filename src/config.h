#include "utils.h" //bad practise??? 
//TODO: fill with stuff
//TODO: name better
const int pwmPin     = 27;
const int dirPin     = 25;
const int freq       = 1000;
const int resolution = 16;
const uint32_t MAX_DUTY = (1u << resolution) - 1;
const int MAX_INPUT_PERCENT = 60;

constexpr uint32_t pct_to_duty(int pct) {
  if (pct <= 0)   return 0u;
  if (pct >= 100) return MAX_DUTY;
  return (MAX_DUTY * static_cast<uint64_t>(pct) + 50) / 100u;
}

constexpr int MAX_INPUT_PWM     = static_cast<int>(pct_to_duty(MAX_INPUT_PERCENT));

struct VelocityPID {
  const float Kp = 5;
  const float Ki_per_s = 1.5;
  const float Kd = 0.0f; // unused so far.
};

struct PositionPID {
    const float Kp = 100.0;
    const float Ki_per_s = 0.0;
    const float Kd = 0.001f;
};

const int MAX_POS_INPUT_PERCENT = 20;
constexpr int MAX_POS_INPUT_PWM     = static_cast<int>(pct_to_duty(MAX_POS_INPUT_PERCENT));
