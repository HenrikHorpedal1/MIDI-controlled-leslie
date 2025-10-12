#include "speed-controller.h"
#include "ir-encoder.h"
#include "arduino.h"

//#define TARGET_SLOW_RPM_X100 4000          // 40.00 RPM
//#define TARGET_SLOW_RPM_X100 3000          // 300.00 RPM for now

// Slow-mode input guardrails

//these could be converted to fractions of reference rpm
//const int   MAX_SLOW_INPUT_PERCENT       = 30;
//const int   MIN_SLOW_INPUT_PERCENT       = 0;
//const int   SLOW_OPEN_LOOP_INPUT_PERCENT = 15;

// Fast-mode input guiderails
//const int   MAX_FAST_INPUT_PERCENT       = 60;
//const int   MIN_FAST_INPUT_PERCENT       = 0;
//
const int MAX_INPUT_PERCENT = 60;

GlobalVelocityReference globalVelocityReference;

// ================= MOTOR PWM & DIRECTION =================
PID pid;
const uint32_t MAX_DUTY = (1u << resolution) - 1;

GlobalShouldStop globalShouldStop;

inline uint32_t pct_to_duty(int pct) { if (pct <= 0)  return 0;
  if (pct >= 100) return MAX_DUTY;
  // rounded conversion
  return (uint32_t)((MAX_DUTY * (uint64_t)pct + 50) / 100);
}

const int MAX_INPUT_PWM = pct_to_duty(MAX_INPUT_PERCENT);


int clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

void toggleDirection() {
  static bool directionState = false;
  directionState = !directionState;

  if (directionState) {
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);
    Serial.println("Direction: LOW (Reversed?)");
  } else {
    pinMode(dirPin, INPUT_PULLUP);
    Serial.println("Direction: HIGH / Floated (Forward?)");
  }
}

void speedControllerTask(void *pvParameters){
  //assume it gets a pointer to rpm target, rpm measurement, stop command
  SpeedControllerParameters* controllerParameters = static_cast<SpeedControllerParameters*>(pvParameters);

  uint64_t targetRPMx100;
  int64_t errorRPMx100;
  uint64_t measuredRPMx100;
  bool stop;
  int pwmCmd;
  float I= 0.0f;
  uint32_t nowMs; 
  uint32_t lastInputMs = millis();
  float dt;
  float P;
  float I_increment = 0.0f;
  float u = 0.0f;

  for (;;){

      //Serial.println(stop);
      //extract variables safely 
      portENTER_CRITICAL(&controllerParameters->velocity->lock);
      measuredRPMx100 = controllerParameters->velocity->velocity_rpm_x100;
      portEXIT_CRITICAL(&controllerParameters->velocity->lock);

      portENTER_CRITICAL(&controllerParameters->reference->lock);
      targetRPMx100 = controllerParameters->reference->velocity_reference_x100;
      portEXIT_CRITICAL(&controllerParameters->reference->lock);

      portENTER_CRITICAL(&controllerParameters->stop->lock);
      stop = controllerParameters->stop->shouldStop;
      portEXIT_CRITICAL(&controllerParameters->stop->lock);

      if (stop) {
        u = 0.0f;
        I = 0.0f;
        lastInputMs = millis();
      } else {

        nowMs = millis();
        dt = (nowMs - lastInputMs)/1000.0f;
        errorRPMx100 = (int32_t)targetRPMx100 - (int32_t)measuredRPMx100;
        Serial.println("measurement: " + String(measuredRPMx100) + "reference: " + String(targetRPMx100) + "error: " + String(errorRPMx100));
        P = pid.Kp*(float)errorRPMx100;
        I_increment = (float)errorRPMx100 * pid.Ki_per_s * (float)dt;
        lastInputMs = nowMs;
        I += I_increment;
        u = P + I;
      }
      pwmCmd = clamp_int(u, 0, MAX_INPUT_PWM);
      ledcWrite(pwmPin, pwmCmd);
      //Serial.println(pwmCmd);
  
      vTaskDelay(pdMS_TO_TICKS(5));
  }
}
