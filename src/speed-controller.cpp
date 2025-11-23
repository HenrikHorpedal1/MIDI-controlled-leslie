#include "speed-controller.h"
#include "ir-encoder.h"
#include "arduino.h"
#include "utils.h"
#include "config.h"
#include "SD-logger.h"

VelocityPID velPID;

GlobalVelocityReference globalVelocityReference;
GlobalShouldStop globalShouldStop;

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
  uint64_t lastTargetRPMx100;
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
        //Serial.println("measurement: " + String(measuredRPMx100) + "reference: " + String(targetRPMx100) + "error: " + String(errorRPMx100));
        P = velPID.Kp*(float)errorRPMx100;

        //for now, reset Integration-term when the reference is changed. TODO: think of better solution to mitigate windup.
        if (targetRPMx100 != lastTargetRPMx100) {
            I = 0.0f;
        }
        else{
            I_increment = (float)errorRPMx100 * velPID.Ki_per_s * (float)dt;
            I += I_increment;
        }
        if (I < 0.0){
                I= 0.0;}

        lastInputMs = nowMs;
        lastTargetRPMx100 = targetRPMx100;
        u = P + I;
      }
      pwmCmd = clamp_int(u, 0, MAX_INPUT_PWM);
      ledcWrite(pwmPin, pwmCmd);
      //Serial.println(pwmCmd);
      //plot to seral plotter:
      unsigned long t_ms = millis();
      Serial.print("Time:");
      Serial.print(t_ms); 
      Serial.print(",");
      Serial.print("Measurement:");
      Serial.print(measuredRPMx100); 
      Serial.print(",");
      Serial.print("Integration-term:");
      Serial.print(I); 
      Serial.print(",");
      Serial.print("reference:");
      Serial.print(targetRPMx100);
      Serial.print(",");
      Serial.print("input:");
      Serial.println(u);
      
      // --- CSV logging (numeric only) ---
      String line;
      line.reserve(80); // avoid reallocations

      line += String(t_ms);
      line += ',';
      line += String((long)measuredRPMx100);
      line += ',';
      line += String((long)targetRPMx100);
      line += ',';
      line += String((double)P, 6);
      line += ',';
      line += String((double)I, 6);
      line += ',';
      line += String((double)u, 6);
      line += ',';
      line += String(pwmCmd);

      logLine(line);
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}
