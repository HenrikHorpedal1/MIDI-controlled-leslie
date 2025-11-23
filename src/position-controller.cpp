#include "speed-controller.h" //for now, find out where to place definition of ShouldStop;
#include "position-controller.h"
#include <cmath>
#include "config.h"
#include "utils.h"

GlobalPositionReference globalPositionReference;
PositionPID posPID;

void positionControllerTask(void *pvParameters){
  //assume it gets a pointer to angle target, angle measurement, stop command
  PositionControllerParameters* controllerParameters = static_cast<PositionControllerParameters*>(pvParameters);

  bool stop;
  float targetAngle;
  float positionError;
  uint64_t measuredRPMx100;
  float measuredAngle;
  int pwmCmd;
  float P;
  float D;
  float u = 0.0f;

  for (;;) {
      //read safely
      portENTER_CRITICAL(&controllerParameters->velocity->lock);
      measuredRPMx100 = controllerParameters->velocity->velocity_rpm_x100;
      portEXIT_CRITICAL(&controllerParameters->velocity->lock);

      portENTER_CRITICAL(&controllerParameters->reference->lock);
      targetAngle = controllerParameters->reference->positionReference;
      portEXIT_CRITICAL(&controllerParameters->reference->lock);

      portENTER_CRITICAL(&controllerParameters->position->lock);
      measuredAngle = controllerParameters->position->position_deg;
      portEXIT_CRITICAL(&controllerParameters->position->lock);

      portENTER_CRITICAL(&controllerParameters->stop->lock);
      stop = controllerParameters->stop->shouldStop;
      portEXIT_CRITICAL(&controllerParameters->stop->lock);

      if (stop && (measuredRPMx100 == 0.0)){
          positionError = smallestSignedAngle(targetAngle - measuredAngle);
          P = positionError*posPID.Kp;
          D = measuredRPMx100*posPID.Kd;
          u = P + D;

          pwmCmd = clamp_int(u, 0, MAX_INPUT_PWM);

          ledcWrite(pwmPin, pwmCmd);

      }
      // Serial.print("Measurement:");
      // Serial.print(measuredAngle); 
      // Serial.print(",");
      // Serial.print("reference:");
      // Serial.print(targetAngle);
      // Serial.print(",");
      // Serial.print("input:");
      // Serial.println(u);


      vTaskDelay(pdMS_TO_TICKS(10));
  }

}
