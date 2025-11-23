#include <Arduino.h>
#include "footswitch.h"
#include "ir-encoder.h"
#include "speed-controller.h"
#include "position-controller.h"
#include "config.h"
#include "SD-logger.h"

extern volatile bool switchA;
extern volatile bool switchB;

extern IrSensor InnerSensor;
extern IrSensor OuterSensor;

extern GlobalPosition globalPosition;
extern GlobalVelocity globalVelocity;
extern GlobalVelocityReference globalVelocityReference;
extern GlobalPositionReference globalPositionReference;
extern GlobalShouldStop globalShouldStop;

SpeedControllerParameters speedControllerParameters{
    &globalVelocity,
    &globalVelocityReference,
    &globalShouldStop 
};

PositionControllerParameters positionControllerParameters{
    &globalVelocity,
    &globalPosition,
    &globalPositionReference,
    &globalShouldStop 
};

extern const int pwmPin;
extern const int dirPin;

extern const int freq;
extern const int resolution;

void setup() {

  Serial.begin(115200);
  delay(50);

  Serial.println("t_ms,meas_rpm_x100,ref_rpm_x100,P,I,u,pwm");
 
  //footswitch
  xTaskCreatePinnedToCore(footSwitchTask, "SwTask", 2048, NULL, 1, NULL, 1);
  //Serial.println("Foot switch task started.");

  //ir-sensor
  pinMode(InnerSensor.pin, INPUT_PULLUP);
  pinMode(OuterSensor.pin, INPUT_PULLUP);
  attachInterruptArg(digitalPinToInterrupt(InnerSensor.pin), onIrEdge, (void*)&InnerSensor, FALLING);
  attachInterruptArg(digitalPinToInterrupt(OuterSensor.pin), onIrEdge, (void*)&OuterSensor, FALLING);
  //xTaskCreatePinnedToCore(IrSensorTask, "IR_INNER", 4096, (void*)&InnerSensor, 1, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(IrSensorTask, "IR_OUTER", 4096, (void*)&OuterSensor, 1, nullptr, APP_CPU_NUM);
  //Serial.println("IR encoder tasks started.");

  //speedcontroller
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);  // default reversed direction
  ledcAttach(pwmPin, freq, resolution);
  ledcWrite(pwmPin, 0);
  xTaskCreatePinnedToCore(speedControllerTask, "SPEED_CONTROLLER", 4096, (void*)&speedControllerParameters, 1, nullptr, APP_CPU_NUM);

  //positioncontroller

  xTaskCreatePinnedToCore(positionControllerTask, "POSITION_CONTROLLER", 4096, (void*)&positionControllerParameters, 1, nullptr, APP_CPU_NUM);
  setupLogger();
}


void loop() {
    //if switchA: false-> true: move
    //if switchB: true-> false: break, stop at correct angle, break, hold.
    //if switchB true: fast, reference 300
    //    else switchB false: slow, reference 40.
    if (switchA){
        portENTER_CRITICAL(&speedControllerParameters.stop->lock);
        speedControllerParameters.stop->shouldStop = false;
        portEXIT_CRITICAL(&speedControllerParameters.stop->lock); 

        if (switchB){
            portENTER_CRITICAL(&speedControllerParameters.reference->lock);
            speedControllerParameters.reference->velocity_reference_x100 = 30000;
            portEXIT_CRITICAL(&speedControllerParameters.reference->lock);
            //Serial.println("reference: 200");

        } else if (!switchB){
            portENTER_CRITICAL(&speedControllerParameters.reference->lock);
            speedControllerParameters.reference->velocity_reference_x100 = 4000;
            portEXIT_CRITICAL(&speedControllerParameters.reference->lock);
            //Serial.println("reference: 40");
        }
    }
    else if (!switchA) {

            portENTER_CRITICAL(&speedControllerParameters.reference->lock);
            speedControllerParameters.reference->velocity_reference_x100 = 0;
            portEXIT_CRITICAL(&speedControllerParameters.reference->lock); 

            portENTER_CRITICAL(&speedControllerParameters.stop->lock);
            speedControllerParameters.stop->shouldStop = true;
            portEXIT_CRITICAL(&speedControllerParameters.stop->lock); 
            //Serial.println("reference: 0");

            portENTER_CRITICAL(&positionControllerParameters.reference->lock);
            positionControllerParameters.reference->positionReference = 0.0f;
            portEXIT_CRITICAL(&positionControllerParameters.reference->lock); 

            
    }
  delay(50);
}
