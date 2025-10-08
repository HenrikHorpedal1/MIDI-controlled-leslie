#include <Arduino.h>
#include "ir-encoder.h"

extern IrSensor InnerSensor;
extern IrSensor OuterSensor;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(InnerSensor.pin, INPUT_PULLUP);
  pinMode(OuterSensor.pin, INPUT_PULLUP);

  attachInterruptArg(digitalPinToInterrupt(InnerSensor.pin), onIrEdge, (void*)&InnerSensor, FALLING);
  attachInterruptArg(digitalPinToInterrupt(OuterSensor.pin), onIrEdge, (void*)&OuterSensor, FALLING);

  xTaskCreatePinnedToCore(IrSensorTask, "IR_INNER", 4096, (void*)&InnerSensor, 1, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(IrSensorTask, "IR_OUTER", 4096, (void*)&OuterSensor, 1, nullptr, APP_CPU_NUM);

  Serial.println("IR encoder tasks started.");
}

void loop() {
  // Nothing required here—work is done in ISR + tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

