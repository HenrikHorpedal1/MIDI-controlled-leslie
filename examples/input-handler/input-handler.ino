#include "input_handler.h"

void setup(){
  Serial.begin(115200);
  delay(50);
  startInputHandler();
}

void loop(){
    vTaskDelay(portMAX_DELAY);
}
