#include "Arduino.h"
#include "footswitch.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(50);
 
  xTaskCreatePinnedToCore(footSwitchTask, "SwTask", 2048, NULL, 1, NULL, 1);
}

void loop() {
  Serial.println("Switch A: " + String(switchA));
  Serial.println("Switch B: " + String(switchB));
  delay(1000);
}
