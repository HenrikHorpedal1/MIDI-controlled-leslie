#include <Arduino.h>
#include "motor.h"

void setup(){
  Serial.begin(115200);
  delay(50);
  motorInit();

}


void loop(){
  delay(5000);

  motorSetNormalized(0.1);

  delay(5000);

  motorSetNormalized(0.2);
  delay(5000);

  motorSetNormalized(0.3);
  delay(5000);

  motorSetNormalized(0.0);
  delay(5000);
  motorSetNormalized(-0.2);
  delay(5000);
  motorSetNormalized(0.0);
}
