#include <Arduino.h>
#include "quadrature-encoder.h"
#include "velocity.h"

void setup(){
  encoderInit();
  velocityInit();
  Serial.begin(115200);

}

VelocityState st;

void loop(){
  velocityUpdate();
  velocityGetState(st);
  Serial.print("ref:");
  Serial.print(0.0);
  Serial.print(",");
  Serial.print("RPM:");
  Serial.println(st.velRpm);
  delay(2);
}
