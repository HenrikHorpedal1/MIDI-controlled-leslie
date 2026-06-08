#include <Arduino.h>
#include "quadrature-encoder.h"

void setup(){
  Serial.begin(115200);
  delay(50);

  encoderInit();
}

EncoderState st;

void loop(){
  getEncoderState(st);
  Serial.print("count: ");
  Serial.println(st.count);
  Serial.print("relative angle: ");
  Serial.println(st.relAngleDeg);
  Serial.print("absolute angle: ");
  Serial.println(st.absAngleDeg);
  Serial.print("homed: ");
  Serial.println(st.homed);
  delay(50);
}
