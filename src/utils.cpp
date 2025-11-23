#include "utils.h" 
#include <Arduino.h>

int clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

float smallestSignedAngle(float angleDeg){
    float wrapped = fmodf(angleDeg + 180.0, 360.0);
    if (wrapped < 0)
        wrapped += 360.0;
    return wrapped - 180.0;
}
