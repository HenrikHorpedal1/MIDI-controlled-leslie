// feedforward.cpp
#include "feedforward.h"
#include <math.h>

static double ffCompute(double vRef, double aRef,
                        double J, double c, double b, double a,
                        bool withFriction) {
  double T = J * aRef;
  if (withFriction && fabs(vRef) > 1e-4)
    T += copysign(c + b * fabs(vRef) + a * vRef * vRef, vRef);
  return T;
}

double ffDrum(double vRef, double aRef, bool withFriction) {
  return ffCompute(vRef, aRef, DRUM_J, DRUM_C, DRUM_B, DRUM_A, withFriction);
}

double ffHorn(double vRef, double aRef, bool withFriction) {
  return ffCompute(vRef, aRef, HORN_J, HORN_C, HORN_B, HORN_A, withFriction);
}
