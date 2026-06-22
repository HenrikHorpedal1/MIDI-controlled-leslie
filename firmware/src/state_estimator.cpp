#include "state_estimator.h"
#include <math.h>

// Wrap to the nearest representation in (-0.5, 0.5].
static inline double wrapRev(double x) { return x - round(x); }

void loadEstimatorInit(LoadEstimator &e, double beltRatio, double alpha, double beta) {
  e.alpha = alpha;
  e.beta = beta;
  e.beltRatio = beltRatio;
  e.posL = 0.0;
  e.velL = 0.0;
  e.slip = 0.0;
  e.slipVel = 0.0;
  e.innov = 0.0;
  e.init = false;
}

void loadEstimatorUpdate(LoadEstimator &e, double motorPos, double motorVel,
                         double ma600Pos, double dt) {
  // Convert raw motor-frame values to load frame.
  const double loadPos = motorPos / e.beltRatio;
  const double loadVel = motorVel / e.beltRatio;

  if (!e.init) {
    e.slip = wrapRev(ma600Pos - loadPos);
    e.slipVel = 0.0;
    e.posL = loadPos + e.slip;
    e.velL = loadVel;
    e.init = true;
    return;
  }

  // Predict: advance slip at the tracked creep rate.
  e.slip += e.slipVel * dt;

  // Residual: both ma600Pos and loadPos are in load frame, so the offset is comparable.
  const double innov = wrapRev((ma600Pos - loadPos) - e.slip);
  e.innov = innov;  // expose slip-tracking error for scoping

  e.slip += e.alpha * innov;
  e.slipVel += (e.beta / dt) * innov;

  e.posL = loadPos + e.slip;
  e.velL = loadVel + e.slipVel;
}

double loadEstimatorFaceToMotor(const LoadEstimator &e, double facePos) {
  // Convert face (load) position to motor position: undo slip, then scale by i.
  return (facePos - e.slip) * e.beltRatio;
}
