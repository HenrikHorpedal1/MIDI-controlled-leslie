#include "state_estimator.h"
#include <math.h>

// Wrap to the nearest representation in (-0.5, 0.5].
static inline double wrapRev(double x) { return x - round(x); }

void loadEstimatorInit(LoadEstimator &e, double alpha, double beta) {
  e.alpha = alpha;
  e.beta = beta;
  e.posL = 0.0;
  e.velL = 0.0;
  e.slip = 0.0;
  e.slipVel = 0.0;
  e.init = false;
}

void loadEstimatorUpdate(LoadEstimator &e, double motorPos, double motorVel,
                         double ma600Pos, double dt) {
  if (!e.init) {
    // Seed slip from the first measured offset (wrapped, since the MA600 may be
    // mounted up to half a turn off) and start with zero creep rate.
    e.slip = wrapRev(ma600Pos - motorPos);
    e.slipVel = 0.0;
    e.posL = motorPos + e.slip;
    e.velL = motorVel;
    e.init = true;
    return;
  }

  // Predict: advance slip at the tracked creep rate.
  e.slip += e.slipVel * dt;

  // Residual between the measured offset and the prediction, wrapped to ±0.5 rev
  // (the MA600 only knows the offset modulo one revolution).
  const double innov = wrapRev((ma600Pos - motorPos) - e.slip);

  // Correct both states (alpha-beta). Constant gains: stable at every speed,
  // follows a steady creep ramp with no steady-state lag.
  e.slip += e.alpha * innov;
  e.slipVel += (e.beta / dt) * innov;

  e.posL = motorPos + e.slip;
  e.velL = motorVel + e.slipVel;
}
