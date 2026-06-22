#pragma once

// Estimates the load position from the clean motor encoder and the noisy,
// wrapping MA600 load encoder, for belt-slip correction in BeatSync.
//
// The motor encoder is high-rate and low-noise but sits upstream of the belt;
// belt creep makes the load fall steadily behind it, so the offset
//   slip = loadPos - motorPos
// is not constant — it ramps at the creep rate (~ -s * motorVel). The MA600
// measures the true load angle directly but is noisy and wraps every rev.
//
// We track slip with a two-state constant-velocity (alpha-beta) filter: slip and
// its rate slipVel. The predictor advances slip at slipVel, so a steady creep
// ramp is followed with no steady-state lag; the MA600 residual corrects both
// states. Constant gains -> well-conditioned at every speed.
//
//   Predict:  slip    += slipVel * dt
//   Residual: innov    = wrapRev((ma600 - motorPos) - slip)   // wrapped to ±0.5
//   Correct:  slip    += alpha * innov
//             slipVel += (beta / dt) * innov
//
// alpha : position correction gain. Larger = faster tracking, more MA600 noise
//         (incl. eccentricity at 1x rotation) in slip. Sets the bandwidth.
// beta  : creep-rate correction gain. Small — the creep rate is near-constant per
//         operating point. Benedict-Bordner pairing is beta ~= alpha^2/(2-alpha).
struct LoadEstimator {
  double alpha;      // position correction gain, in (0,1)
  double beta;       // velocity (creep-rate) correction gain
  double beltRatio;  // motor revs per load rev (i); set at init
  double posL;       // fused load position (load revs, continuous)
  double velL;       // fused load velocity (load rev/s)
  double slip;       // loadPos - motorPos/i (load revs); position offset
  double slipVel;    // d(slip)/dt (load rev/s); tracked creep rate
  double innov;      // last innovation (wrapped MA600 residual, load revs): the
                     // slip-tracking error driving both corrections. For scoping.
  bool init;
};

// Benedict-Bordner optimal pairing: for a given position gain alpha, the
// velocity gain that minimizes transient + noise error is
//   beta = alpha^2 / (2 - alpha).
// Lets beta follow from the single tuning knob (alpha) instead of two.
constexpr double loadEstBenedictBordnerBeta(double alpha) {
  return alpha * alpha / (2.0 - alpha);
}

// Default tracker gains for the Leslie load:
//   alpha = 0.010 -> tau ~= dt/alpha ~= 0.5 s, cutoff ~0.3 Hz: rejects both
//           chorale (~0.7 Hz) and tremolo (~7 Hz) MA600 eccentricity.
//   beta  = Benedict-Bordner(alpha) ~= 5.0e-5 -> tracks the creep ramp so the
//           steady-state position lag (slipVel * tau) goes to zero. NOTE: the
//           MA600 eccentricity is asymmetric (especially horn) and can let beta
//           integrate a bias / 1x-rev ripple into slipVel at chorale; watch
//           slip/vel and slip/err. Set beta = 0.0 to disable if that recurs.
static constexpr double LOAD_EST_ALPHA = 0.010;
static constexpr double LOAD_EST_BETA  = loadEstBenedictBordnerBeta(LOAD_EST_ALPHA);

// beltRatio: motor revs per load rev (i). Pass HORN_BELT_RATIO / DRUM_BELT_RATIO.
void loadEstimatorInit(LoadEstimator &e, double beltRatio,
                       double alpha = LOAD_EST_ALPHA,
                       double beta = LOAD_EST_BETA);

// motorPos, motorVel : raw moteus position/velocity in MOTOR frame (motor revs, rev/s).
// ma600Pos           : MA600 absolute angle in LOAD frame (revs; wraps each revolution).
// dt                 : seconds since the previous update.
// Internally divides motor values by beltRatio so all stored state is in load frame.
void loadEstimatorUpdate(LoadEstimator &e, double motorPos, double motorVel,
                         double ma600Pos, double dt);

// Convert a face-frame (load) position command to a motor-frame command.
// Returns (facePos - slip) * beltRatio — the single motor<->load authority.
double loadEstimatorFaceToMotor(const LoadEstimator &e, double facePos);

// Convert a load-frame velocity to motor-frame velocity.
inline double loadEstimatorFaceVelToMotor(const LoadEstimator &e, double faceVel) {
  return faceVel * e.beltRatio;
}
