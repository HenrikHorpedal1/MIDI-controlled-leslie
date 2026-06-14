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
  double alpha;   // position correction gain, in (0,1)
  double beta;    // velocity (creep-rate) correction gain
  double posL;    // fused load position = motorPos + slip (revs, continuous)
  double velL;    // fused load velocity = motorVel + slipVel (rev/s)
  double slip;    // loadPos - motorPos (revs); position offset for the controller
  double slipVel; // d(slip)/dt (rev/s); tracked creep rate
  bool init;
};

void loadEstimatorInit(LoadEstimator &e, double alpha, double beta);

// motorPos, motorVel : Moteus output position/velocity (rotor revs, rev/s).
// ma600Pos           : MA600 absolute angle (revs; wraps each revolution).
// dt                 : seconds since the previous update.
void loadEstimatorUpdate(LoadEstimator &e, double motorPos, double motorVel,
                         double ma600Pos, double dt);
