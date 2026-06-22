// beat_cam_planner.h
//
// Plans the BeatSync rotor trajectory using moteus's own trapezoidal planner: on
// (re)entry it picks a whole-rev offset and a spin-up accel so a single monotonic
// velocity ramp lands on the beat with phase already correct; thereafter it hands
// moteus a moving face target that advances with the beat.
//
// The planner works PURELY in face (load) coordinates — it never sees belt slip.
// The caller reads the current face position from the estimator (e.posL), and
// converts the returned face-frame command back to a motor command with
// loadEstimatorFaceToMotor before SetPosition. This keeps slip in one place.
//
// Style matches LoadEstimator: a POD struct + free functions, no classes.
//
// The planner knows nothing about moteus or CAN — it speaks pure kinematics in the
// face (load) frame. The caller maps a BeatCamCommand onto its motor driver.
#pragma once

#include "beat_sync.h" // BeatTarget

// Live rotor motion this tick, in the load/face frame.
struct BeatCamInput {
  double faceRevs;      // current load (face) position, revs (= est.posL)
  double signedVelRevS; // load-frame velocity, rev/s (motor enc / i, not fused)
};

// A kinematic setpoint in the face frame (position) plus the planner's limits.
// The caller converts position to the motor frame and packs this into whatever
// command its driver uses.
struct BeatCamCommand {
  double facePos;       // target position, FACE frame (revs)
  double velRevS;       // velocity feedforward (rev/s)
  double accelLimit;    // rev/s^2
  double velocityLimit; // rev/s
};

struct BeatCamPlanner {
  // Config (set by beatCamInit).
  double accelUp = 0.0;   // spin-up accel ceiling (rev/s^2)
  double accelDown = 0.0; // spin-down accel ceiling (rev/s^2)
  double dt = 0.005;      // control period (s), for the scoping derivative

  // State.
  bool init = false;       // false until the plan is computed on (re)entry
  bool locked = false;     // true once the spin-up ramp has reached tempo
  double revOffset = 0.0;  // whole-rev offset of target face from beat face
  double accel = 0.0;      // planned accel for the monotonic spin-up ramp (rev/s^2)
  double prevFace = 0.0;   // previous raw beat face, for the jump derivative

  // One-shot transition flags. Set by beatCamStep, cleared by the caller (the
  // controller owns the rotor name, so it does the logging).
  bool justPlanned = false; // a fresh plan was computed this step
  bool justLocked = false;  // the spin-up ramp reached tempo this step

  // Scope (telemetry only; read by the controller).
  double dbgCmdPos = 0.0;       // last commanded face position (revs)
  double dbgCmdVel = 0.0;       // last commanded velocity feedforward (rev/s)
  double dbgPhaseErrDeg = 0.0;  // rotor face vs beat target, wrapped (mech deg)
  double dbgFaceJumpPeak = 0.0; // peak face step error since last telem (revs)
};

void beatCamInit(BeatCamPlanner &c, double accelUp, double accelDown, double dt);

// Force a re-plan on the next beatCamStep (call on BeatSync entry or a subdivision
// change). Does not touch the cruise/lock tuning, only the one-shot plan latch.
void beatCamReset(BeatCamPlanner &c);

// Compute this tick's setpoint. The returned facePos is in the face frame; the
// caller converts it to the motor frame (loadEstimatorFaceToMotor) before sending.
BeatCamCommand beatCamStep(BeatCamPlanner &c, const BeatTarget &target,
                           const BeatCamInput &in);
