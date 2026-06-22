// beat_cam_planner.cpp
#include "beat_cam_planner.h"

#include <math.h>

// ===== Tunables (rev, rev/s, rev/s^2) =======================================

// |speed error| below which the spin-up ramp is considered locked.
static constexpr double BEAT_CAM_LOCK_VEL_REVS = 0.05; // ~3 RPM
// Cruise headroom above tempo, so small phase disturbances can be trimmed after
// lock without the velocity_limit pinning the rotor exactly at tempo.
static constexpr double BEAT_CAM_VEL_MARGIN = 0.02; // +2 %
// Floor on the planned ramp accel, so a near-zero speed change still gets a
// finite, gentle accel limit rather than 0.
static constexpr double BEAT_CAM_MIN_ACCEL = 0.05;
// After lock, phase is held by gentle trim only. The PLL re-anchors phase by a
// small step every MIDI tick; a low post-lock accel limit turns those steps into
// slow, smooth velocity nudges (a low-pass) instead of per-tick chases that
// jitter the velocity feedforward. Raise for snappier phase recovery, lower for
// a steadier cruise.
static constexpr double BEAT_CAM_TRIM_ACCEL_REV_S2 = 0.20;

// Fractional part in [0, 1).
static double wrap01(double x) { return x - floor(x); }

void beatCamInit(BeatCamPlanner &c, double accelUp, double accelDown, double dt) {
  c = BeatCamPlanner{};
  c.accelUp = accelUp;
  c.accelDown = accelDown;
  c.dt = dt;
}

void beatCamReset(BeatCamPlanner &c) { c.init = false; }

// One-shot cam plan, computed once on (re)entry. Picks two things:
//   revOffset (m) – which whole-rev branch of the beat face to aim at, and
//   accel     (a) – the accel that makes a single v0->vf ramp land on it.
//
// Phase is periodic, so the rotor may "drop back" any whole number of revs; that
// integer is the slack that lets a *monotonic* velocity ramp arrive at tempo
// with the phase already correct. The lag area A = ∫(vf - v) dt of a constant-a
// ramp is dv^2/(2a); it must sit on the beat's rev grid (A ≡ frac mod 1) and be
// at least the fastest-ramp area (a = accelMax), so we pick the nearest feasible
// branch and back out a <= accelMax. The target face track then starts
// `signedArea` behind the current face, which moteus reads as a dx exactly on
// its decel curve — so its own planner ramps monotonically and lands on phase.
static void planBeatCam(BeatCamPlanner &c, const BeatCamInput &in,
                        const BeatTarget &target, double accelMax) {
  const double v0 = in.signedVelRevS;
  const double vf = target.velRevS;
  const double p0Face = in.faceRevs; // current face position
  const double dv = vf - v0;

  const double frac = wrap01(p0Face - target.facePos); // where we sit, [0,1) rev
  const double minArea = (dv * dv) / (2.0 * accelMax);  // fastest-ramp deficit

  // Smallest |area| on the rev grid (area ≡ frac mod 1) with |area| >= minArea,
  // signed so the target sits behind on spin-up / ahead on spin-down.
  double signedArea;
  if (dv >= 0.0) {
    const double n = ceil(minArea - frac);
    signedArea = frac + (n > 0.0 ? n : 0.0);
  } else {
    const double k = ceil(minArea + frac);
    signedArea = frac - (k > 0.0 ? k : 0.0);
  }

  const double area = fabs(signedArea);
  double a = (area > 1e-6) ? (dv * dv) / (2.0 * area) : 0.0;
  if (a < BEAT_CAM_MIN_ACCEL)
    a = BEAT_CAM_MIN_ACCEL;
  c.accel = a;
  // p0Face - signedArea lands on the beat grid; its offset from the beat face is
  // an exact integer (round() only sheds float error).
  c.revOffset = round((p0Face - signedArea) - target.facePos);
  c.locked = false;
  c.init = true;
}

BeatCamCommand beatCamStep(BeatCamPlanner &c, const BeatTarget &target,
                           const BeatCamInput &in) {
  c.justPlanned = false;
  c.justLocked = false;

  if (!c.init) {
    const double accelMax =
        (target.velRevS >= in.signedVelRevS) ? c.accelUp : c.accelDown;
    planBeatCam(c, in, target, accelMax);
    // Seed the scoping derivative one tempo-step back so the first sample after
    // (re)entry reads ~0 jump instead of a false spike.
    c.prevFace = target.facePos - target.velRevS * c.dt;
    c.justPlanned = true;
  }

  if (!c.locked &&
      fabs(in.signedVelRevS - target.velRevS) < BEAT_CAM_LOCK_VEL_REVS) {
    c.locked = true;
    c.justLocked = true;
  }

  // Moving face target = live beat face + the chosen whole-rev offset.
  const double targetFace = target.facePos + c.revOffset;
  // Spin-up: the planned ramp accel. Locked: a gentle trim accel so per-tick
  // phase corrections don't blip the velocity feedforward.
  const double accel = c.locked ? BEAT_CAM_TRIM_ACCEL_REV_S2 : c.accel;

  BeatCamCommand cmd;
  cmd.facePos = targetFace;      // face frame; caller converts to motor
  cmd.velRevS = target.velRevS;  // terminal velocity = tempo
  cmd.accelLimit = accel;
  cmd.velocityLimit = target.velRevS * (1.0 + BEAT_CAM_VEL_MARGIN);

  // Scoping: how far this tick's face step departs from a smooth tempo step.
  const double faceStep = target.facePos - c.prevFace;
  const double jump = fabs(faceStep - target.velRevS * c.dt);
  if (jump > c.dbgFaceJumpPeak)
    c.dbgFaceJumpPeak = jump;
  c.prevFace = target.facePos;
  c.dbgCmdPos = cmd.facePos;
  c.dbgCmdVel = cmd.velRevS;
  // Phase error: actual rotor face minus the beat's commanded face, wrapped to
  // the nearest revolution and converted to mechanical degrees (±180).
  const double errRev = in.faceRevs - targetFace;
  c.dbgPhaseErrDeg = (errRev - round(errRev)) * 360.0;

  return cmd;
}
