// controller.cpp
//
// Drives the two Leslie rotors (horn + drum) via moteus, at 100 Hz.
//
// Each loop:
//   1. read the operator reference + MIDI clock
//   2. nextState()  -> one shared ControlState for both rotors
//   3. dispatch the matching per-rotor command builder
//   4. updateRotorState() folds the moteus reply back into our estimate
//
//        reference + clock ─► nextState ─► ControlState
//                                              │
//                 ┌────────────────┬───────────┴───────────┐
//            sendVelocity    sendBeatSyncCam           sendPark
//                 └────────────────┴───────────┬───────────┘
//                                          moteus reply
//                                              │
//                                       updateRotorState
//
// The three states:
//   Velocity – follow the RPM reference, ramping at per-rotor accel limits.
//   BeatSync – follow the MIDI clock: rotor speed + phase locked to the beat.
//   Park     – reference is 0: coast down, then hold position on a mark.
//
// Both motors report in rotor/output units (the drum 0.25 output ratio is set
// in tview), so no gear ratio is applied in code.

#include "controller.h"
#include "beat_sync.h"
#include "clock_sync.h"
#include "moteus-config.h"
#include "reference.h"
#include "state_estimator.h"
#include "udp_telemetry.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <limits>
#include <math.h>

// ===== Tunables (rev, rev/s, rev/s^2) =======================================

// Controller loop period: 200 Hz.
static constexpr double CONTROL_DT_S = 0.005;

// Velocity mode: independent up/down ramps per rotor.
static constexpr double ACCEL_UP_HORN_REV_S2 = 3.50;   // 420 RPM in ~2 s
static constexpr double ACCEL_DOWN_HORN_REV_S2 = 2.50; // ~2.7 s down from tremolo
static constexpr double ACCEL_UP_DRUM_REV_S2 = 1.458;  // 350 RPM in ~4 s
static constexpr double ACCEL_DOWN_DRUM_REV_S2 = 2.50;

// BeatSync cam planner: spin-up accel limits, separated only for tuning. The
// planner hands moteus a moving phase target and lets its own trapezoidal
// planner reach it; see sendBeatSyncCam / planBeatCam.
static constexpr double BEAT_ACCEL_UP_REV_S2 = 3.50;
static constexpr double BEAT_ACCEL_DOWN_REV_S2 = 2.50;

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

// Park: gentle hold limits, and the speed below which a rotor is slow enough to
// snap onto a mark.
static constexpr double ACCEL_LIMIT_STOP_REV_S2 = 0.50;
static constexpr double VELOCITY_LIMIT_STOP = 0.5;
static constexpr double HORN_STOP_THRESH_REV_S = 0.333; // 20 RPM
static constexpr double DRUM_STOP_THRESH_REV_S = 0.333; // 20 RPM

// Load estimator (alpha-beta tracker on the motor->load slip offset):
//   alpha = position correction gain. tau ~= dt/alpha; 0.05 -> ~0.1 s, cutoff
//           ~1.6 Hz (attenuates tremolo eccentricity at ~7 Hz, passes chorale).
//   beta  = creep-rate correction gain. Benedict-Bordner pairing for alpha=0.05
//           is beta ~= alpha^2/(2-alpha) ~= 0.0013; small since creep is steady.
static constexpr double SLIP_FUSE_ALPHA = 0.05;
static constexpr double SLIP_TRACK_BETA = 0.0013;

static const double kNaN = std::numeric_limits<double>::quiet_NaN();

// ===== Per-rotor state ======================================================

// Rotor identity: selects the beat-sync tick count and labels debug output.
enum class Rotor { Horn, Drum };
static const char *rotorName(Rotor r) {
  return r == Rotor::Horn ? "horn" : "drum";
}

// Live runtime values only. Per-rotor *constants* (accel limits, stop
// threshold, name, isDrum) are passed to the command builders, not stored here.
struct RotorState {
  double lastPosRevs = 0.0;       // moteus output position (revs)
  double lastVelRevs = 0.0;       // |velocity| (rev/s)
  double lastSignedVelRevs = 0.0; // velocity with sign (rev/s)
  double lastRefRevs = 0.0;       // last commanded speed; sets ramp direction
  double filteredSlip = 0.0;      // fused output-vs-encoder1 offset = est.slip (revs)
  float enc1Pos = 0.0f;           // encoder 1 (rotor) fractional position
  float enc1Vel = 0.0f;

  // Fuses the motor-referred position/velocity with the MA600 angle into one
  // load estimate; est.slip replaces the old first-order slip low-pass.
  LoadEstimator est{};

  // Park
  double homeInteger = 0.0; // rotor revolution to hold on once stopped
  bool parked = false;      // homeInteger has been snapped this Park

  // BeatSync cam planner (one-shot plan computed on entry; see sendBeatSyncCam)
  bool camInit = false;      // false until the plan is computed on (re)entry
  bool camLocked = false;    // true once the spin-up ramp has reached tempo
  double camRevOffset = 0.0; // whole-rev offset of target face from beat face
  double camAccel = 0.0;     // planned accel for the monotonic ramp (rev/s^2)

  // BeatSync setpoint scoping (telemetry only). dbgFaceJumpPeak is the worst
  // |Δface - tempo step| seen since the last telemetry sample, so a stair-step
  // in facePos shows up even at the 40 Hz telemetry rate; a smooth setpoint
  // leaves it near zero. dbgCmdPos is the last position handed to moteus.
  double dbgCmdPos = 0.0;       // last commanded position (revs)
  double dbgCmdVel = 0.0;       // last commanded velocity feedforward (rev/s)
  double dbgPhaseErrDeg = 0.0;  // rotor face vs beat target, wrapped (mech deg)
  double dbgPrevFace = 0.0;     // previous raw beat face, for the derivative
  double dbgFaceJumpPeak = 0.0; // peak face step error since last telem (revs)
};

// ===== Load state estimation ================================================

// Fold the latest moteus reply into the rotor estimate, fusing the motor-referred
// output position/velocity with the absolute MA600 angle (encoder 1) into one
// load estimate. The belt slip the rest of the controller subtracts from its
// position commands falls out as est.slip.
static void updateRotorState(Moteus &moteus, RotorState &st) {
  const auto &v = moteus.last_result().values;
  st.lastPosRevs = v.position;
  st.lastSignedVelRevs = v.velocity;
  st.lastVelRevs = fabs(v.velocity);
  st.enc1Pos = v.extra[0].value;
  st.enc1Vel = v.extra[1].value;
  loadEstimatorUpdate(st.est, v.position, v.velocity, st.enc1Pos, CONTROL_DT_S);
  st.filteredSlip = st.est.slip;
}

// ===== Command builders (one per control state) =============================
// Each returns true if the moteus replied.

// Velocity: follow the RPM reference, ramping at the rotor's per-direction limit.
static bool sendVelocity(Moteus &moteus, RotorState &st, double targetRPM,
                         double accelUp, double accelDown) {
  const double targetRevS = targetRPM / 60.0;
  mm::PositionMode::Command cmd;
  cmd.position = kNaN;
  cmd.velocity = targetRevS;
  cmd.accel_limit = (targetRevS >= st.lastRefRevs) ? accelUp : accelDown;
  cmd.velocity_limit = kNaN;
  st.lastRefRevs = targetRevS;
  return moteus.SetPosition(cmd, &lesliePositionFmt(), &leslieQueryFmt());
}

// Fractional part in [0, 1).
static double wrap01(double x) { return x - floor(x); }

// One-shot cam plan, computed once on (re)entry. Picks two things:
//   camRevOffset (m) – which whole-rev branch of the beat face to aim at, and
//   camAccel     (a) – the accel that makes a single v0->vf ramp land on it.
//
// Phase is periodic, so the rotor may "drop back" any whole number of revs; that
// integer is the slack that lets a *monotonic* velocity ramp arrive at tempo
// with the phase already correct. The lag area A = ∫(vf - v) dt of a constant-a
// ramp is dv^2/(2a); it must sit on the beat's rev grid (A ≡ frac mod 1) and be
// at least the fastest-ramp area (a = accelMax), so we pick the nearest feasible
// branch and back out a <= accelMax. The target face track then starts
// `signedArea` behind the current face, which moteus reads as a dx exactly on
// its decel curve — so its own planner ramps monotonically and lands on phase.
static void planBeatCam(RotorState &st, const BeatTarget &target,
                        double accelMax) {
  const double v0 = st.lastSignedVelRevs;
  const double vf = target.velRevS;
  const double p0Face = st.lastPosRevs + st.filteredSlip; // current face pos
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
  st.camAccel = a;
  // p0Face - signedArea lands on the beat grid; its offset from the beat face is
  // an exact integer (round() only sheds float error).
  st.camRevOffset = round((p0Face - signedArea) - target.facePos);
  st.camLocked = false;
  st.camInit = true;
}

// BeatSync via moteus's own planner. On entry we plan (camRevOffset, camAccel),
// then every cycle hand moteus an absolute face target that advances with the
// beat plus the chosen whole-rev offset, with the planned accel during spin-up.
// Once tempo is reached we relax to the full accel limit and a little cruise
// headroom so small phase disturbances re-trim quickly.
static bool sendBeatSyncCam(Moteus &moteus, RotorState &st, Rotor rotor,
                            double accelUp, double accelDown) {
  const BeatTarget target =
      (rotor == Rotor::Horn) ? beatSyncHornTarget() : beatSyncDrumTarget();

  if (!st.camInit) {
    const double accelMax =
        (target.velRevS >= st.lastSignedVelRevs) ? accelUp : accelDown;
    planBeatCam(st, target, accelMax);
    // Seed the scoping derivative one tempo-step back so the first sample after
    // (re)entry reads ~0 jump instead of a false spike.
    st.dbgPrevFace = target.facePos - target.velRevS * CONTROL_DT_S;
    Serial.printf("[cam] %s plan: v0=%.2f vf=%.2f m=%.0f accel=%.2f frac=%.3f\n",
                  rotorName(rotor), st.lastSignedVelRevs, target.velRevS,
                  st.camRevOffset, st.camAccel,
                  wrap01(st.lastPosRevs + st.filteredSlip - target.facePos));
  }

  if (!st.camLocked &&
      fabs(st.lastSignedVelRevs - target.velRevS) < BEAT_CAM_LOCK_VEL_REVS) {
    st.camLocked = true;
    Serial.printf("[cam] %s locked at %.2f rev/s\n", rotorName(rotor),
                  st.lastSignedVelRevs);
  }

  // Moving face target = live beat face + the chosen whole-rev offset.
  const double targetFace = target.facePos + st.camRevOffset;
  // Spin-up: the planned ramp accel. Locked: a gentle trim accel so per-tick
  // phase corrections don't blip the velocity feedforward.
  const double accel = st.camLocked ? BEAT_CAM_TRIM_ACCEL_REV_S2 : st.camAccel;

  mm::PositionMode::Command cmd;
  cmd.position = targetFace - st.filteredSlip; // encoder face lands on targetFace
  cmd.velocity = target.velRevS;               // terminal velocity = tempo
  cmd.accel_limit = accel;
  cmd.velocity_limit = target.velRevS * (1.0 + BEAT_CAM_VEL_MARGIN);
  st.lastRefRevs = target.velRevS;

  // Scoping: how far this tick's face step departs from a smooth tempo step.
  const double faceStep = target.facePos - st.dbgPrevFace;
  const double jump = fabs(faceStep - target.velRevS * CONTROL_DT_S);
  if (jump > st.dbgFaceJumpPeak)
    st.dbgFaceJumpPeak = jump;
  st.dbgPrevFace = target.facePos;
  st.dbgCmdPos = cmd.position;
  st.dbgCmdVel = cmd.velocity;
  // Phase error: actual rotor face minus the beat's commanded face, wrapped to
  // the nearest revolution and converted to mechanical degrees (±180).
  const double errRev = (st.lastPosRevs + st.filteredSlip) - targetFace;
  st.dbgPhaseErrDeg = (errRev - round(errRev)) * 360.0;

  return moteus.SetPosition(cmd, &lesliePositionFmt(), &leslieQueryFmt());
}

// Park: reference is 0. While still spinning, ramp down like a Velocity(0)
// request; once below the stop threshold, freeze a rotor revolution to hold and
// keep applying the live slip so the encoder lands on the mark.
static bool sendPark(Moteus &moteus, RotorState &st, double stopThresh,
                     double accelDown, Rotor rotor) {
  // Always decelerate at accelDown: pass it for both directions, otherwise once
  // sendVelocity sets lastRefRevs=0 the (0 >= 0) test would pick accelUp and
  // freeze accel_limit at 0 — moteus would coast instead of braking to a stop.
  if (!st.parked && st.lastVelRevs >= stopThresh)
    return sendVelocity(moteus, st, 0.0, /*accelUp=*/accelDown, accelDown);

  if (!st.parked) {
    st.homeInteger =
        (st.lastRefRevs >= 0.0) ? ceil(st.lastPosRevs) : floor(st.lastPosRevs);
    st.parked = true;
    Serial.printf("[pos snap] %s pos=%.3f ref=%.3f homeInt=%.0f slip=%.4f\n",
                  rotorName(rotor), st.lastPosRevs, st.lastRefRevs,
                  st.homeInteger, st.filteredSlip);
  }

  mm::PositionMode::Command cmd;
  cmd.position = st.homeInteger - st.filteredSlip;
  cmd.velocity = 0.0;
  cmd.accel_limit = ACCEL_LIMIT_STOP_REV_S2;
  cmd.velocity_limit = VELOCITY_LIMIT_STOP;
  return moteus.SetPosition(cmd, &lesliePositionFmt(), &leslieQueryFmt());
}

// ===== State machine ========================================================

enum class ControlState { Velocity, Park, BeatSync };

// One shared state for both rotors, decided from the reference and the clock.
// hornRPM and drumRPM are always set together by the input handler (both zero
// or both non-zero), so one reference==0 test governs both rotors.
static ControlState nextState(const Reference &ref) {
  // In BeatSync mode the clock governs everything — there is no manual RPM
  // target — so the clock decides the state: follow it when running and locked,
  // otherwise Park. (The RPM reference is meaningless here and would otherwise
  // leak through as Velocity when the song stops, leaving the rotors spinning.)
  if (ref.mode == DriveMode::BeatSync) {
    return (clockSyncIsRunning() && clockSyncIsLocked()) ? ControlState::BeatSync
                                                         : ControlState::Park;
  }
  // Other sources track the RPM reference directly.
  if (ref.hornRPM == 0.0f && ref.drumRPM == 0.0f)
    return ControlState::Park;
  return ControlState::Velocity;
}

static const char *controlStateName(ControlState s) {
  return (s == ControlState::Velocity)   ? "Velocity"
         : (s == ControlState::BeatSync) ? "BeatSync"
                                         : "Park";
}

// ===== Task =================================================================

void controllerTask(void *pvParameters) {
  if (!configureMoteus(Serial)) {
    Serial.println("Moteus init failed — controller task exiting");
    vTaskDelete(nullptr);
    return;
  }

  RotorState horn;
  RotorState drum;

  loadEstimatorInit(horn.est, SLIP_FUSE_ALPHA, SLIP_TRACK_BETA);
  loadEstimatorInit(drum.est, SLIP_FUSE_ALPHA, SLIP_TRACK_BETA);

  // Prime each rotor's estimate from a real reply BEFORE the first command.
  // Otherwise the default lastPosRevs=0 makes the first Park snap homeInteger to
  // 0 and slam a stopped rotor back to position 0 (seen as a slow spin-down to 0
  // at the park speed limit). SetStop holds position while it fetches the query.
  if (hornMoteus().SetStop(&leslieQueryFmt()))
    updateRotorState(hornMoteus(), horn);
  if (drumMoteus().SetStop(&leslieQueryFmt()))
    updateRotorState(drumMoteus(), drum);

  ControlState state = ControlState::Velocity;
  Subdivision beatSubdiv = beatSyncGetSubdivision();

  for (;;) {
    Reference ref;
    referenceGet(ref);

    const ControlState next = nextState(ref);
    if (next != state) {
      // Reset the per-state entry latches for both rotors.
      if (next == ControlState::BeatSync)
        horn.camInit = drum.camInit = false; // re-plan the cam on entry
      else if (next == ControlState::Park)
        horn.parked = drum.parked = false; // a fresh stop re-snaps the mark
      state = next;
    }

    // A subdivision change retargets the rotors, so re-plan the cam (a new tpc
    // means a new speed/phase target) while we're locked to the beat.
    const Subdivision sd = beatSyncGetSubdivision();
    if (sd != beatSubdiv) {
      beatSubdiv = sd;
      if (state == ControlState::BeatSync)
        horn.camInit = drum.camInit = false;
    }

    bool hornReplied = false;
    bool drumReplied = false;
    switch (state) {
    case ControlState::Velocity:
      hornReplied = sendVelocity(hornMoteus(), horn, ref.hornRPM,
                                 ACCEL_UP_HORN_REV_S2, ACCEL_DOWN_HORN_REV_S2);
      drumReplied = sendVelocity(drumMoteus(), drum, ref.drumRPM,
                                 ACCEL_UP_DRUM_REV_S2, ACCEL_DOWN_DRUM_REV_S2);
      break;
    case ControlState::BeatSync:
      hornReplied = sendBeatSyncCam(hornMoteus(), horn, Rotor::Horn,
                                    BEAT_ACCEL_UP_REV_S2, BEAT_ACCEL_DOWN_REV_S2);
      drumReplied = sendBeatSyncCam(drumMoteus(), drum, Rotor::Drum,
                                    BEAT_ACCEL_UP_REV_S2, BEAT_ACCEL_DOWN_REV_S2);
      break;
    case ControlState::Park:
      hornReplied = sendPark(hornMoteus(), horn, HORN_STOP_THRESH_REV_S,
                             ACCEL_DOWN_HORN_REV_S2, Rotor::Horn);
      drumReplied = sendPark(drumMoteus(), drum, DRUM_STOP_THRESH_REV_S,
                             ACCEL_DOWN_DRUM_REV_S2, Rotor::Drum);
      break;
    }

    if (hornReplied)
      updateRotorState(hornMoteus(), horn);
    if (drumReplied)
      updateRotorState(drumMoteus(), drum);

    // --- Telemetry at 40 Hz (every 5 ticks at 200 Hz) ---
    static uint8_t s_telemDiv = 0;
    if (++s_telemDiv >= 5) {
      s_telemDiv = 0;
      Telemetry.send(
          "horn_rpm",    (float)(horn.lastVelRevs * 60.0),
          "horn_ref_rpm",(float)(ref.hornRPM),
          "drum_rpm",    (float)(drum.lastVelRevs * 60.0),
          "drum_ref_rpm",(float)(ref.drumRPM));
      // Estimator tracking. Both wrapped to (-0.5, 0.5] so they overlay:
      //   *_slip     = tracked slip offset (the controller's correction), smooth.
      //   *_slip_raw = raw measured offset wrapRev(ma600 - motorPos), noisy.
      // *_slip should ride the mean of *_slip_raw, rejecting the 1x-rev eccentricity.
      auto wrapHalf = [](double x) { return x - round(x); };
      Telemetry.send(
          "horn_slip",     (float)wrapHalf(horn.filteredSlip),
          "horn_slip_raw", (float)wrapHalf(horn.enc1Pos - wrap01(horn.lastPosRevs)),
          "drum_slip",     (float)wrapHalf(drum.filteredSlip),
          "drum_slip_raw", (float)wrapHalf(drum.enc1Pos - wrap01(drum.lastPosRevs)));
      // Tracked creep rate (rev/s). Should settle to a small steady value at a
      // given speed; if *_slip drifts, watch whether *_slip_vel has converged.
      Telemetry.send(
          "horn_slip_vel",(float)horn.est.slipVel,
          "drum_slip_vel",(float)drum.est.slipVel);
      // BeatSync setpoint scoping: *_face_jump is the worst face step error
      // since the last sample (≈0 if facePos is smooth; spikes = staircase);
      // *_cmd_pos is the absolute position handed to moteus.
      Telemetry.send(
          "horn_face_jump",(float)horn.dbgFaceJumpPeak,
          "drum_face_jump",(float)drum.dbgFaceJumpPeak,
          "horn_cmd_pos",  (float)horn.dbgCmdPos,
          "drum_cmd_pos",  (float)drum.dbgCmdPos);
      horn.dbgFaceJumpPeak = drum.dbgFaceJumpPeak = 0.0;
      // PLL health + commanded velocity feedforward: if cmd_vel and bpm are flat
      // but horn_rpm/drum_rpm jitter, the ripple is in moteus tracking, not the
      // setpoint. pll_err_us shows the MIDI jitter driving the phase correction.
      Telemetry.send(
          "horn_cmd_vel",(float)horn.dbgCmdVel,
          "drum_cmd_vel",(float)drum.dbgCmdVel,
          "pll_err_us",  (float)clockSyncGetLastErrUs(),
          "bpm",         (float)clockSyncGetBpm());
      // Clock + state machine: on pause, watch whether clock_running drops to 0
      // and ctrl_state goes to 1 (Park). If clock_running stays 1, the DAW keeps
      // streaming clock while paused — the issue is the clock, not the controller.
      // ctrl_state: 0=Velocity 1=Park 2=BeatSync.
      Telemetry.send(
          "clock_running",(float)(clockSyncIsRunning() ? 1 : 0),
          "clock_locked", (float)(clockSyncIsLocked() ? 1 : 0),
          "ctrl_state",   (float)(int)state,
          "drive_mode",   (float)(int)ref.mode);
      // BeatSync phase error in mechanical degrees (rotor face vs beat target,
      // ±180). Only meaningful in BeatSync; stale otherwise.
      Telemetry.send(
          "horn_err_deg",(float)horn.dbgPhaseErrDeg,
          "drum_err_deg",(float)drum.dbgPhaseErrDeg,
          // Fractional rotor angle [0,1): if horn_err_deg dips line up with the
          // same horn_pos_frac value every rev, the ripple is angle-locked
          // (encoder/mechanical), not beat-locked.
          "horn_pos_frac",(float)(horn.lastPosRevs - floor(horn.lastPosRevs)),
          "drum_pos_frac",(float)(drum.lastPosRevs - floor(drum.lastPosRevs)));
      // Output encoders + fused load estimate (state_estimator):
      //   *_enc0_frac = onboard output position (the control source),
      //   *_enc1_frac = MA600 (true load, sources[1]),
      //   *_est_frac  = fused load position; tracks enc1 but smoothed.
      // enc1-enc0 is the live slip; est_frac is enc0 + est.slip.
      Telemetry.send(
          "horn_enc0_frac",(float)wrap01(horn.lastPosRevs),
          "horn_enc1_frac",(float)horn.enc1Pos,
          "horn_est_frac", (float)wrap01(horn.est.posL));
      Telemetry.send(
          "drum_enc0_frac",(float)wrap01(drum.lastPosRevs),
          "drum_enc1_frac",(float)drum.enc1Pos,
          "drum_est_frac", (float)wrap01(drum.est.posL));
      // Velocities: motor_vel is the raw motor encoder; fused_vel = motor_vel +
      // slipVel is the estimated load velocity. Once the tracker settles, fused_vel
      // should match enc1_vel (the noisy MA600 velocity) in mean.
      Telemetry.send(
          "horn_motor_vel", (float)horn.lastSignedVelRevs,
          "horn_fused_vel", (float)horn.est.velL,
          "horn_enc1_vel",  (float)horn.enc1Vel);
      Telemetry.send(
          "drum_motor_vel", (float)drum.lastSignedVelRevs,
          "drum_fused_vel", (float)drum.est.velL,
          "drum_enc1_vel",  (float)drum.enc1Vel);
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 200 Hz
  }
}
