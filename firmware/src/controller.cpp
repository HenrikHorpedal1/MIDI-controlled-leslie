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
// Both motors run at rotor_to_output_ratio=1 (motor frame). The belt ratio i
// is applied by the load estimator and the command builders; all tunables in
// this file are in load (Leslie rotor) frame.

#include "controller.h"
#include "beat_cam_planner.h"
#include "beat_sync.h"
#include "clock_sync.h"
#include "feedforward.h"
#include "leslie_config.h"
#include "moteus-config.h"
#include "reference.h"
#include "state_estimator.h"
#include "telemetry_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <limits>
#include <math.h>

// ===== Tunables (rev, rev/s, rev/s^2) =======================================
// RPM targets and acceleration limits live in leslie_config.h.

// Controller loop period: 200 Hz.
static constexpr double CONTROL_DT_S = 0.005;

// The load-estimator tracker gains (alpha/beta) live with the estimator; see
// LOAD_EST_ALPHA / LOAD_EST_BETA in state_estimator.h.

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
  double lastCmdVelRevS = 0.0;    // velocity sent to moteus this tick [motor rev/s]
  double ffVRef = 0.0;            // mirrored control_velocity for FF [rev/s]
  float enc1Pos = 0.0f;           // encoder 1 (rotor) fractional position
  float enc1Vel = 0.0f;

  // Position-controller test scoping: moteus' internal planner references and
  // the matching measurements, MOTOR frame (rev, rev/s, Nm) so ref/meas overlay
  // directly. See makeQueryFmt() in moteus-config.cpp.
  float ctrlPos = 0.0f;           // control_position  (0x038) planner ref
  float ctrlVel = 0.0f;           // control_velocity  (0x039) planner ref
  float ctrlTorque = 0.0f;        // control_torque    (0x03a) planner ref
  float measPosMotor = 0.0f;      // measured position (0x000) motor frame
  float measTorque = 0.0f;        // measured torque   (0x00a)
  float ffTorque = 0.0f;          // feedforward_torque we send to moteus [Nm]
  float cmdPosMotor = 0.0f;       // cmd.position the ESP sends to moteus, motor frame

  // Fuses the motor-referred position/velocity with the MA600 angle into one
  // load estimate; est.slip replaces the old first-order slip low-pass.
  LoadEstimator est{};

  // Park
  double homeInteger = 0.0; // face revolution to park on (snapped from face pos)
  bool parked = false;      // homeInteger has been snapped this Park

  // BeatSync trajectory planner. Owns the cam state and the setpoint scoping that
  // telemetry reads back (cam.dbg*); see beat_cam_planner.h.
  BeatCamPlanner cam{};
};

// ===== Load state estimation ================================================

// Fold the latest moteus reply into the rotor estimate, fusing the motor-referred
// output position/velocity with the absolute MA600 angle (encoder 1) into one
// load estimate. The belt slip the rest of the controller subtracts from its
// position commands falls out as est.slip.
static void updateRotorState(Moteus &moteus, RotorState &st) {
  const auto &v = moteus.last_result().values;
  const double i = st.est.beltRatio;
  st.lastPosRevs        = v.position / i;
  st.lastSignedVelRevs  = v.velocity / i;
  st.lastVelRevs        = fabs(v.velocity / i);
  st.ctrlPos    = v.extra[0].value;   // 0x038 control_position
  st.ctrlVel    = v.extra[1].value;   // 0x039 control_velocity
  st.ctrlTorque = v.extra[2].value;   // 0x03a control_torque
  st.enc1Pos    = v.extra[3].value;   // 0x052 encoder 1 position
  st.enc1Vel    = v.extra[4].value;   // 0x053 encoder 1 velocity
  st.measPosMotor = v.position;
  st.measTorque   = v.torque;
  loadEstimatorUpdate(st.est, v.position, v.velocity, st.enc1Pos, CONTROL_DT_S);
}

// ===== Feedforward reference helper =========================================

// Mirrors the moteus DoVelocityModeLimits integrator at 200 Hz.
// Advances ffVRef toward target at accel and returns the instantaneous a_ref.
static double advanceVRef(double &vRef, double target, double accel) {
  const double dv   = target - vRef;
  const double step = accel * CONTROL_DT_S;
  if (fabs(dv) <= step) {
    vRef = target;
    return 0.0;
  }
  vRef += copysign(step, dv);
  return copysign(accel, dv);
}

// ===== Command builders (one per control state) =============================
// Each returns true if the moteus replied.

// Velocity: follow the RPM reference, ramping at the rotor's per-direction limit.
static bool sendVelocity(Moteus &moteus, RotorState &st, double targetRPM,
                         double accelUp, double accelDown, Rotor rotor) {
  const double i          = st.est.beltRatio;
  const double targetRevS = targetRPM / 60.0;           // load frame
  const double accelLoad  = (targetRevS >= st.lastRefRevs) ? accelUp : accelDown;
  st.lastRefRevs = targetRevS;

  mm::PositionMode::Command cmd;
  cmd.position       = kNaN;
  cmd.velocity       = targetRevS * i;    // motor frame
  cmd.accel_limit    = accelLoad * i;     // motor frame
  cmd.velocity_limit = kNaN;
  st.lastCmdVelRevS  = cmd.velocity;

  const double aRef = advanceVRef(st.ffVRef, targetRevS * i, accelLoad * i);
  cmd.feedforward_torque = (rotor == Rotor::Drum)
      ? ffDrum(st.ffVRef, aRef, true) : ffHorn(st.ffVRef, aRef, true);
  st.ffTorque = (float)cmd.feedforward_torque;

  return moteus.SetPosition(cmd, &lesliePositionFmt(), &leslieQueryFmt());
}

// Fractional part in [0, 1).
static double wrap01(double x) { return x - floor(x); }

// BeatSync: delegate the trajectory to the cam planner (face frame), convert the
// face command back to the motor frame via the estimator, and log the planner's
// one-shot plan/lock transitions (the planner stays free of Serial and identity).
static bool sendBeatSync(Moteus &moteus, RotorState &st, Rotor rotor,
                         const BeatTarget &target) {
  const double i = st.est.beltRatio;
  const BeatCamInput in{st.est.posL, st.lastSignedVelRevs};  // load frame

  const BeatCamCommand plan = beatCamStep(st.cam, target, in);
  st.lastRefRevs = plan.velRevS;  // load frame

  mm::PositionMode::Command cmd;
  // loadEstimatorFaceToMotor already multiplies by i — motor frame position.
  cmd.position       = loadEstimatorFaceToMotor(st.est, plan.facePos);
  st.cmdPosMotor     = (float)cmd.position;
  // velocity, accel, vel_limit: load→motor by *i; slipVel is in load frame.
  cmd.velocity       = (plan.velRevS + st.est.slipVel) * i;
  cmd.accel_limit    = plan.accelLimit * i;
  cmd.velocity_limit = plan.velocityLimit * i;

  const double aRef = advanceVRef(st.ffVRef, target.velRevS * i, plan.accelLimit * i);
  cmd.feedforward_torque = (rotor == Rotor::Drum)
      ? ffDrum(st.ffVRef, aRef, true) : ffHorn(st.ffVRef, aRef, true);
  st.ffTorque = (float)cmd.feedforward_torque;

  if (st.cam.justPlanned)
    Serial.printf("[cam] %s plan: v0=%.2f vf=%.2f m=%.0f accel=%.2f\n",
                  rotorName(rotor), st.lastSignedVelRevs, target.velRevS,
                  st.cam.revOffset, st.cam.accel);
  if (st.cam.justLocked)
    Serial.printf("[cam] %s locked at %.2f rev/s\n", rotorName(rotor),
                  st.lastSignedVelRevs);

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
    return sendVelocity(moteus, st, 0.0, /*accelUp=*/accelDown, accelDown, rotor);

  if (!st.parked) {
    const double facePos = st.est.posL; // load (face) frame; slip lives in est
    st.homeInteger =
        (st.lastRefRevs >= 0.0) ? ceil(facePos) : floor(facePos);
    st.parked = true;
    Serial.printf("[pos snap] %s pos=%.3f ref=%.3f homeInt=%.0f slip=%.4f\n",
                  rotorName(rotor), st.lastPosRevs, st.lastRefRevs,
                  st.homeInteger, st.est.slip);
  }

  st.ffVRef = 0.0;  // hold phase: no FF needed
  st.ffTorque = 0.0f;
  const double i = st.est.beltRatio;
  mm::PositionMode::Command cmd;
  cmd.position       = loadEstimatorFaceToMotor(st.est, st.homeInteger);
  cmd.velocity       = 0.0;
  cmd.accel_limit    = ACCEL_LIMIT_STOP_REV_S2 * i;
  cmd.velocity_limit = VELOCITY_LIMIT_STOP * i;
  st.cmdPosMotor     = (float)cmd.position;
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
    bool go = clockSyncIsRunning() && clockSyncIsLocked()
           && beatSyncGetSubdivision() != Subdivision::Rest;
    return go ? ControlState::BeatSync : ControlState::Park;
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

  loadEstimatorInit(horn.est, HORN_BELT_RATIO);
  loadEstimatorInit(drum.est, DRUM_BELT_RATIO);

  beatCamInit(horn.cam, BEAT_ACCEL_UP_REV_S2, BEAT_ACCEL_DOWN_REV_S2, CONTROL_DT_S);
  beatCamInit(drum.cam, BEAT_ACCEL_UP_REV_S2, BEAT_ACCEL_DOWN_REV_S2, CONTROL_DT_S);

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

  TickType_t lastWakeTime = xTaskGetTickCount();
  for (;;) {
    Reference ref;
    referenceGet(ref);

    const ControlState next = nextState(ref);
    if (next != state) {
      // Reset the per-state entry latches for both rotors.
      if (next == ControlState::BeatSync) {
        beatCamReset(horn.cam); // re-plan the cam on entry
        beatCamReset(drum.cam);
      } else if (next == ControlState::Park)
        horn.parked = drum.parked = false; // a fresh stop re-snaps the mark
      // Seed ffVRef in motor frame so there is no torque spike on entry.
      horn.ffVRef = horn.lastSignedVelRevs * HORN_BELT_RATIO;
      drum.ffVRef = drum.lastSignedVelRevs * DRUM_BELT_RATIO;
      state = next;
    }

    // A subdivision change retargets the rotors, so re-plan the cam (a new tpc
    // means a new speed/phase target) while we're locked to the beat.
    const Subdivision sd = beatSyncGetSubdivision();
    if (sd != beatSubdiv) {
      beatSubdiv = sd;
      if (state == ControlState::BeatSync) {
        beatCamReset(horn.cam);
        beatCamReset(drum.cam);
      }
    }

    bool hornReplied = false;
    bool drumReplied = false;
    switch (state) {
    case ControlState::Velocity:
      hornReplied = sendVelocity(hornMoteus(), horn, ref.hornRPM,
                                 ACCEL_UP_HORN_REV_S2, ACCEL_DOWN_HORN_REV_S2,
                                 Rotor::Horn);
      drumReplied = sendVelocity(drumMoteus(), drum, ref.drumRPM,
                                 ACCEL_UP_DRUM_REV_S2, ACCEL_DOWN_DRUM_REV_S2,
                                 Rotor::Drum);
      break;
    case ControlState::BeatSync: {
      // One coherent clock+subdivision read drives both rotors this tick.
      BeatTarget hornTarget, drumTarget;
      beatSyncGetTargets(hornTarget, drumTarget);
      hornReplied = sendBeatSync(hornMoteus(), horn, Rotor::Horn, hornTarget);
      drumReplied = sendBeatSync(drumMoteus(), drum, Rotor::Drum, drumTarget);
      break;
    }
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
    // Publish the controller's own signals to the shared logging bus (non-
    // blocking; telemetryLogTask does the UDP off this loop). Clock/PLL signals
    // are logged by clock_sync itself. dbgFaceJumpPeak accumulates between
    // samples, so it is logged and then cleared here.
    static uint8_t s_telemDiv = 0;
    if (++s_telemDiv >= 5) {
      s_telemDiv = 0;
      auto wrapHalf = [](double x) { return x - round(x); };

      // All keys are PlotJuggler tree paths: <motor>/<subsystem>/<signal>, so
      // each motor and subsystem collapses into its own branch and a whole group
      // can be dragged into one plot. Global signals live under beat/ and state/.

      // -- speed: measured RPM vs the active RPM reference -------------------
      telemetryLog("horn/speed/rpm",     (float)(horn.lastVelRevs * 60.0));
      telemetryLog("horn/speed/ref_rpm", ref.hornRPM);
      telemetryLog("drum/speed/rpm",     (float)(drum.lastVelRevs * 60.0));
      telemetryLog("drum/speed/ref_rpm", ref.drumRPM);

      // -- posctrl: position-controller test (chap. Test / Position Controller).
      // The three states moteus reports IN ONE REPLY (timestep-aligned), motor
      // frame: traj = moteus' internal trajectory-generator reference (control_*),
      // meas = measurement, torque/cmd = control_torque the loops command.
      telemetryLog("horn/posctrl/pos/cmd",     horn.cmdPosMotor);
      telemetryLog("horn/posctrl/pos/traj",    horn.ctrlPos);
      telemetryLog("horn/posctrl/pos/meas",    horn.measPosMotor);
      telemetryLog("horn/posctrl/vel/traj",    horn.ctrlVel);
      telemetryLog("horn/posctrl/vel/meas",    (float)(horn.lastSignedVelRevs * horn.est.beltRatio));
      telemetryLog("horn/posctrl/torque/cmd",  horn.ctrlTorque);
      telemetryLog("horn/posctrl/torque/meas", horn.measTorque);
      telemetryLog("horn/posctrl/torque/ff",   horn.ffTorque);
      telemetryLog("drum/posctrl/pos/cmd",     drum.cmdPosMotor);
      telemetryLog("drum/posctrl/pos/traj",    drum.ctrlPos);
      telemetryLog("drum/posctrl/pos/meas",    drum.measPosMotor);
      telemetryLog("drum/posctrl/vel/traj",    drum.ctrlVel);
      telemetryLog("drum/posctrl/vel/meas",    (float)(drum.lastSignedVelRevs * drum.est.beltRatio));
      telemetryLog("drum/posctrl/torque/cmd",  drum.ctrlTorque);
      telemetryLog("drum/posctrl/torque/meas", drum.measTorque);
      telemetryLog("drum/posctrl/torque/ff",   drum.ffTorque);

      // -- slip: offset = tracked (smooth); raw = raw measured (noisy, offset
      // should ride its mean); vel = creep rate -----------------------------
      telemetryLog("horn/slip/offset", (float)wrapHalf(horn.est.slip));
      telemetryLog("horn/slip/raw",    (float)wrapHalf(horn.enc1Pos - wrap01(horn.lastPosRevs)));
      telemetryLog("horn/slip/vel",    (float)horn.est.slipVel);
      telemetryLog("horn/slip/err",    (float)horn.est.innov);  // tracking error (innovation)
      telemetryLog("drum/slip/offset", (float)wrapHalf(drum.est.slip));
      telemetryLog("drum/slip/raw",    (float)wrapHalf(drum.enc1Pos - wrap01(drum.lastPosRevs)));
      telemetryLog("drum/slip/vel",    (float)drum.est.slipVel);
      telemetryLog("drum/slip/err",    (float)drum.est.innov);  // tracking error (innovation)

      // -- beat: BeatSync scoping. face_jump = worst face step since last sample
      // (≈0 if smooth); cmd_pos/cmd_vel = setpoint handed to the planner;
      // err_deg = phase error (mech deg, ±180); err_ms = same as a time delay;
      // pos_frac = motor angle fraction; motor_cmd = (faceCmd-slip)*i in motor
      // frame (plot vs enc/enc0_frac to see tracking lag); cmd_vel_revs = last
      // velocity command [rev/s].
      telemetryLog("horn/beat/face_jump",    (float)horn.cam.dbgFaceJumpPeak);
      telemetryLog("horn/beat/cmd_pos",      (float)horn.cam.dbgCmdPos);
      telemetryLog("horn/beat/cmd_vel",      (float)horn.cam.dbgCmdVel);
      telemetryLog("horn/beat/err_deg",      (float)horn.cam.dbgPhaseErrDeg);
      telemetryLog("horn/beat/err_ms",       (horn.cam.dbgCmdVel > 0.01) ? (float)(horn.cam.dbgPhaseErrDeg / 360.0 / horn.cam.dbgCmdVel * 1000.0) : 0.0f);
      telemetryLog("horn/beat/pos_frac",     (float)(horn.lastPosRevs - floor(horn.lastPosRevs)));
      telemetryLog("horn/beat/motor_cmd",    (float)wrap01((horn.cam.dbgCmdPos - horn.est.slip) * HORN_BELT_RATIO));
      telemetryLog("horn/beat/cmd_vel_revs", (float)horn.lastCmdVelRevS);
      telemetryLog("drum/beat/face_jump",    (float)drum.cam.dbgFaceJumpPeak);
      telemetryLog("drum/beat/cmd_pos",      (float)drum.cam.dbgCmdPos);
      telemetryLog("drum/beat/cmd_vel",      (float)drum.cam.dbgCmdVel);
      telemetryLog("drum/beat/err_deg",      (float)drum.cam.dbgPhaseErrDeg);
      telemetryLog("drum/beat/err_ms",       (drum.cam.dbgCmdVel > 0.01) ? (float)(drum.cam.dbgPhaseErrDeg / 360.0 / drum.cam.dbgCmdVel * 1000.0) : 0.0f);
      telemetryLog("drum/beat/pos_frac",     (float)(drum.lastPosRevs - floor(drum.lastPosRevs)));
      telemetryLog("drum/beat/motor_cmd",    (float)wrap01((drum.cam.dbgCmdPos - drum.est.slip) * DRUM_BELT_RATIO));
      telemetryLog("drum/beat/cmd_vel_revs", (float)drum.lastCmdVelRevS);
      horn.cam.dbgFaceJumpPeak = drum.cam.dbgFaceJumpPeak = 0.0;

      // -- cam: BeatCam planner state. locked = spin-up ramp reached tempo;
      // accel = planned spin-up accel [rev/s^2]; rev_offset = whole-rev branch at
      // (re)entry; just_planned/just_locked = one-shot flags, cleared after log.
      telemetryLog("horn/cam/locked",       horn.cam.locked ? 1.0f : 0.0f);
      telemetryLog("horn/cam/accel",        (float)horn.cam.accel);
      telemetryLog("horn/cam/rev_offset",   (float)horn.cam.revOffset);
      telemetryLog("horn/cam/just_planned", horn.cam.justPlanned ? 1.0f : 0.0f);
      telemetryLog("horn/cam/just_locked",  horn.cam.justLocked  ? 1.0f : 0.0f);
      telemetryLog("drum/cam/locked",       drum.cam.locked ? 1.0f : 0.0f);
      telemetryLog("drum/cam/accel",        (float)drum.cam.accel);
      telemetryLog("drum/cam/rev_offset",   (float)drum.cam.revOffset);
      telemetryLog("drum/cam/just_planned", drum.cam.justPlanned ? 1.0f : 0.0f);
      telemetryLog("drum/cam/just_locked",  drum.cam.justLocked  ? 1.0f : 0.0f);
      horn.cam.justPlanned = drum.cam.justPlanned = false;
      horn.cam.justLocked  = drum.cam.justLocked  = false;

      // -- enc: output encoders + fused load estimate. enc0 = onboard output,
      // enc1 = MA600, est = fused (enc0 + est.slip), all angle fractions ------
      telemetryLog("horn/enc/enc0_frac", (float)wrap01(horn.lastPosRevs));
      telemetryLog("horn/enc/enc1_frac", horn.enc1Pos);
      telemetryLog("horn/enc/est_frac",  (float)wrap01(horn.est.posL));
      telemetryLog("drum/enc/enc0_frac", (float)wrap01(drum.lastPosRevs));
      telemetryLog("drum/enc/enc1_frac", drum.enc1Pos);
      telemetryLog("drum/enc/est_frac",  (float)wrap01(drum.est.posL));

      // -- vel_est: load-frame velocity estimates. motor = raw; fused = motor +
      // slipVel; enc1 = MA600 [rev/s] ----------------------------------------
      telemetryLog("horn/vel_est/motor", (float)horn.lastSignedVelRevs);
      telemetryLog("horn/vel_est/fused", (float)horn.est.velL);
      telemetryLog("horn/vel_est/enc1",  horn.enc1Vel);
      telemetryLog("drum/vel_est/motor", (float)drum.lastSignedVelRevs);
      telemetryLog("drum/vel_est/fused", (float)drum.est.velL);
      telemetryLog("drum/vel_est/enc1",  drum.enc1Vel);

      // -- global: beat phase (0..1 within one MIDI beat) + subdivision enum
      // (0=Rest,1=Half…12=32ndTriplet); state/ctrl = 0:Velocity 1:Park 2:BeatSync.
      telemetryLog("beat/phase",       clockSyncGetPhase());
      telemetryLog("beat/subdivision", (float)(int)beatSyncGetSubdivision());
      telemetryLog("state/ctrl",       (float)(int)state);
      telemetryLog("state/drive",      (float)(int)ref.mode);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5)); // 200 Hz
  }
}

