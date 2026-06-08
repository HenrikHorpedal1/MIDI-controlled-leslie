// controller.cpp
#include "controller.h"
#include "clock_sync.h"
#include "moteus-config.h"
#include "reference.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <limits>
#include <math.h>

// Both motors report in rotor/output units — the output (gear) ratio is
// configured in tview (drum 0.25), so no gear ratio is applied in code.

static constexpr double ACCEL_LIMIT_UP_HORN_REV_S2 =
    3.50; // 7.0 rev/s (420 RPM) in 2s
static constexpr double ACCEL_LIMIT_UP_DRUM_REV_S2 =
    1.458; // 5.833 rev/s (350 RPM) in 4s
static constexpr double ACCEL_LIMIT_DOWN_REV_S2 =
    2.50; // ~2.7s ramp down from tremolo
static constexpr double ACCEL_LIMIT_BEAT_REV_S2 = 1.67;
static constexpr double BEAT_HOME_FRAC =
    0.0; // 0..1: rotor face to align with beat
static constexpr double BEAT_REVS_PER_BEAT =
    2.0; // rotor revolutions per MIDI quarter-note beat

static constexpr double ACCEL_LIMIT_STOP_REV_S2 = 0.50;
static constexpr double VELOCITY_LIMIT_STOP = 0.5;
static constexpr double SLIP_FILTER_ALPHA = 0.005; // ~2s TC at 100 Hz
static constexpr double HORN_STOP_THRESH_REV_S =
    0.333; // 20 RPM in rotor units (output ratio applied)
static constexpr double DRUM_STOP_THRESH_REV_S =
    0.333; // 20 RPM in rotor units (output ratio applied)

static const double kNaN = std::numeric_limits<double>::quiet_NaN();

static mm::PositionMode::Format makeFullFmt() {
  mm::PositionMode::Format f;
  f.accel_limit = mm::kFloat;
  f.velocity_limit = mm::kFloat;
  return f;
}
static const mm::PositionMode::Format kFullFmt = makeFullFmt();

static mm::Query::Format makeQueryFmt() {
  mm::Query::Format f;
  f.position = mm::kFloat;
  f.velocity = mm::kFloat;
  f.extra[0].register_number = mm::Register(0x052); // Encoder 1 position
  f.extra[0].resolution = mm::kFloat;
  f.extra[1].register_number = mm::Register(0x053); // Encoder 1 velocity
  f.extra[1].resolution = mm::kFloat;
  return f;
}
static const mm::Query::Format kQueryFmt = makeQueryFmt();

enum class ControlMode { Velocity, Position, BeatSync };

struct RotorState {
  double lastPosRevs = 0.0;
  double lastVelRevs = 0.0;
  double lastSignedVelRevs = 0.0;
  double lastRefRevs = 0.0;
  double homePos = 0.0;
  float enc1Pos = 0.0f;
  float enc1Vel = 0.0f;
};

// Per-rotor control context: bundles the moteus handle with all the state the
// control loop needs so the same logic can run for both horn and drum.
struct RotorCtrl {
  Moteus *moteus;
  const char *name;
  double stopThreshRevS;
  double accelUpRevS2;
  RotorState state;
  ControlMode mode;
  double filteredSlip;
  double beatOffset;
  bool beatTracking;
};

static mm::PositionMode::Command velocityCmd(double targetRevS,
                                             double lastRefRevS,
                                             double accelUpRevS2) {
  mm::PositionMode::Command cmd;
  cmd.position = kNaN;
  cmd.velocity = targetRevS;
  cmd.accel_limit =
      (targetRevS >= lastRefRevS) ? accelUpRevS2 : ACCEL_LIMIT_DOWN_REV_S2;
  cmd.velocity_limit = kNaN;
  return cmd;
}

// Selects the control mode for one rotor based on the reference and the MIDI
// clock, then issues the matching moteus command. Returns true if the
// controller replied.
static bool driveRotor(RotorCtrl &rc, const ReferenceState &ref,
                       bool clockActive) {
  // --- Mode selection ---
  if (clockActive) {
    if (rc.mode != ControlMode::BeatSync) {
      rc.mode = ControlMode::BeatSync;
      rc.beatTracking = false;
    }
  } else if (ref.velRPM == 0.0f) {
    if ((rc.mode == ControlMode::Velocity || rc.mode == ControlMode::BeatSync) &&
        rc.state.lastVelRevs < rc.stopThreshRevS) {
      const double snapPos = (rc.state.lastRefRevs >= 0.0)
                                 ? ceil(rc.state.lastPosRevs)
                                 : floor(rc.state.lastPosRevs);
      rc.state.homePos = snapPos - rc.filteredSlip;
      Serial.printf("[pos snap] %s pos=%.3f ref=%.3f home=%.3f slip=%.4f\n",
                    rc.name, rc.state.lastPosRevs, rc.state.lastRefRevs,
                    rc.state.homePos, rc.filteredSlip);
      rc.mode = ControlMode::Position;
      rc.beatTracking = false;
    }
  } else {
    if (rc.mode != ControlMode::Velocity) {
      rc.mode = ControlMode::Velocity;
      rc.beatTracking = false;
    }
  }

  // --- Commands ---
  switch (rc.mode) {

  case ControlMode::Velocity: {
    const double targetRevS = ref.velRPM / 60.0; // rotor units, no gear ratio
    auto cmd = velocityCmd(targetRevS, rc.state.lastRefRevs, rc.accelUpRevS2);
    rc.state.lastRefRevs = targetRevS;
    return rc.moteus->SetPosition(cmd, &kFullFmt, &kQueryFmt);
  }

  case ControlMode::BeatSync: {
    // Speed is derived from the MIDI clock, not from the Leslie
    // chorale/tremolo reference.
    const double bpm = clockSyncGetBpm();
    const double targetRevS = bpm / 60.0 * BEAT_REVS_PER_BEAT;
    const uint64_t ticks = clockSyncGetTickCount();
    const double subPhase = clockSyncGetPhase();
    const double clockPos = (ticks + subPhase) / 24.0 * BEAT_REVS_PER_BEAT;

    if (!rc.beatTracking) {
      // encoder absolute pos = lastPosRevs + slip (slip is negative: motor
      // ahead of enc1) We want the encoder to land on BEAT_HOME_FRAC at each
      // beat. cmd.position = clockPos + beatOffset - slip  →  encoder =
      // clockPos + beatOffset Choose K (integer) to minimise the initial
      // jump from current encoder position.
      const double encoderPos = rc.state.lastPosRevs + rc.filteredSlip;
      const double K = round(encoderPos - BEAT_HOME_FRAC - clockPos);
      rc.beatOffset = BEAT_HOME_FRAC + K;
      rc.beatTracking = true;
      Serial.printf("[beatsync] %s locked: encoderPos=%.3f clockPos=%.3f K=%.0f "
                    "offset=%.3f slip=%.4f\n",
                    rc.name, encoderPos, clockPos, K, rc.beatOffset,
                    rc.filteredSlip);
    }

    mm::PositionMode::Command cmd;
    cmd.position = clockPos + rc.beatOffset - rc.filteredSlip;
    cmd.velocity = targetRevS;
    cmd.accel_limit = ACCEL_LIMIT_BEAT_REV_S2;
    cmd.velocity_limit = kNaN;
    rc.state.lastRefRevs = targetRevS;
    return rc.moteus->SetPosition(cmd, &kFullFmt, &kQueryFmt);
  }

  case ControlMode::Position: {
    mm::PositionMode::Command cmd;
    cmd.position = rc.state.homePos;
    cmd.velocity = 0.0;
    cmd.accel_limit = ACCEL_LIMIT_STOP_REV_S2;
    cmd.velocity_limit = VELOCITY_LIMIT_STOP;
    return rc.moteus->SetPosition(cmd, &kFullFmt, &kQueryFmt);
  }
  }

  return false;
}

// Pulls the latest reply into the rotor state and updates the filtered slip
// between the moteus output position and encoder 1.
static void updateRotorState(RotorCtrl &rc) {
  const auto &v = rc.moteus->last_result().values;
  rc.state.lastPosRevs = v.position;
  rc.state.lastSignedVelRevs = v.velocity;
  rc.state.lastVelRevs = fabs(v.velocity);
  rc.state.enc1Pos = v.extra[0].value;
  rc.state.enc1Vel = v.extra[1].value;
  const double lastPosFrac = rc.state.lastPosRevs - floor(rc.state.lastPosRevs);
  double rawSlip = rc.state.enc1Pos - lastPosFrac;
  if (rawSlip > 0.5)
    rawSlip -= 1.0;
  if (rawSlip < -0.5)
    rawSlip += 1.0;
  rc.filteredSlip += SLIP_FILTER_ALPHA * (rawSlip - rc.filteredSlip);
}

static const char *modeName(ControlMode m) {
  return (m == ControlMode::Velocity)   ? "Velocity"
         : (m == ControlMode::BeatSync) ? "BeatSync"
                                        : "Position";
}

void controllerTask(void *pvParameters) {
  if (!configureMoteus(Serial)) {
    Serial.println("Moteus init failed — controller task exiting");
    vTaskDelete(nullptr);
    return;
  }

  RotorCtrl horn{&hornMoteus(),
                 "horn",
                 HORN_STOP_THRESH_REV_S,
                 ACCEL_LIMIT_UP_HORN_REV_S2,
                 {},
                 ControlMode::Velocity,
                 0.0,
                 0.0,
                 false};
  RotorCtrl drum{&drumMoteus(),
                 "drum",
                 DRUM_STOP_THRESH_REV_S,
                 ACCEL_LIMIT_UP_DRUM_REV_S2,
                 {},
                 ControlMode::Velocity,
                 0.0,
                 0.0,
                 false};

  for (;;) {
    ReferenceState hornRef, drumRef;
    referenceGet(Rotor::Horn, hornRef);
    referenceGet(Rotor::Drum, drumRef);

    const bool clockActive = clockSyncIsRunning() && clockSyncIsLocked();

    const bool hornReplied = driveRotor(horn, hornRef, clockActive);
    const bool drumReplied = driveRotor(drum, drumRef, clockActive);

    if (hornReplied)
      updateRotorState(horn);
    if (drumReplied)
      updateRotorState(drum);

    // --- Debug: print speed every second ---
    static uint32_t s_lastPrintMs = 0;
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - s_lastPrintMs >= 1000) {
      s_lastPrintMs = now;
      for (RotorCtrl *rc : {&horn, &drum}) {
        const double lastPosFrac =
            rc->state.lastPosRevs - floor(rc->state.lastPosRevs);
        double displayRawSlip = rc->state.enc1Pos - lastPosFrac;
        if (displayRawSlip > 0.5)
          displayRawSlip -= 1.0;
        if (displayRawSlip < -0.5)
          displayRawSlip += 1.0;
        Serial.printf("[ctrl] mode=%s %s: %.2f rev/s (%.1f RPM) | enc1 "
                      "pos=%.3f vel=%.2f rev/s | slip raw=%.4f filt=%.4f\n",
                      modeName(rc->mode), rc->name, rc->state.lastVelRevs,
                      rc->state.lastVelRevs * 60.0, rc->state.enc1Pos,
                      rc->state.enc1Vel, displayRawSlip, rc->filteredSlip);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
  }
}
