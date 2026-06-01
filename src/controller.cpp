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

// static constexpr double HORN_GEAR_RATIO = 3.5;  // drum-only testing
// DRUM_GEAR_RATIO removed — output ratio 0.25 configured in tview, moteus
// reports in rotor units

static constexpr double ACCEL_LIMIT_UP_REV_S2 = 1.67; // ~4s ramp up to tremolo
static constexpr double ACCEL_LIMIT_DOWN_REV_S2 =
    2.50; // ~2.7s ramp down from tremolo
static constexpr double ACCEL_LIMIT_BEAT_REV_S2 = 1.67;
static constexpr double BEAT_HOME_FRAC =
    0.0; // 0..1: rotor face to align with beat
static constexpr double BEAT_REVS_PER_BEAT =
    2.0; // rotor revolutions per MIDI quarter-note beat

// Velocity-mode gain scales — commented out, using moteus saved gains directly
// static constexpr double VEL_KP_SCALE     = 0.0714;
// static constexpr double VEL_ILIMIT_SCALE = 0.0;
// static constexpr double VEL_KD_SCALE     = 0.714;

static constexpr double ACCEL_LIMIT_STOP_REV_S2 = 0.50;
static constexpr double VELOCITY_LIMIT_STOP = 0.5;
static constexpr double SLIP_FILTER_ALPHA = 0.005; // ~2s TC at 100 Hz
// static constexpr double HORN_STOP_THRESH_REV_S = 1.167;  // drum-only testing
static constexpr double DRUM_STOP_THRESH_REV_S =
    0.333; // 20 RPM in rotor units (output ratio applied)

static const double kNaN = std::numeric_limits<double>::quiet_NaN();

static mm::PositionMode::Format makeFullFmt() {
  mm::PositionMode::Format f;
  f.accel_limit = mm::kFloat;
  f.velocity_limit = mm::kFloat;
  // f.kp_scale       = mm::kFloat;  // PID scaling disabled — using saved gains
  // f.kd_scale       = mm::kFloat;
  // f.ilimit_scale   = mm::kFloat;
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

static mm::PositionMode::Command velocityCmd(double targetRevS,
                                             double lastRefRevS) {
  mm::PositionMode::Command cmd;
  cmd.position = kNaN;
  cmd.velocity = targetRevS;
  cmd.accel_limit = (targetRevS >= lastRefRevS) ? ACCEL_LIMIT_UP_REV_S2
                                                : ACCEL_LIMIT_DOWN_REV_S2;
  cmd.velocity_limit = kNaN;
  // cmd.kp_scale       = VEL_KP_SCALE;    // PID scaling disabled
  // cmd.kd_scale       = VEL_KD_SCALE;
  // cmd.ilimit_scale   = VEL_ILIMIT_SCALE;
  return cmd;
}

void controllerTask(void *pvParameters) {
  if (!configureMoteus(Serial)) {
    Serial.println("Moteus init failed — controller task exiting");
    vTaskDelete(nullptr);
    return;
  }

  ControlMode mode = ControlMode::Velocity;
  RotorState hornState, drumState;
  double drumFilteredSlip = 0.0;
  double s_beatOffset = 0.0;
  bool s_beatTracking = false;

  for (;;) {
    ReferenceState hornRef, drumRef;
    referenceGet(Rotor::Horn, hornRef);
    referenceGet(Rotor::Drum, drumRef);

    const bool clockActive = clockSyncIsRunning() && clockSyncIsLocked();

    // --- Mode selection (drum only) ---
    if (clockActive) {
      if (mode != ControlMode::BeatSync) {
        mode = ControlMode::BeatSync;
        s_beatTracking = false;
      }
    } else if (drumRef.velRPM == 0.0f) {
      if ((mode == ControlMode::Velocity || mode == ControlMode::BeatSync) &&
          drumState.lastVelRevs < DRUM_STOP_THRESH_REV_S) {
        const double snapPos = (drumState.lastRefRevs >= 0.0)
                                   ? ceil(drumState.lastPosRevs)
                                   : floor(drumState.lastPosRevs);
        drumState.homePos = snapPos - drumFilteredSlip;
        Serial.printf("[pos snap] drum pos=%.3f ref=%.3f home=%.3f slip=%.4f\n",
                      drumState.lastPosRevs, drumState.lastRefRevs,
                      drumState.homePos, drumFilteredSlip);
        mode = ControlMode::Position;
        s_beatTracking = false;
      }
    } else {
      if (mode != ControlMode::Velocity) {
        mode = ControlMode::Velocity;
        s_beatTracking = false;
      }
    }

    // --- Commands ---
    bool hornReplied = false, drumReplied = false;

    switch (mode) {

    case ControlMode::Velocity: {
      // hornReplied left false — drum only testing
      const double drumTargetRevS =
          drumRef.velRPM / 60.0; // rotor units, no gear ratio
      auto drumCmd = velocityCmd(drumTargetRevS, drumState.lastRefRevs);
      drumState.lastRefRevs = drumTargetRevS;
      drumReplied = drumMoteus().SetPosition(drumCmd, &kFullFmt, &kQueryFmt);
      break;
    }

    case ControlMode::BeatSync: {
      // hornReplied left false — drum only testing
      // Speed is derived from the MIDI clock, not from the Leslie
      // chorale/tremolo reference.
      const double bpm = clockSyncGetBpm();
      const double drumTargetRevS = bpm / 60.0 * BEAT_REVS_PER_BEAT;
      const uint64_t ticks = clockSyncGetTickCount();
      const double subPhase = clockSyncGetPhase();
      const double clockPos = (ticks + subPhase) / 24.0 * BEAT_REVS_PER_BEAT;

      if (!s_beatTracking) {
        // encoder absolute pos = lastPosRevs + slip (slip is negative: motor
        // ahead of enc1) We want the encoder to land on BEAT_HOME_FRAC at each
        // beat. cmd.position = clockPos + s_beatOffset - slip  →  encoder =
        // clockPos + s_beatOffset Choose K (integer) to minimise the initial
        // jump from current encoder position.
        const double encoderPos = drumState.lastPosRevs + drumFilteredSlip;
        const double K = round(encoderPos - BEAT_HOME_FRAC - clockPos);
        s_beatOffset = BEAT_HOME_FRAC + K;
        s_beatTracking = true;
        Serial.printf("[beatsync] locked: encoderPos=%.3f clockPos=%.3f K=%.0f "
                      "offset=%.3f slip=%.4f\n",
                      encoderPos, clockPos, K, s_beatOffset, drumFilteredSlip);
      }

      mm::PositionMode::Command drumCmd;
      drumCmd.position = clockPos + s_beatOffset - drumFilteredSlip;
      drumCmd.velocity = drumTargetRevS;
      drumCmd.accel_limit = ACCEL_LIMIT_BEAT_REV_S2;
      drumCmd.velocity_limit = kNaN;
      drumState.lastRefRevs = drumTargetRevS;
      drumReplied = drumMoteus().SetPosition(drumCmd, &kFullFmt, &kQueryFmt);
      break;
    }

    case ControlMode::Position: {
      // hornReplied left false — drum only testing
      mm::PositionMode::Command drumCmd;
      drumCmd.position = drumState.homePos;
      drumCmd.velocity = 0.0;
      drumCmd.accel_limit = ACCEL_LIMIT_STOP_REV_S2;
      drumCmd.velocity_limit = VELOCITY_LIMIT_STOP;
      drumReplied = drumMoteus().SetPosition(drumCmd, &kFullFmt, &kQueryFmt);
      break;
    }
    }

    // --- Update state from replies ---
    if (hornReplied) {
      const auto &v = hornMoteus().last_result().values;
      hornState.lastPosRevs = v.position;
      hornState.lastSignedVelRevs = v.velocity;
      hornState.lastVelRevs = fabs(v.velocity);
      hornState.enc1Pos = v.extra[0].value;
      hornState.enc1Vel = v.extra[1].value;
    }
    if (drumReplied) {
      const auto &v = drumMoteus().last_result().values;
      drumState.lastPosRevs = v.position;
      drumState.lastSignedVelRevs = v.velocity;
      drumState.lastVelRevs = fabs(v.velocity);
      drumState.enc1Pos = v.extra[0].value;
      drumState.enc1Vel = v.extra[1].value;
      const double lastPosFrac =
          drumState.lastPosRevs - floor(drumState.lastPosRevs);
      double rawSlip = drumState.enc1Pos - lastPosFrac;
      if (rawSlip > 0.5)
        rawSlip -= 1.0;
      if (rawSlip < -0.5)
        rawSlip += 1.0;
      drumFilteredSlip += SLIP_FILTER_ALPHA * (rawSlip - drumFilteredSlip);
    }

    // --- Debug: print speed every second ---
    static uint32_t s_lastPrintMs = 0;
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - s_lastPrintMs >= 1000) {
      s_lastPrintMs = now;
      const char *modeName = (mode == ControlMode::Velocity)   ? "Velocity"
                             : (mode == ControlMode::BeatSync) ? "BeatSync"
                                                               : "Position";
      const double lastPosFrac =
          drumState.lastPosRevs - floor(drumState.lastPosRevs);
      double displayRawSlip = drumState.enc1Pos - lastPosFrac;
      if (displayRawSlip > 0.5)
        displayRawSlip -= 1.0;
      if (displayRawSlip < -0.5)
        displayRawSlip += 1.0;
      Serial.printf("[ctrl] mode=%s drum: %.2f rev/s (%.1f RPM) | enc1 "
                    "pos=%.3f vel=%.2f rev/s | slip raw=%.4f filt=%.4f\n",
                    modeName, drumState.lastVelRevs,
                    drumState.lastVelRevs * 60.0, drumState.enc1Pos,
                    drumState.enc1Vel, displayRawSlip, drumFilteredSlip);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
  }
}
