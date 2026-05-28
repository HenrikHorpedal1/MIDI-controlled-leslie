// controller.cpp
#include "controller.h"
#include "reference.h"
#include "moteus-config.h"
#include "clock_sync.h"

#include <Arduino.h>
#include <math.h>
#include <limits>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr double HORN_GEAR_RATIO = 3.5;
static constexpr double DRUM_GEAR_RATIO = 4.0;

static constexpr double ACCEL_LIMIT_UP_REV_S2   = 1.67;  // ~4s ramp up to tremolo
static constexpr double ACCEL_LIMIT_DOWN_REV_S2 = 2.50;  // ~2.7s ramp down from tremolo
// static constexpr double ACCEL_LIMIT_BEAT_REV_S2 = 1.67;  // BeatSync
// static constexpr double SUBDIVISION             = 1.0;   // BeatSync
// static constexpr double SPINUP_TOLERANCE        = 0.15;  // BeatSync

// Velocity-mode gain scales — commented out, using moteus saved gains directly
// static constexpr double VEL_KP_SCALE     = 0.0714;
// static constexpr double VEL_ILIMIT_SCALE = 0.0;
// static constexpr double VEL_KD_SCALE     = 0.714;

// Position mode constants — kept here for when tuning is done
// static constexpr double ACCEL_LIMIT_STOP_REV_S2 = 0.50;
// static constexpr double VELOCITY_LIMIT_STOP      = 0.5;
// static constexpr double STOP_THRESH_REV_S        = 0.4;

static const double kNaN = std::numeric_limits<double>::quiet_NaN();

static mm::PositionMode::Format makeFullFmt() {
    mm::PositionMode::Format f;
    f.accel_limit    = mm::kFloat;
    f.velocity_limit = mm::kFloat;
    // f.kp_scale       = mm::kFloat;  // PID scaling disabled — using saved gains
    // f.kd_scale       = mm::kFloat;
    // f.ilimit_scale   = mm::kFloat;
    return f;
}
static const mm::PositionMode::Format kFullFmt = makeFullFmt();

enum class ControlMode { Velocity /*, BeatSync, Position — needs tuning */ };

struct RotorState {
    double lastPosRevs       = 0.0;
    double lastVelRevs       = 0.0;
    double lastSignedVelRevs = 0.0;
    double lastRefRevs       = 0.0;
};

static mm::PositionMode::Command velocityCmd(double targetRevS, double lastRefRevS) {
    mm::PositionMode::Command cmd;
    cmd.position       = kNaN;
    cmd.velocity       = targetRevS;
    cmd.accel_limit    = (targetRevS >= lastRefRevS) ? ACCEL_LIMIT_UP_REV_S2
                                                     : ACCEL_LIMIT_DOWN_REV_S2;
    cmd.velocity_limit = kNaN;
    // cmd.kp_scale       = VEL_KP_SCALE;    // PID scaling disabled
    // cmd.kd_scale       = VEL_KD_SCALE;
    // cmd.ilimit_scale   = VEL_ILIMIT_SCALE;
    return cmd;
}

void controllerTask(void* pvParameters) {
    if (!configureMoteus(Serial)) {
        Serial.println("Moteus init failed — controller task exiting");
        vTaskDelete(nullptr);
        return;
    }

    ControlMode mode = ControlMode::Velocity;
    RotorState  hornState, drumState;
    // double      s_beatOffset   = 0.0;  // BeatSync
    // bool        s_beatTracking = false; // BeatSync

    for (;;) {
        ReferenceState hornRef, drumRef;
        referenceGet(Rotor::Horn, hornRef);
        referenceGet(Rotor::Drum, drumRef);

        // const bool clockActive = clockSyncIsRunning() && clockSyncIsLocked();  // BeatSync

        // --- Mode selection ---
        // if (clockActive) {
        //     if (mode != ControlMode::BeatSync) {
        //         s_beatTracking = false;
        //         mode = ControlMode::BeatSync;
        //     }
        // } else {
        //     if (mode == ControlMode::BeatSync) {
        //         mode = ControlMode::Velocity;
        //         s_beatTracking = false;
        //     }
        //     // Position mode transition removed — needs tuning before re-enabling
        // }

        // --- Commands ---
        bool hornReplied = false, drumReplied = false;

        switch (mode) {

            case ControlMode::Velocity: {
                const double hornTargetRevS = (hornRef.velRPM / 60.0) * HORN_GEAR_RATIO;
                const double drumTargetRevS = (drumRef.velRPM / 60.0) * DRUM_GEAR_RATIO;

                auto hornCmd = velocityCmd(hornTargetRevS, hornState.lastRefRevs);
                auto drumCmd = velocityCmd(drumTargetRevS, drumState.lastRefRevs);

                hornState.lastRefRevs = hornTargetRevS;
                drumState.lastRefRevs = drumTargetRevS;

                hornReplied = hornMoteus().SetPosition(hornCmd, &kFullFmt);
                drumReplied = drumMoteus().SetPosition(drumCmd, &kFullFmt);
                break;
            }

            // case ControlMode::BeatSync: {
            //     const double targetVel = clockSyncGetBpm() * SUBDIVISION / 60.0;
            //     if (!s_beatTracking) {
            //         auto hornCmd        = velocityCmd(targetVel, hornState.lastRefRevs);
            //         hornCmd.accel_limit = ACCEL_LIMIT_BEAT_REV_S2;
            //         hornState.lastRefRevs = targetVel;
            //         hornReplied = hornMoteus().SetPosition(hornCmd, &kFullFmt);
            //         const double velErr = fabs(hornState.lastVelRevs - targetVel);
            //         if (targetVel > 0.0 && velErr < targetVel * SPINUP_TOLERANCE) {
            //             uint64_t ticks    = clockSyncGetTickCount();
            //             double   subPhase = clockSyncGetPhase();
            //             double   expected = (ticks + subPhase) / 24.0 * SUBDIVISION;
            //             s_beatOffset   = hornState.lastPosRevs - expected;
            //             s_beatTracking = true;
            //         }
            //     } else {
            //         uint64_t ticks    = clockSyncGetTickCount();
            //         double   subPhase = clockSyncGetPhase();
            //         double   expected = (ticks + subPhase) / 24.0 * SUBDIVISION + s_beatOffset;
            //         mm::PositionMode::Command hornCmd;
            //         hornCmd.position       = expected;
            //         hornCmd.velocity       = targetVel;
            //         hornCmd.accel_limit    = kNaN;
            //         hornCmd.velocity_limit = kNaN;
            //         hornCmd.kp_scale       = VEL_KP_SCALE;
            //         hornCmd.kd_scale       = VEL_KD_SCALE;
            //         hornCmd.ilimit_scale   = VEL_ILIMIT_SCALE;
            //         hornReplied = hornMoteus().SetPosition(hornCmd, &kFullFmt);
            //     }
            //     // Drum in velocity mode independently during BeatSync
            //     const double drumTargetRevS = drumRef.velRPM / 60.0;
            //     auto drumCmd = velocityCmd(drumTargetRevS, drumState.lastRefRevs);
            //     drumState.lastRefRevs = drumTargetRevS;
            //     drumReplied = drumMoteus().SetPosition(drumCmd, &kFullFmt);
            //     break;
            // }

            /*
            case ControlMode::Position: {
                // Needs position PID tuning before re-enabling.
                // Horn: snap to nearest whole revolution, hold with integral.
                // Drum: TBD.
                break;
            }
            */
        }

        // --- Update state from replies ---
        if (hornReplied) {
            const auto& v = hornMoteus().last_result().values;
            hornState.lastPosRevs       = v.position;
            hornState.lastSignedVelRevs = v.velocity;
            hornState.lastVelRevs       = fabs(v.velocity);
        }
        if (drumReplied) {
            const auto& v = drumMoteus().last_result().values;
            drumState.lastPosRevs       = v.position;
            drumState.lastSignedVelRevs = v.velocity;
            drumState.lastVelRevs       = fabs(v.velocity);
        }

        // --- Debug: print speed every second ---
        static uint32_t s_lastPrintMs = 0;
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (now - s_lastPrintMs >= 1000) {
            s_lastPrintMs = now;
            const char* modeName = "Velocity";
            Serial.printf("[ctrl] mode=%s horn=%.2f rev/s (%.1f RPM) drum=%.2f rev/s (%.1f RPM)\n",
                          modeName,
                          hornState.lastVelRevs, hornState.lastVelRevs * 60.0,
                          drumState.lastVelRevs, drumState.lastVelRevs * 60.0);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}
