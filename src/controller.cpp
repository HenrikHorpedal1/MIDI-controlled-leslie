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

// Tuning constants
static constexpr double ACCEL_LIMIT_UP_REV_S2   = 1.67;  // ~4s ramp up to tremolo (6.67 rev/s)
static constexpr double ACCEL_LIMIT_DOWN_REV_S2 = 2.50;  // ~2.7s ramp down from tremolo
static constexpr double ACCEL_LIMIT_BEAT_REV_S2 = 1.67;  // spin-up accel in BeatSync mode
static constexpr double ACCEL_LIMIT_STOP_REV_S2 = 0.50;  // gentle decel into position hold
static constexpr double VELOCITY_LIMIT_STOP     = 0.5;   // rev/s max speed when homing to position
static constexpr double SUBDIVISION             = 1.0;   // 1 = quarter note, 0.5 = half, 2 = 8th
static constexpr double STOP_THRESH_REV_S   = 0.4;  // ~6 RPM — below this, enter Position mode
static constexpr double SPINUP_TOLERANCE    = 0.15; // fraction of target vel — within 15% → lock

// Velocity-mode gain scales — target kp=0.5, ki=0, kd=0.5 against the base gains from moteus-config
static constexpr double VEL_KP_SCALE     = 0.5 / MOTEUS_BASE_KP;  // ≈ 0.0714
static constexpr double VEL_KI_SCALE     = 0.0;                    // integral disabled
static constexpr double VEL_KD_SCALE     = 0.5 / MOTEUS_BASE_KD;  // = 1.25

static const double kNaN = std::numeric_limits<double>::quiet_NaN();

// Full format: transmits all gain-scale and limit fields every cycle
static mm::PositionMode::Format makeFullFmt() {
    mm::PositionMode::Format f;
    f.accel_limit    = mm::kFloat;
    f.velocity_limit = mm::kFloat;
    f.kp_scale       = mm::kFloat;
    f.kd_scale       = mm::kFloat;
    f.ilimit_scale   = mm::kFloat;
    return f;
}
static const mm::PositionMode::Format kFullFmt = makeFullFmt();

enum class ControlMode { Velocity, BeatSync, Position };

void controllerTask(void* pvParameters) {
    if (!configureMoteus(Serial)) {
        Serial.println("Moteus init failed — controller task exiting");
        vTaskDelete(nullptr);
        return;
    }

    ControlMode mode              = ControlMode::Velocity;
    double s_lastPosRevs          = 0.0;
    double s_lastVelRevs          = 0.0;  // magnitude, used for thresholds and display
    double s_lastSignedVelRevs    = 0.0;  // signed, used to determine direction when homing
    double s_lastRefRevs          = 0.0;  // previous reference velocity — used for accel limit selection
    double s_homePos              = 0.0;
    double s_beatOffset           = 0.0;
    bool   s_beatTracking         = false;  // false = spinning up, true = position tracking

    for (;;) {
        ReferenceState ref;
        referenceGet(ref);

        const bool clockActive = clockSyncIsRunning() && clockSyncIsLocked();

        // --- Mode selection ---
        if (clockActive) {
            if (mode != ControlMode::BeatSync) {
                // Entering BeatSync — start in spin-up sub-state
                s_beatTracking = false;
                mode = ControlMode::BeatSync;
            }
        } else {
            if (mode == ControlMode::BeatSync) {
                mode = ControlMode::Velocity;
                s_beatTracking = false;
            }
            if (ref.velRPM == 0.0f) {
                if (mode != ControlMode::Position && s_lastVelRevs < STOP_THRESH_REV_S) {
                    // Snap to the next whole revolution continuing in the current direction
                    s_homePos = (s_lastSignedVelRevs >= 0.0) ? ceil(s_lastPosRevs)
                                                              : floor(s_lastPosRevs);
                    mode = ControlMode::Position;
                }
            } else {
                if (mode == ControlMode::Position) {
                    mode = ControlMode::Velocity;
                }
            }
        }

        // --- Command ---
        mm::PositionMode::Command cmd;
        bool replied = false;

        switch (mode) {

            case ControlMode::Velocity: {
                const double targetVel = ref.velRPM / 60.0;
                cmd.position       = kNaN;
                cmd.velocity       = targetVel;
                cmd.accel_limit    = (targetVel >= s_lastRefRevs) ? ACCEL_LIMIT_UP_REV_S2
                                                                   : ACCEL_LIMIT_DOWN_REV_S2;
                cmd.velocity_limit = kNaN;
                cmd.kp_scale       = VEL_KP_SCALE;
                cmd.kd_scale       = VEL_KD_SCALE;
                cmd.ilimit_scale   = VEL_KI_SCALE;
                s_lastRefRevs = targetVel;
                replied = hornMoteus().SetPosition(cmd, &kFullFmt);
                break;
            }

            case ControlMode::BeatSync: {
                const double targetVel = clockSyncGetBpm() * SUBDIVISION / 60.0;

                if (!s_beatTracking) {
                    // Spin-up: velocity mode gains
                    cmd.position       = kNaN;
                    cmd.velocity       = targetVel;
                    cmd.accel_limit    = ACCEL_LIMIT_BEAT_REV_S2;
                    cmd.velocity_limit = kNaN;
                    cmd.kp_scale       = VEL_KP_SCALE;
                    cmd.kd_scale       = VEL_KD_SCALE;
                    cmd.ilimit_scale   = VEL_KI_SCALE;
                    replied = hornMoteus().SetPosition(cmd, &kFullFmt);

                    const double velErr = fabs(s_lastVelRevs - targetVel);
                    if (targetVel > 0.0 && velErr < targetVel * SPINUP_TOLERANCE) {
                        uint64_t ticks    = clockSyncGetTickCount();
                        double   subPhase = clockSyncGetPhase();
                        double   expected = (ticks + subPhase) / 24.0 * SUBDIVISION;
                        s_beatOffset   = s_lastPosRevs - expected;
                        s_beatTracking = true;
                    }
                } else {
                    // Position tracking: gains TBD
                    uint64_t ticks    = clockSyncGetTickCount();
                    double   subPhase = clockSyncGetPhase();
                    double   expected = (ticks + subPhase) / 24.0 * SUBDIVISION + s_beatOffset;

                    cmd.position       = expected;
                    cmd.velocity       = targetVel;
                    cmd.accel_limit    = kNaN;
                    cmd.velocity_limit = kNaN;
                    cmd.kp_scale       = VEL_KP_SCALE;
                    cmd.kd_scale       = VEL_KD_SCALE;
                    cmd.ilimit_scale   = VEL_KI_SCALE;
                    replied = hornMoteus().SetPosition(cmd, &kFullFmt);
                }
                break;
            }

            case ControlMode::Position: {
                if (ref.velRPM > 0.0f) {
                    // New speed requested — exit to Velocity
                    mode = ControlMode::Velocity;
                    cmd.position       = kNaN;
                    cmd.velocity       = ref.velRPM / 60.0;
                    cmd.accel_limit    = ACCEL_LIMIT_UP_REV_S2;
                    cmd.velocity_limit = kNaN;
                    cmd.kp_scale       = VEL_KP_SCALE;
                    cmd.kd_scale       = VEL_KD_SCALE;
                    cmd.ilimit_scale   = VEL_KI_SCALE;
                } else {
                    cmd.position       = s_homePos;
                    cmd.velocity       = 0.0;
                    cmd.accel_limit    = ACCEL_LIMIT_STOP_REV_S2;
                    cmd.velocity_limit = VELOCITY_LIMIT_STOP;
                    cmd.kp_scale       = 1.0;
                    cmd.kd_scale       = 1.0;
                    cmd.ilimit_scale   = 1.0;
                }
                replied = hornMoteus().SetPosition(cmd, &kFullFmt);
                break;
            }
        }

        // --- Update state from reply ---
        if (replied) {
            const auto& v = hornMoteus().last_result().values;
            s_lastPosRevs       = v.position;
            s_lastSignedVelRevs = v.velocity;
            s_lastVelRevs       = fabs(v.velocity);
        }

        // --- Debug: print speed every second ---
        static uint32_t s_lastPrintMs = 0;
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (now - s_lastPrintMs >= 1000) {
            s_lastPrintMs = now;
            const char* modeName = (mode == ControlMode::Velocity) ? "Velocity"
                                 : (mode == ControlMode::BeatSync)  ? "BeatSync"
                                                                     : "Position";
            Serial.printf("[ctrl] mode=%s pos=%.3f rev vel=%.2f rev/s (%.1f RPM)\n",
                          modeName, s_lastPosRevs, s_lastVelRevs, s_lastVelRevs * 60.0);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}
