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
static constexpr double ACCEL_LIMIT_REV_S2  = 1.67;  // ~4s ramp to tremolo (6.67 rev/s)
static constexpr double SUBDIVISION         = 1.0;   // 1 = quarter note, 0.5 = half, 2 = 8th
static constexpr double STOP_THRESH_REV_S   = 0.1;  // ~6 RPM — below this, enter Position mode
static constexpr double SPINUP_TOLERANCE    = 0.15; // fraction of target vel — within 15% → lock

static const double kNaN = std::numeric_limits<double>::quiet_NaN();

// Format that enables accel_limit to be transmitted (default resolution is kIgnore)
static mm::PositionMode::Format makeAccelFmt() {
    mm::PositionMode::Format f;
    f.accel_limit = mm::kFloat;
    return f;
}
static const mm::PositionMode::Format kAccelFmt = makeAccelFmt();

enum class ControlMode { Velocity, BeatSync, Position };

void controllerTask(void* pvParameters) {
    if (!configureMoteus(Serial)) {
        Serial.println("Moteus init failed — controller task exiting");
        vTaskDelete(nullptr);
        return;
    }

    ControlMode mode         = ControlMode::Velocity;
    double s_lastPosRevs     = 0.0;
    double s_lastVelRevs     = 0.0;
    double s_homePos         = 0.0;
    double s_beatOffset      = 0.0;
    bool   s_beatTracking    = false;  // false = spinning up, true = position tracking

    for (;;) {
        ReferenceState ref;
        referenceGetActive(ref);

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
                    s_homePos = round(s_lastPosRevs);
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
                cmd.position    = kNaN;              // velocity-only
                cmd.velocity    = ref.velRPM / 60.0;
                cmd.accel_limit = ACCEL_LIMIT_REV_S2;
                replied = hornMoteus().SetPosition(cmd, &kAccelFmt);
                break;
            }

            case ControlMode::BeatSync: {
                const double targetVel = clockSyncGetBpm() * SUBDIVISION / 60.0;

                if (!s_beatTracking) {
                    // Spin-up: command target velocity, wait until close enough
                    cmd.position    = kNaN;
                    cmd.velocity    = targetVel;
                    cmd.accel_limit = ACCEL_LIMIT_REV_S2;
                    replied = hornMoteus().SetPosition(cmd, &kAccelFmt);

                    const double velErr = fabs(s_lastVelRevs - targetVel);
                    if (targetVel > 0.0 && velErr < targetVel * SPINUP_TOLERANCE) {
                        // Speed is close — compute offset and lock to position tracking
                        uint64_t ticks    = clockSyncGetTickCount();
                        double   subPhase = clockSyncGetPhase();
                        double   expected = (ticks + subPhase) / 24.0 * SUBDIVISION;
                        s_beatOffset   = s_lastPosRevs - expected;
                        s_beatTracking = true;
                    }
                } else {
                    // Position tracking: feed derived beat position to moteus
                    uint64_t ticks    = clockSyncGetTickCount();
                    double   subPhase = clockSyncGetPhase();
                    double   expected = (ticks + subPhase) / 24.0 * SUBDIVISION + s_beatOffset;

                    cmd.position = expected;
                    cmd.velocity = targetVel;  // feedforward
                    replied = hornMoteus().SetPosition(cmd);
                }
                break;
            }

            case ControlMode::Position: {
                if (ref.velRPM > 0.0f) {
                    // New speed requested — exit to Velocity
                    mode = ControlMode::Velocity;
                    cmd.position    = kNaN;
                    cmd.velocity    = ref.velRPM / 60.0;
                    cmd.accel_limit = ACCEL_LIMIT_REV_S2;
                } else {
                    cmd.position    = s_homePos;
                    cmd.velocity    = 0.0;
                    cmd.accel_limit = ACCEL_LIMIT_REV_S2;
                }
                replied = hornMoteus().SetPosition(cmd, &kAccelFmt);
                break;
            }
        }

        // --- Update state from reply ---
        if (replied) {
            const auto& v = hornMoteus().last_result().values;
            s_lastPosRevs = v.position;
            s_lastVelRevs = fabs(v.velocity);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}
