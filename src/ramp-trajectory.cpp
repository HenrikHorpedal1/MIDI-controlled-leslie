#include "ramp-trajectory.h"

#include <math.h>

#include "reference.h"
#include "velocity.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const float CHORALE_RPM   = 40.0f;
static const float TREMOLO_RPM   = 400.0f;
static const float RISE_TIME_SEC = 4.0f;
static const float FALL_TIME_SEC = 4.0f;

// Full-scale slopes (0 ↔ TREMOLO)
static const float RISE_SLOPE_RPM_PER_SEC = (TREMOLO_RPM - 0.0f) / RISE_TIME_SEC;
static const float FALL_SLOPE_RPM_PER_SEC = (TREMOLO_RPM - 0.0f) / FALL_TIME_SEC;

// Command passed to the ramp task
struct TrajCommand {
    float startRpm;
    float targetRpm;
    RefSource refSource;
};

static QueueHandle_t s_cmdQueue   = nullptr;
static float         s_lastRpm    = 0.0f;   // fallback if velocity isn't valid


static float targetForCommand(SpeedCommand cmd)
{
    switch (cmd) {
        case SpeedCommand::CHORALE: return CHORALE_RPM;
        case SpeedCommand::TREMOLO: return TREMOLO_RPM;
        case SpeedCommand::STOP:
        default:                    return 0.0f;
    }
}

// --- Task ---
static void rampTrajectoryTask(void *pvParameters)
{
    (void)pvParameters;

    const TickType_t stepTicks = pdMS_TO_TICKS(10);
    const float      dt        = 0.010f;
    const float      eps       = 1e-3f;

    for (;;) {
        TrajCommand cmd;
        if (xQueueReceive(s_cmdQueue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        float current = cmd.startRpm;
        float target  = cmd.targetRpm;
        RefSource src = cmd.refSource;

        // initial write
        {
            ReferenceState ref{};
            ref.velRPM = current;
            referenceSetFrom(src, ref);
        }

        while (true) {
            float delta = target - current;
            if (fabsf(delta) < eps) {
                current = target;

                ReferenceState ref{};
                ref.velRPM = current;
                referenceSetFrom(src, ref);

                s_lastRpm = current;
                break;
            }

            vTaskDelay(stepTicks);

            // preemption: new command?
            TrajCommand newCmd;
            if (xQueueReceive(s_cmdQueue, &newCmd, 0) == pdTRUE) {
                current  = newCmd.startRpm;
                target   = newCmd.targetRpm;
                src      = newCmd.refSource;

                ReferenceState ref{};
                ref.velRPM = current;
                referenceSetFrom(src, ref);
                continue;
            }

            float slope   = (delta > 0.0f) ? RISE_SLOPE_RPM_PER_SEC
                                           : FALL_SLOPE_RPM_PER_SEC;
            float maxStep = slope * dt;

            if (maxStep <= 0.0f) {
                current = target;
            } else {
                if (delta >  maxStep) delta =  maxStep;
                if (delta < -maxStep) delta = -maxStep;
                current += delta;
            }

            ReferenceState ref{};
            ref.velRPM = current;
            referenceSetFrom(src, ref);

            s_lastRpm = current;
        }
    }
}

void rampTrajectoryInit()
{
    if (s_cmdQueue == nullptr) {
        // Queue length 1 + xQueueOverwrite ⇒ "always keep latest command"
        s_cmdQueue = xQueueCreate(1, sizeof(TrajCommand));
    }
    s_lastRpm = 0.0f;
}

void rampTrajectoryStartTask(UBaseType_t priority)
{
    static TaskHandle_t taskHandle = nullptr;
    if (taskHandle != nullptr) {
        return;
    }

    xTaskCreate(
        rampTrajectoryTask,
        "RampTrajectoryTask",
        4096,
        nullptr,
        priority,
        &taskHandle
    );
}

void rampTrajectoryCommand(SpeedCommand cmd, RefSource src)
{
    if (s_cmdQueue == nullptr) return;

    VelocityState vel{};
    velocityGetState(vel);

    float start  = vel.valid ? vel.velRpm : s_lastRpm;
    float target = targetForCommand(cmd);

    TrajCommand t{ start, target, src };

    // overwrite any pending command; latest wins
    xQueueOverwrite(s_cmdQueue, &t);
}
