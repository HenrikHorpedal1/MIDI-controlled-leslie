#include "velocity.h"
#include "quadrature-encoder.h"

static constexpr float    COUNTS_PER_REV   = 32.0f;   // must match CPR in encoder
static constexpr uint32_t VEL_TIMEOUT_MS   = 500;     // "no steps for 200ms" => stopped
static constexpr float    VEL_FILTER_ALPHA = 1.0;    // IIR low-pass factor (0..1)

struct VelEvent {
    int8_t   stepDir;   // +1 / -1
    uint32_t tsMicros;  // micros() timestamp of the step edge
};

static QueueHandle_t  s_velQueue      = nullptr;
static VelocityState  s_velState      = {0.0f, 0.0f, false};
static uint32_t       s_lastEdgeUs    = 0;
static bool           s_havePrevEdge  = false;

static portMUX_TYPE   s_velMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t   s_velTaskHandle = nullptr;

void velocityInit()
{
    s_velQueue = xQueueCreate(16, sizeof(VelEvent));

    if (s_velQueue == nullptr) {
        Serial.println("velocityInit: failed to create queue");
        return;
    }

    BaseType_t ok = xTaskCreate(
        velocityTask,
        "velocityTask",
        2048,
        nullptr,
        2,   // priority
        &s_velTaskHandle
    );

    if (ok != pdPASS) {
        Serial.println("velocityInit: failed to create task");
        s_velTaskHandle = nullptr;
    }
}

// Called from encoderTask context (NOT ISR).
void velocityPushStep(int8_t stepDir, uint32_t edgeMicros)
{
    if (s_velQueue == nullptr) {
        return;
    }

    VelEvent evt;
    evt.stepDir  = stepDir;
    evt.tsMicros = edgeMicros;

    // Don't block encoderTask; drop event if queue is full
    xQueueSend(s_velQueue, &evt, 0);
}

void velocityGetState(VelocityState &out)
{
    portENTER_CRITICAL(&s_velMux);
    out = s_velState;
    portEXIT_CRITICAL(&s_velMux);
}

// The velocity task: blocks on step events OR timeout.
void velocityTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("Velocity task started");

    const TickType_t timeoutTicks = pdMS_TO_TICKS(VEL_TIMEOUT_MS);

    for (;;)
    {
        VelEvent evt;
        bool gotEvent = (xQueueReceive(s_velQueue, &evt, timeoutTicks) == pdTRUE);

        if (gotEvent) {

            // First edge: initialise timestamp, but no velocity yet
            if (!s_havePrevEdge) {
                s_lastEdgeUs   = evt.tsMicros;
                s_havePrevEdge = true;
                continue;
            }

            uint32_t dtUs = evt.tsMicros - s_lastEdgeUs;
            if (dtUs == 0) {
                // micros() resolution edge case
                continue;
            }

            s_lastEdgeUs = evt.tsMicros;
            float dt     = dtUs * 1e-6f;                        // seconds
            float cpsInst = static_cast<float>(evt.stepDir) / dt; // counts/s

            // Get homing state so we can set validity
            EncoderState enc;
            getEncoderState(enc);

            portENTER_CRITICAL(&s_velMux);

            float cpsPrev = s_velState.velCountsPerSec;
            float cpsFilt;

            if (!s_velState.valid) {
                // First real velocity sample
                cpsFilt = cpsInst;
            } else {
                // First-order IIR low-pass filter
                cpsFilt = cpsPrev + VEL_FILTER_ALPHA * (cpsInst - cpsPrev);
            }

            s_velState.velCountsPerSec = cpsFilt;
            s_velState.velRpm          = (cpsFilt / COUNTS_PER_REV) * 60.0f;
            s_velState.valid           = enc.homed;   // only "valid" once homed

            portEXIT_CRITICAL(&s_velMux);
        } else {
            // Timeout: we haven't seen any step for VEL_TIMEOUT_MS
            EncoderState enc;
            getEncoderState(enc);

            portENTER_CRITICAL(&s_velMux);
            s_velState.velCountsPerSec = 0.0f;
            s_velState.velRpm          = 0.0f;
            s_velState.valid           = enc.homed;   // still only valid if homed
            portEXIT_CRITICAL(&s_velMux);

            // You *could* also reset s_havePrevEdge here if you want
            // the next step to re-initialize period estimation.
        }
    }
}
