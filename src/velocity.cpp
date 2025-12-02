#include "velocity.h"
#include "quadrature-encoder.h"

static float s_countsPerRev = 32.0f;        
static const uint32_t VEL_TIMEOUT_US = 200000; // 2.0s

static VelocityState g_vel = {0.0f, 0.0f, false};

static int32_t  s_lastCount      = 0;
static uint32_t s_lastEdgeMicros = 0;
static bool     s_firstSample    = true;

static portMUX_TYPE velMux = portMUX_INITIALIZER_UNLOCKED;

void velocityInit()
{
    uint32_t nowUs = micros();

    portENTER_CRITICAL(&velMux);
    g_vel.velCountsPerSec = 0.0f;
    g_vel.velRpm          = 0.0f;
    g_vel.valid           = false;
    s_lastCount           = 0;
    s_lastEdgeMicros      = nowUs;
    s_firstSample         = true;
    portEXIT_CRITICAL(&velMux);
}

void velocityUpdate()
{
    EncoderState enc;
    getEncoderState(enc);

    uint32_t nowUs = micros();

    // Local copies 
    int32_t  lastCount;
    uint32_t lastEdge;
    bool     first;

    portENTER_CRITICAL(&velMux);
    lastCount = s_lastCount;
    lastEdge  = s_lastEdgeMicros;
    first     = s_firstSample;
    portEXIT_CRITICAL(&velMux);

    if (first) {
        // Initialize on first call
        portENTER_CRITICAL(&velMux);
        s_firstSample      = false;
        s_lastCount        = enc.count;
        s_lastEdgeMicros   = nowUs;
        g_vel.velCountsPerSec = 0.0f;
        g_vel.velRpm          = 0.0f;
        g_vel.valid           = false;
        portEXIT_CRITICAL(&velMux);
        return;
    }

    VelocityState newVel;

    if (enc.count != lastCount) {
        int32_t  dCount = enc.count - lastCount;
        uint32_t dtUs   = nowUs - lastEdge;

        if (dtUs > 0) {
            float dt  = dtUs * 1e-6f;                       // seconds
            float cps = static_cast<float>(dCount) / dt;    // counts per second

            newVel.velCountsPerSec = cps;

            float revPerSec = cps / s_countsPerRev;
            newVel.velRpm   = revPerSec * 60.0f;

            newVel.valid    = enc.homed;
        } else {
            // dtUs == 0 
            newVel = g_vel;
        }

        portENTER_CRITICAL(&velMux);
        g_vel            = newVel;
        s_lastCount      = enc.count;
        s_lastEdgeMicros = nowUs;
        portEXIT_CRITICAL(&velMux);
    } else {
        // timeout to zero
        uint32_t ageUs = nowUs - lastEdge;
        if (ageUs > VEL_TIMEOUT_US) {
            portENTER_CRITICAL(&velMux);
            g_vel.velCountsPerSec = 0.0f;
            g_vel.velRpm          = 0.0f;
            g_vel.valid           = enc.homed;
            portEXIT_CRITICAL(&velMux);
        }
    }
}

void velocityGetState(VelocityState &out)
{
    portENTER_CRITICAL(&velMux);
    out = g_vel;
    portEXIT_CRITICAL(&velMux);
}
