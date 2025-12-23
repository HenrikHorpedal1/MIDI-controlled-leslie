#include <Arduino.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "quadrature-encoder.h"
#include "velocity.h"
#include "reference.h"
#include "motor.h"


#include "sd_logger.h"

// Control loop period
static constexpr float      CTRL_DT_S          = 0.001f;          // 1 kHz
static constexpr TickType_t CTRL_PERIOD_TICKS  = pdMS_TO_TICKS(1);

// Unit conversions
static constexpr float DEG_PER_SEC_PER_RPM = 360.0f / 60.0f;      // 6.0
static constexpr float VEL_THRESH_RPM      = 15.0f;
//static constexpr float VEL_THRESH_DEG_S    = VEL_THRESH_RPM * DEG_PER_SEC_PER_RPM; // 180 deg/s

// “Zero” velocity reference threshold (deg/s)
static constexpr float VEL_REF_ZERO_THRES_RPM = 10.0f;             // ~0.17 rpm

// Homing open-loop speed (normalized, -1..1)
static constexpr float HOMING_U = 0.14f;                           // ~10% of MAX_INPUT_PWM

static constexpr float VEL_KP = 0.01f;
static constexpr float VEL_KI = 0.005f;

static constexpr float POS_KP = 0.00000001f;
static constexpr float POS_KD = 0.00000000001f;

static constexpr float U_STATIC_FWD = 0.070f;    // forward direction
static constexpr float U_STATIC_REV = 0.070f;    // reverse direction
static constexpr float POS_FF_DEADBAND_DEG = 22.5f;

// Integrator clamp
static constexpr float INTEGRATOR_MAX = U_STATIC_FWD;
static constexpr float INTEGRATOR_MIN = -U_STATIC_REV;


static inline float clampFloat(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float wrapAngleErrorDeg(float e)
{
    // Wrap into [-180, 180]
    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

// ---------------- Control mode ----------------

enum class ControlMode : uint8_t {
    Homing,
    Velocity,
    Position
};

static const char* controlModeToString(ControlMode mode)
{
    switch (mode) {
        case ControlMode::Homing:   return "Homing";
        case ControlMode::Velocity: return "Velocity";
        case ControlMode::Position: return "Position";
        default:                    return "Unknown";
    }
}

static ControlMode selectControlMode(const EncoderState   &enc,
                                     const VelocityState  &vel,
                                     const ReferenceState &ref)
{
    // 1) Not homed yet - homing mode
    if (!enc.homed) {
        return ControlMode::Homing;
    }

    // 2) Homed, but something is “not ready” - just go to zero position.
    if (!vel.valid) {
        return ControlMode::Position;
    }

    const bool refIsZero  = fabsf(ref.velRPM)  < VEL_REF_ZERO_THRES_RPM;
    const bool speedIsLow = fabsf(vel.velRpm) < VEL_THRESH_RPM;

    if (!refIsZero) {
        return ControlMode::Velocity;
    }

    //   still moving fast - velocity mode (brake)
    //   slow enough       - position mode (hold angle)
    if (!speedIsLow) {
        return ControlMode::Velocity;
    }

    return ControlMode::Position;
}


static void plotController(float ref,
                           float measurement,
                           float error,
                           float input,
                           float P,
                           float I,
                           float D)
{
    // don't spam the USB
    static uint32_t counter = 0;
    constexpr uint32_t PLOT_EVERY_N = 10; 

    if (++counter < PLOT_EVERY_N) {
        return;
    }
    counter = 0;

    Serial.print("Ref:");           Serial.print(ref);          Serial.print(",");
    Serial.print("Measurement:");   Serial.print(measurement);  Serial.print(",");
    Serial.print("err:");           Serial.print(error);        Serial.print(",");
    Serial.print("u:");             Serial.print(input);        Serial.print(",");
    Serial.print("P:");             Serial.print(P);            Serial.print(",");
    Serial.print("I:");             Serial.print(I);            Serial.print(",");
    Serial.print("D:");             Serial.println(D);            
}

void controllerTask(void *pvParameters)
{
    (void)pvParameters;

    EncoderState   enc{};
    VelocityState  vel{};
    ReferenceState ref{};

    float velInt = 0.0f;   // integrator for velocity loop
    float posInt = 0.0f;   // integrator for position loop

    TickType_t lastWakeTime = xTaskGetTickCount();

    uint32_t   loopCounter  = 0;

    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, CTRL_PERIOD_TICKS);

        getEncoderState(enc);
        velocityGetState(vel);
        referenceGetActive(ref);

        ControlMode mode = selectControlMode(enc, vel, ref);

        float u = 0.0f;
        float e = 0.0f;
        float P = 0.0f;
        float I = 0.0f;
        float D = 0.0f;

        switch (mode)
        {
            case ControlMode::Homing:
            {
                velInt = 0.0f;
                posInt = 0.0f;

                motorSetNormalized(HOMING_U);
                plotController(ref.angleDeg, enc.absAngleDeg, 0.0f, HOMING_U,0.0f,0.0f, 0.0f);
                continue;
            }

            case ControlMode::Velocity:
            {

                float e = ref.velRPM - vel.velRpm; 
                float P = VEL_KP * e;
                velInt += e * CTRL_DT_S;
                float D = 0.0f;
                float I = velInt*VEL_KI;
                float u_pi = P + I;

                float u_ff = 0.0f;

                //if (e > 0.0f) {
                //    // Need to move in positive direction
                //    u_ff = U_STATIC_FWD;
                //} else {
                //    // Need to move in negative direction
                //    u_ff = -U_STATIC_REV;
                //}


                //velInt += e * CTRL_DT_S;
                //velInt = clampFloat(velInt, INTEGRATOR_MIN, INTEGRATOR_MAX);

                float u_unsat = u_pi + u_ff;
                u = clampFloat(u_unsat, -1.0f, 1.0f);
                motorSetNormalized(u);
                plotController(ref.velRPM, vel.velRpm, e, u, P, I, D);

                break;
            }

            case ControlMode::Position:
            {
                const float posRef_deg  = ref.angleDeg;
                const float posMeas_deg = enc.absAngleDeg;
                const float vel_rpm = vel.velRpm;

                float e = posRef_deg - posMeas_deg;
                e = wrapAngleErrorDeg(e);
                float P = POS_KP * e;


                float D = -POS_KD*vel_rpm;

                float u_pi = P + D;
                float u_ff = 0.0f;

                // Only apply FF when we are "meaningfully away" from target
                if (fabsf(e) >= POS_FF_DEADBAND_DEG) {
                    if (e > 0.0f) {
                        u_ff = U_STATIC_FWD;
                    } else {
                        u_ff = -U_STATIC_REV;
                    }
                } else {
                    u_pi = 0.0f;
                    }

                float u_unsat = u_pi + u_ff;
                u = clampFloat(u_unsat, -1.0f, 1.0f);

                plotController(ref.angleDeg, enc.absAngleDeg, e, u, P, I, D);
                motorSetNormalized(u);
                break;
            }
        }

        sdLoggerLogControllerSample(
                    controlModeToString(mode),
                    ref.angleDeg,
                    ref.velRPM,
                    enc.absAngleDeg,
                    vel.velRpm,
                    e,
                    u,
                    P,
                    I,
                    D,
                    loopCounter
            );

        ++loopCounter;
    }
}

