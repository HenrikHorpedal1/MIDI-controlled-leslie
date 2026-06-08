#include <Arduino.h>

#include "motor.h"
#include "quadrature-encoder.h"

// ---------------- Config ----------------

// How far to test (normalized command)
constexpr float U_START      = 0.0f;
constexpr float U_MAX_TEST   = 0.6f;    // don't go full blast; adjust if needed
constexpr float U_STEP       = 0.01f;   // resolution of search

// Movement detection
constexpr float ANGLE_THRESHOLD_DEG = 3.0f;   // how many degrees counts as "moved"
constexpr uint32_t TEST_DURATION_MS = 2000;   // time to watch for motion per test
constexpr uint32_t SAMPLE_INTERVAL_MS = 20;  // how often we sample encoder

// Small settle time between tests
constexpr uint32_t SETTLE_MS = 2000;

// ---------------- Helpers ----------------

static bool detectMovement(float uCmd)
{
    EncoderState start{};
    EncoderState cur{};

    // Read starting angle
    getEncoderState(start);

    // Command motor
    motorSetNormalized(uCmd);

    uint32_t t0 = millis();

    while (millis() - t0 < TEST_DURATION_MS) {
        delay(SAMPLE_INTERVAL_MS);

        getEncoderState(cur);

        // Use relAngleDeg so we don't care about homing / wrapping
        float dAngle = cur.relAngleDeg - start.relAngleDeg;

        if (fabsf(dAngle) > ANGLE_THRESHOLD_DEG) {
            // Movement detected
            motorSetNormalized(0.0f);
            return true;
        }
    }

    // No movement detected for this command
    motorSetNormalized(0.0f);
    return false;
}

/**
 * Find smallest |u| that causes motion in given direction (+1 forward, -1 reverse).
 * Returns positive magnitude (e.g. 0.12), direction is implied by sign argument.
 */
static float findStaticU(int direction)
{
    Serial.println();
    Serial.printf("Finding U_STATIC for direction %s...\n",
                  (direction > 0) ? "FORWARD" : "REVERSE");

    // Ensure we start from rest
    motorSetNormalized(0.0f);
    delay(SETTLE_MS);

    for (float uMag = U_START; uMag <= U_MAX_TEST + 1e-6f; uMag += U_STEP) {
        float uCmd = direction * uMag;

        Serial.printf("  Testing u = %.3f\n", uCmd);

        // Let things settle before each test
        motorSetNormalized(0.0f);
        delay(SETTLE_MS);

        bool moved = detectMovement(uCmd);

        if (moved) {
            Serial.printf("  Movement detected at u = %.3f\n", uCmd);
            return uMag; // return magnitude
        }
    }

    Serial.println("  Reached U_MAX_TEST without detected motion.");
    return U_MAX_TEST;
}

// ---------------- Arduino setup/loop ----------------

void setup()
{
    Serial.begin(115200);
    // Give USB serial some time (optional)
    delay(2000);

    Serial.println();
    Serial.println("=== U_STATIC Calibration ===");

    motorInit();
    // Start your encoder task / init here. Adjust the name if different.
    // For example:
    encoderInit();

    // Give encoder task some time to start and stabilize
    delay(500);

    // Optionally, you can print homing status:
    EncoderState enc{};
    getEncoderState(enc);
    Serial.printf("Encoder homed: %s\n", enc.homed ? "YES" : "NO");
    Serial.println("Starting search. Make sure the rotor can spin freely.");

    // Forward direction (+1)
    float uStaticFwd = findStaticU(+1);

    // Reverse direction (-1)
    float uStaticRev = findStaticU(-1);

    Serial.println();
    Serial.println("=== U_STATIC Results ===");
    Serial.printf("U_STATIC_FORWARD ≈ %.3f\n", uStaticFwd);
    Serial.printf("U_STATIC_REVERSE ≈ %.3f\n", uStaticRev);
    Serial.println("You can now plug these into your controller as feedforward/deadzone.");

    // Stop motor
    motorSetNormalized(0.0f);
}

void loop()
{
    // Nothing to do; calibration ran in setup().
    // Keep loop idle.
    delay(1000);
}
