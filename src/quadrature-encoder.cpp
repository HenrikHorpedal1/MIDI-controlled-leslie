#include "quadrature-encoder.h"
#include <math.h>

// ---------------- Pin & encoder configuration ----------------

static const uint8_t PIN_A = 16;   // TODO: choose correct pins
static const uint8_t PIN_B = 17;   // TODO: choose correct pins
static const uint8_t PIN_Z = 18;   // TODO: choose correct pins

constexpr float CPR = 24.0f;       // TODO: set to your detents/steps per rev

// ---------------- Ring buffer for A/B transitions -------------
// Size must be a power of 2 for the & (ENC_BUF_SIZE - 1) trick
static const uint8_t ENC_BUF_SIZE = 32;

volatile uint8_t encBuf[ENC_BUF_SIZE];
volatile uint8_t encHead = 0;
volatile uint8_t encTail = 0;

// Z pulse count (how many Z events not yet handled in the task)
volatile uint32_t zPulseCount = 0;

// Critical-section mux for all shared encoder data
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Task handle for encoder task (woken by ISRs)
TaskHandle_t encoderTaskHandle = nullptr;

// Incremental multi-turn count, updated only in encoderTask
volatile int32_t encoderCount = 0;

// ---------------- Shared state for other tasks ----------------
volatile EncoderState encoderState = {0, 0.0f, 0.0f, false};

void encoderTask(void *pvParameters);

void IRAM_ATTR onABEdge()
{
    static uint8_t lastAB = 0;

    uint8_t a = digitalRead(PIN_A) ? 1 : 0;
    uint8_t b = digitalRead(PIN_B) ? 1 : 0;
    uint8_t ab = (a << 1) | b;   // 0..3

    uint8_t transition = ((lastAB & 0x03) << 2) | ab; // oldAB(2) | newAB(2)
    lastAB = ab;

    BaseType_t hpTaskWoken = pdFALSE;

    portENTER_CRITICAL_ISR(&encoderMux);
    uint8_t head = encHead;
    uint8_t nextHead = (head + 1) & (ENC_BUF_SIZE - 1);
    if (nextHead != encTail) {
        encBuf[head] = transition;
        encHead = nextHead;
    }
    // else: buffer full, drop this event (rare if ENC_BUF_SIZE is reasonable)
    portEXIT_CRITICAL_ISR(&encoderMux);

    if (encoderTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(encoderTaskHandle, &hpTaskWoken);
        portYIELD_FROM_ISR(hpTaskWoken);
    }
}

// Z ISR: record that a Z event happened + notify encoder task
void IRAM_ATTR onZRising()
{
    BaseType_t hpTaskWoken = pdFALSE;

    portENTER_CRITICAL_ISR(&encoderMux);
    zPulseCount++;  // we only care that at least one occurred
    portEXIT_CRITICAL_ISR(&encoderMux);

    if (encoderTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(encoderTaskHandle, &hpTaskWoken);
        portYIELD_FROM_ISR(hpTaskWoken);
    }
}

// ---------------- Encoder task (A/B decoding + Z-based homing) ----------------
void encoderTask(void *pvParameters)
{
    (void) pvParameters;

    // Same TRANS table as original quadrature example
    static const int8_t TRANS[16] = {
         0, -1,  1, 14,
         1,  0, 14, -1,
        -1, 14,  0,  1,
        14,  1, -1,  0
    };

    static int lrsum = 0;

    // Homing-related state (lives only in this task)
    bool     homed         = false;
    bool     zAlignPending = false;  // "we saw Z, waiting for alignment A edge"
    int8_t   lastDir       = 0;      // +1 for CW, -1 for CCW
    int32_t  zeroOffset    = 0;      // encoderCount at home position

    Serial.println("Encoder task started");

    for (;;)
    {
        // Block until any ISR notifies us (A/B or Z)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // --- 1) Check if any Z pulses occurred since last time ---
        uint32_t zCountLocal = 0;
        portENTER_CRITICAL(&encoderMux);
        if (zPulseCount > 0) {
            zCountLocal = zPulseCount;
            zPulseCount = 0;
        }
        portEXIT_CRITICAL(&encoderMux);

        // If we haven't homed yet and saw at least one Z, arm alignment
        if (zCountLocal > 0 && !homed) {
            zAlignPending = true;
        }

        // --- 2) Process all pending A/B transitions in the ring buffer ---
        while (true)
        {
            uint8_t tail, head, idx;

            portENTER_CRITICAL(&encoderMux);
            tail = encTail;
            head = encHead;

            if (tail == head) {
                // Buffer empty
                portEXIT_CRITICAL(&encoderMux);
                break;
            }

            idx = encBuf[tail];
            encTail = (tail + 1) & (ENC_BUF_SIZE - 1);
            portEXIT_CRITICAL(&encoderMux);

            uint8_t oldAB = (idx >> 2) & 0x03;
            uint8_t newAB =  idx       & 0x03;

            lrsum += TRANS[idx];

            int8_t stepDir = 0;
            if (lrsum % 4 == 0) {
                if (lrsum == 4) {
                    lrsum = 0;
                    encoderCount++;
                    stepDir = +1;
                } else if (lrsum == -4) {
                    lrsum = 0;
                    encoderCount--;
                    stepDir = -1;
                } else {
                    lrsum = 0; // invalid sequence
                }
            }

            if (stepDir != 0) {
                lastDir = stepDir;
            }

            // --- A-edge alignment for precise Z homing ---
            // We implement A TRAILING only:
            //   trailing edge: falling if dir+, rising if dir-
            if (zAlignPending && lastDir != 0) {
                bool oldA = (oldAB >> 1) & 0x01;
                bool newA = (newAB >> 1) & 0x01;
                bool aRising  = (!oldA &&  newA);
                bool aFalling = ( oldA && !newA);

                bool hitAlignEdge = false;

                // A trailing:
                if ((lastDir > 0 && aFalling) ||
                    (lastDir < 0 && aRising)) {
                    hitAlignEdge = true;
                }

                if (hitAlignEdge) {
                    zeroOffset    = encoderCount;
                    homed         = true;
                    zAlignPending = false;

                    Serial.print("Homed at count = ");
                    Serial.println(zeroOffset);
                }
            }
        }

        // --- 3) Update shared EncoderState for other tasks ---
        float relAngle = (encoderCount * 360.0f) / CPR;

        float absAngle = relAngle;
        if (homed) {
            int32_t relCount = encoderCount - zeroOffset;
            absAngle = fmodf((relCount * 360.0f) / CPR, 360.0f);
            if (absAngle < 0.0f) absAngle += 360.0f;
        }

        portENTER_CRITICAL(&encoderMux);
        encoderState.count       = encoderCount;
        encoderState.relAngleDeg = relAngle;
        encoderState.absAngleDeg = absAngle;
        encoderState.homed       = homed;
        portEXIT_CRITICAL(&encoderMux);
    }
}

void encoderInit()
{
    // Configure pins
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);
    pinMode(PIN_Z, INPUT_PULLUP); // adjust if needed

    // Clear state
    encHead = encTail = 0;
    zPulseCount = 0;
    encoderCount = 0;
    encoderState = {0, 0.0f, 0.0f, false};

    // Create encoder task
    xTaskCreate(
        encoderTask,
        "EncoderTask",
        4096,
        nullptr,
        5,
        &encoderTaskHandle
    );

    // Attach interrupts AFTER encoderTaskHandle is valid
    attachInterrupt(digitalPinToInterrupt(PIN_A), onABEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), onABEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_Z), onZRising, RISING);

    Serial.println("Encoder init done");
}

void encoderGetState(EncoderState &out)
{
    portENTER_CRITICAL(&encoderMux);
    out = encoderState;
    portEXIT_CRITICAL(&encoderMux);
}

