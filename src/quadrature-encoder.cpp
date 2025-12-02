#include "quadrature-encoder.h"
#include <math.h>

static const uint8_t PIN_A = 10;   
static const uint8_t PIN_B = 17;   
static const uint8_t PIN_Z = 18;   

constexpr float CPR = 32;

static const uint8_t ENC_BUF_SIZE = 32;

volatile uint8_t encBuf[ENC_BUF_SIZE];
volatile uint8_t encHead = 0;
volatile uint8_t encTail = 0;

volatile uint32_t zPulseCount = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t encoderTaskHandle = nullptr;

volatile int32_t encoderCount = 0;

// ---------------- Shared state for other tasks ----------------
EncoderState encoderState = {0, 0.0f, 0.0f, false};

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

    portEXIT_CRITICAL_ISR(&encoderMux);

    if (encoderTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(encoderTaskHandle, &hpTaskWoken);
        portYIELD_FROM_ISR(hpTaskWoken);
    }
}

void IRAM_ATTR onZRising()
{
    BaseType_t hpTaskWoken = pdFALSE;

    portENTER_CRITICAL_ISR(&encoderMux);
    zPulseCount++;  
    portEXIT_CRITICAL_ISR(&encoderMux);

    if (encoderTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(encoderTaskHandle, &hpTaskWoken);
        portYIELD_FROM_ISR(hpTaskWoken);
    }
}

void encoderTask(void *pvParameters)
{
    (void) pvParameters;

    static const int8_t TRANS[16] = {
         0, -1,  1, 14,
         1,  0, 14, -1,
        -1, 14,  0,  1,
        14,  1, -1,  0
    };

    static int lrsum = 0;

    bool     homed         = false;
    bool     zAlignPending = false;  
    int8_t   lastDir       = 0;      
    int32_t  zeroOffset    = 0;      

    Serial.println("Encoder task started");

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t zCountLocal = 0;
        portENTER_CRITICAL(&encoderMux);
        if (zPulseCount > 0) {
            zCountLocal = zPulseCount;
            zPulseCount = 0;
        }
        portEXIT_CRITICAL(&encoderMux);

        if (zCountLocal > 0 && !homed) {
            zAlignPending = true;
        }

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

        // Update shared EncoderState for other tasks ---
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
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);
    pinMode(PIN_Z, INPUT_PULLUP); 

    encHead = encTail = 0;
    zPulseCount = 0;
    encoderCount = 0;
    encoderState = {0, 0.0f, 0.0f, false};

    xTaskCreate(
        encoderTask,
        "EncoderTask",
        4096,
        nullptr,
        5,
        &encoderTaskHandle
    );

    attachInterrupt(digitalPinToInterrupt(PIN_A), onABEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), onABEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_Z), onZRising, RISING);

    Serial.println("Encoder init done");
}

void getEncoderState(EncoderState &out)
{
    portENTER_CRITICAL(&encoderMux);
    out = encoderState;
    portEXIT_CRITICAL(&encoderMux);
}

