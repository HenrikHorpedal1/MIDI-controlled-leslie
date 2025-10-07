#include <Arduino.h>

// ================= IR SENSOR (N marks per revolution) =================
#define INNER_IR_PIN 4                     // D4 on Nano ESP32 (GPIO4)
#define OUTER_IR_PIN 16

#define MARKS_PER_REV 8               // <<<--- set number of lines on drum here [TUNE]
#define RPM_TIMEOUT_MS 3000           // if no edges for this long -> feedback invalid

// Adaptive debounce clamps
#define DEBOUNCE_MIN_US 10000         // lower bound [TUNE]
#define DEBOUNCE_MAX_US 120000        // upper bound [TUNE]

// ================= PI CONTROLLER TARGET (SLOW MODE) ====================
#define TARGET_SLOW_RPM_X100 4000          // 40.00 RPM
#define TARGET_SLOW_RPM_X100 3000          // 300.00 RPM for now


// Slow-mode input guardrails
const int   MAX_SLOW_INPUT_PERCENT       = 30;
const int   MIN_SLOW_INPUT_PERCENT       = 0;
const int   SLOW_OPEN_LOOP_INPUT_PERCENT = 15;

// Fast-mode input guiderails
const int   MAX_FAST_INPUT_PERCENT       = 60;
const int   MIN_FAST_INPUT_PERCENT       = 0;


struct PID {
  const float Kp = 0.005f;
  const float Ki_per_s = 0.003f;
  const float Kd = 0.0f;
}

// ================ Shared IR state =================
portMUX_TYPE irMux = portMUX_INITIALIZER_UNLOCKED;

struct IrSensor {
  uint32_t lastEdgeUs   = 0;
  uint32_t prevEdgeUs   = 0;
  uint32_t lastPeriodUs = 0;
  uint32_t passCount    = 0;
  bool     edgeFlag     = false;
  uint32_t debounceUs   = 2000;
};

IrSensor InnerSensor;
IrSensor OuterSensor;


//volatile uint32_t ir_lastEdgeUs   = 0;
// volatile uint32_t ir_prevEdgeUs   = 0;
// volatile uint32_t ir_lastPeriodUs = 0;
// volatile uint32_t ir_passCount    = 0;
// volatile bool     ir_edgeFlag     = false;
// volatile uint32_t ir_debounceUs   = 2000;

volatile uint32_t ir_rpm_x100     = 0;
volatile uint32_t ir_fbMillis     = 0;

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void IRAM_ATTR onIrInnerEdge() {
  uint32_t now = micros();

  if ((uint32_t)(now - InnerSensor.lastEdgeUs) > InnerSensor.debounceUs) {
    if (InnerSensor.prevEdgeUs != 0) {
      InnerSensor.lastPeriodUs = now - InnerSensor.prevEdgeUs;
    } else {
      InnerSensor.lastPeriodUs = 0;
    }
    InnerSensor.prevEdgeUs = now;
    InnerSensor.lastEdgeUs = now;
    InnerSensor.passCount++;
    InnerSensor.edgeFlag = true;
  }
}

void IRAM_ATTR onIrOuterEdge() {
  uint32_t now = micros();

  // wrap-safe debounce check
  if ((uint32_t)(now - OuterSensor.lastEdgeUs) > OuterSensor.debounceUs) {
    if (OuterSensor.prevEdgeUs != 0) {
      OuterSensor.lastPeriodUs = now - OuterSensor.prevEdgeUs;
    } else {
      OuterSensor.lastPeriodUs = 0;
    }
    OuterSensor.prevEdgeUs = now;
    OuterSensor.lastEdgeUs = now;
    OuterSensor.passCount++;
    OuterSensor.edgeFlag = true;
  }
}



// Helper: print RPM without floats
static void printRpmLine(uint32_t pass, uint32_t periodUs) {
  if (periodUs == 0) {
    Serial.printf("[IR] pass=%lu  rpm=0.00  (period=%lu us)\n",
                  (unsigned long)pass, (unsigned long)periodUs);
    return;
  }

  // Adjust for multiple marks per revolution
  uint64_t rpm_x100_local = (6000000000ULL / (uint64_t)periodUs) / MARKS_PER_REV;

  uint32_t rpm_int  = (uint32_t)(rpm_x100_local / 100ULL);
  uint32_t rpm_frac = (uint32_t)(rpm_x100_local % 100ULL);
  Serial.printf("[IR] pass=%lu  rpm=%lu.%02lu  (period=%lu us)\n",
                (unsigned long)pass,
                (unsigned long)rpm_int,
                (unsigned long)rpm_frac,
                (unsigned long)periodUs);
}

// Task: adaptive debounce, compute RPM, print
void irRpmTask(void* pv) {
  uint32_t lastPrintedCount = 0;
  uint32_t lastPrintMillis  = 0;

  for (;;) {
    bool edge;
    uint32_t count, periodUs, lastEdgeUs_snapshot;

    noInterrupts();
    edge                 = ir_edgeFlag;
    ir_edgeFlag          = false;
    count                = ir_passCount;
    periodUs             = ir_lastPeriodUs;
    lastEdgeUs_snapshot  = ir_lastEdgeUs;
    interrupts();

    if (periodUs > 0) {
      uint32_t targetDebounce = periodUs / 10;
      targetDebounce = clamp_u32(targetDebounce, DEBOUNCE_MIN_US, DEBOUNCE_MAX_US);
      noInterrupts();
      ir_debounceUs = targetDebounce;
      interrupts();

      uint32_t rpm_x100_local = (uint32_t)((6000000000ULL / (uint64_t)periodUs) / MARKS_PER_REV);
      noInterrupts();
      ir_rpm_x100 = rpm_x100_local;
      ir_fbMillis = millis();
      interrupts();
    }

    uint32_t nowMs   = millis();
    bool haveAnyEdge = (lastEdgeUs_snapshot != 0);
    bool timeout     = haveAnyEdge && ((nowMs - (lastEdgeUs_snapshot / 1000)) > RPM_TIMEOUT_MS);

    if (edge) {
      printRpmLine(count, periodUs);
      lastPrintedCount = count;
      lastPrintMillis  = nowMs;
    } else if (timeout && (nowMs - lastPrintMillis > 250)) {
      Serial.printf("[IR] pass=%lu  rpm=0.00  (timeout)\n", (unsigned long)lastPrintedCount);
      lastPrintMillis = nowMs;
      noInterrupts();
      ir_rpm_x100 = 0;
      interrupts();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ================= SWITCHES + LEDS =================

// ================= MOTOR PWM & DIRECTION =================
const int pwmPin     = 27;
const int dirPin     = 25;

const int freq       = 1000;
const int resolution = 16;

const int fastSpeed = 150;

float     pi_integrator = 0.0f;
uint32_t  lastCtlMs     = 0;

static inline int clamp_int(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void toggleDirection() {
  static bool directionState = false;
  directionState = !directionState;

  if (directionState) {
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);
    Serial.println("Direction: LOW (Reversed?)");
  } else {
    pinMode(dirPin, INPUT_PULLUP);
    Serial.println("Direction: HIGH / Floated (Forward?)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), onIrEdge, FALLING);



  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);  // default reversed direction

  ledcAttach(pwmPin, freq, resolution);
  ledcWrite(pwmPin, 0);

  xTaskCreatePinnedToCore(switchAndLedTask, "SwLedTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(irRpmTask, "IrRpmTask", 3072, NULL, 2, NULL, 0);

  lastCtlMs = millis();

  Serial.println("Setup complete. Type 'D' to toggle direction.");
  Serial.printf("MARKS_PER_REV = %d\n", MARKS_PER_REV);
}


void PIDControllerTask(void *pv){
  float targetRPM;
  float errorRPM;



  for (;;){

  }
}

void loop() {
  bool motorOn   = switchA;
  bool fastMode  = switchB;
  int pwmCmd = 0;

  if (!motorOn) {
    pwmCmd = 0;
    pi_integrator = 0.0f;
  } else if (fastMode) {
    pwmCmd = fastSpeed;
  } else {
    uint32_t rpm_x100, fbMs;
    noInterrupts();
    rpm_x100 = ir_rpm_x100;
    fbMs     = ir_fbMillis;
    interrupts();

    uint32_t nowMs = millis();
    bool feedbackValid = (rpm_x100 > 0) && ((nowMs - fbMs) <= RPM_TIMEOUT_MS);

    float dt = (nowMs - lastCtlMs) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastCtlMs = nowMs;

    int32_t error_x100 = (int32_t)TARGET_RPM_X100 - (int32_t)rpm_x100;

    float P = Kp * (float)error_x100;
    float candidateIntegrator = pi_integrator;

    if (feedbackValid) {
      float I_increment = (Ki_per_s * dt) * (float)error_x100;
      float unsat = P + (pi_integrator + I_increment);
      bool wouldSaturateHigh = (unsat > SLOW_PWM_MAX) && (error_x100 > 0);
      bool wouldSaturateLow  = (unsat < SLOW_PWM_MIN) && (error_x100 < 0);
      if (!(wouldSaturateHigh || wouldSaturateLow)) candidateIntegrator += I_increment;
    } else candidateIntegrator *= 0.95f;

    if (candidateIntegrator > (float)SLOW_PWM_MAX) candidateIntegrator = (float)SLOW_PWM_MAX;
    if (candidateIntegrator < (float)(-SLOW_PWM_MAX)) candidateIntegrator = (float)(-SLOW_PWM_MAX);
    pi_integrator = candidateIntegrator;

    float u = P + pi_integrator;

    if (!feedbackValid)
      pwmCmd = clamp_int(SLOW_OPEN_LOOP_PWM, SLOW_PWM_MIN, SLOW_PWM_MAX);
    else {
      pwmCmd = (int)(u + 0.5f);
      pwmCmd = clamp_int(pwmCmd, SLOW_PWM_MIN, SLOW_PWM_MAX);
    }
  }

  ledcWrite(pwmPin, pwmCmd);

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("D")) toggleDirection();
  }

  delay(50);
}
