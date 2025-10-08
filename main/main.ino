#include <Arduino.h>

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
  //Serial.printf("MARKS_PER_REV = %d\n", MARKS_PER_REV);
}


// void PIDControllerTask(void *pv){
//   float targetRPM;
//   float errorRPM;
// 
// 
// 
//   for (;;){
// 
//   }
// }

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
