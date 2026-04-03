#include <Arduino.h>
#include <math.h>

#include "moteus-config.h"
#include "clock_sync.h"
#include "midi-listner.h"
#include "input_event.h"

// ------------------------------------------------------------
// MIDI task infrastructure
// ------------------------------------------------------------

static QueueHandle_t inputQueue;
static QueueHandle_t clockQueue;
static MidiTaskParams midiParams;

// ------------------------------------------------------------
// User tuning
// ------------------------------------------------------------

// One full HORN rotation per quarter note
constexpr double kHornTurnsPerBeat = 1.0;

// Load pulley is 3.5x bigger than motor pulley.
// Therefore motor must rotate 3.5 turns for 1 horn/load turn.
constexpr double kHornMotorTurnsPerHornTurn = 3.5;

// Optional phase offset in HORN turns
// 0.25 = 90 degrees, 0.5 = 180 degrees
constexpr double kHornPhaseOffsetTurns = 0.0;

// Command horn update rate
constexpr uint32_t kCommandPeriodUs = 5000;   // 200 Hz

// Debug print rate
constexpr uint32_t kPrintPeriodMs = 100;

// Only move when the MIDI clock estimator says lock is good
constexpr bool kRequireClockLock = true;

// ------------------------------------------------------------
// State
// ------------------------------------------------------------

static uint32_t g_nextCommandUs = 0;
static uint32_t g_lastPrintMs = 0;
static bool g_sentStop = false;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

static double getQuarterNoteTurns() {
  // 24 MIDI clock ticks per quarter note
  const double ticks = static_cast<double>(clockSyncGetTickCount());
  const double frac  = static_cast<double>(clockSyncGetPhase());
  return (ticks + frac) / 24.0;
}

static bool shouldRunHorn() {
  if (!clockSyncIsRunning()) {
    return false;
  }
  if (kRequireClockLock && !clockSyncIsLocked()) {
    return false;
  }
  return true;
}

static void stopHorn() {
  const bool ok = hornMoteus().SetStop();
  if (!ok) {
    Serial.println("horn SetStop() timeout/no reply");
  }
}

static void commandHornFromClock() {
  const double bpm = clockSyncGetBpm();
  const double quarterNoteTurns = getQuarterNoteTurns();

  // Desired horn/load position in turns
  const double hornTargetTurns =
      quarterNoteTurns * kHornTurnsPerBeat + kHornPhaseOffsetTurns;

  // Convert horn/load turns -> motor turns
  const double motorTargetTurns =
      hornTargetTurns * kHornMotorTurnsPerHornTurn;

  // Desired horn/load speed in turns/sec
  const double hornVelocityTurnsPerSec =
      (bpm / 60.0) * kHornTurnsPerBeat;

  // Convert horn/load speed -> motor speed
  const double motorVelocityTurnsPerSec =
      hornVelocityTurnsPerSec * kHornMotorTurnsPerHornTurn;

  Moteus::PositionMode::Command cmd;
  cmd.position = motorTargetTurns;
  cmd.velocity = motorVelocityTurnsPerSec;

  const bool ok = hornMoteus().SetPosition(cmd);
  if (!ok) {
    Serial.println("horn SetPosition() timeout/no reply");
  }
}

// ------------------------------------------------------------
// Arduino setup / loop
// ------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Queues
  inputQueue = xQueueCreate(32, sizeof(InputEvent));
  clockQueue = xQueueCreate(128, sizeof(ClockMsg));  // 24ppqn; 128 is plenty

  midiParams = { inputQueue, clockQueue };

  // MIDI parser / listener task
  BaseType_t midiOk = xTaskCreatePinnedToCore(
      midiListnerTask,
      "midi",
      4096,
      &midiParams,
      3,
      nullptr,
      1);

  // Clock sync estimator task
  BaseType_t clkOk = xTaskCreatePinnedToCore(
      clockSyncTask,
      "clk",
      4096,
      clockQueue,
      2,
      nullptr,
      1);

  if (midiOk != pdPASS) {
    Serial.println("Failed to create midi task");
    while (true) {
      delay(1000);
    }
  }

  if (clkOk != pdPASS) {
    Serial.println("Failed to create clock sync task");
    while (true) {
      delay(1000);
    }
  }

  const bool moteusOk = configureMoteus(Serial);
  if (!moteusOk) {
    Serial.println("configureMoteus() failed");
  }

  g_nextCommandUs = micros();
  g_lastPrintMs = millis();

  Serial.println("Ready");
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // Run horn command loop at fixed rate
  if ((int32_t)(nowUs - g_nextCommandUs) >= 0) {
    g_nextCommandUs += kCommandPeriodUs;

    if (!shouldRunHorn()) {
      if (!g_sentStop) {
        stopHorn();
        g_sentStop = true;
      }
    } else {
      g_sentStop = false;
      commandHornFromClock();
    }
  }

  if ((uint32_t)(nowMs - g_lastPrintMs) >= kPrintPeriodMs) {
    g_lastPrintMs += kPrintPeriodMs;

    Serial.printf(
      "run=%d lock=%d bpm=%.2f period=%.0fus err=%.0fus phase=%.2f tick=%llu qn=%.4f\n",
      (int)clockSyncIsRunning(),
      (int)clockSyncIsLocked(),
      clockSyncGetBpm(),
      clockSyncGetPeriodUs(),
      clockSyncGetLastErrUs(),
      clockSyncGetPhase(),
      (unsigned long long)clockSyncGetTickCount(),
      getQuarterNoteTurns()
    );
  }

  delay(1);
}