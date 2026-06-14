// beat-sync-test.ino
//
// MIDI-only test of the tempo-sync *subdivision* logic. No motor control,
// no moteus/CAN required — so it runs on a bare Nano ESP32 over USB MIDI.
//
// It runs the same two tasks the firmware uses (MIDI listener + clock-sync
// PLL), then reproduces the exact speed/position math from controller.cpp's
// BeatSync mode and prints it. Use it to confirm that selecting a subdivision
// actually changes the commanded rotor speed before wiring up the motor.
//
// Drive it from a DAW / MIDI clock source:
//   * Transport Start / Clock / Stop  -> sets tempo + run/lock state
//   * CC MIDI_SUBDIV_CC on channel 1, value 0..11 -> selects 1/2 ... 1/32T
//     (0=1/2, 1=1/4, 2=1/8., 3=1/4T, 4=1/8, 5=1/16., 6=1/8T, 7=1/16,
//      8=1/32., 9=1/16T, 10=1/32, 11=1/32T)

#include <Arduino.h>
#include <math.h>

#include "beat_sync.h"
#include "clock_sync.h"
#include "input_event.h"
#include "midi-listner.h"

// Mirrors controller.cpp's BEAT_HOME_FRAC: the rotor face fraction to align with
// the beat. One rotor revolution per synced subdivision cycle (RPM == BPM at
// quarter notes). Kept in sync manually so this sketch needs no motor code.
static constexpr double kBeatHomeFrac = 0.0;

static constexpr uint32_t kPrintPeriodMs = 200;

static QueueHandle_t inputQueue;
static QueueHandle_t clockQueue;
static MidiTaskParams midiParams;

static uint32_t g_lastPrintMs = 0;

// Mirrors controller.cpp's per-rotor BeatSync wrap state (no encoder here).
static double g_beatPrevCmd = 0.0;
static bool   g_beatInit    = false;

// Snapshots taken at the previous print, so we can show how far clockPos vs the
// wrapped command moved between prints (the key contrast on a playhead jump).
static double g_clockPosAtLastPrint = 0.0;
static double g_cmdAtLastPrint      = 0.0;

static const char *subdivisionName(Subdivision s) {
  switch (s) {
  case Subdivision::Half:                return "1/2";
  case Subdivision::Quarter:             return "1/4";
  case Subdivision::EighthDotted:        return "1/8.";
  case Subdivision::QuarterTriplet:      return "1/4T";
  case Subdivision::Eighth:              return "1/8";
  case Subdivision::SixteenthDotted:     return "1/16.";
  case Subdivision::EighthTriplet:       return "1/8T";
  case Subdivision::Sixteenth:           return "1/16";
  case Subdivision::ThirtySecondDotted:  return "1/32.";
  case Subdivision::SixteenthTriplet:    return "1/16T";
  case Subdivision::ThirtySecond:        return "1/32";
  case Subdivision::ThirtySecondTriplet: return "1/32T";
  }
  return "?";
}

// Drain the InputEvent queue and apply any subdivision-select CC. This is the
// same path input_handler uses in beat-sync mode, minus the motor reference.
static void pumpInputQueue() {
  InputEvent ev;
  while (xQueueReceive(inputQueue, &ev, 0) == pdTRUE) {
    if (ev.type == EventType::MidiCC &&
        ev.data.midiCC.control == MIDI_SUBDIV_CC) {
      beatSyncSetSubdivisionFromCC(ev.data.midiCC.value);
      Serial.printf("[subdiv] CC value=%u -> %s (horn %u, drum %u ticks/rev)\n",
                    ev.data.midiCC.value,
                    subdivisionName(beatSyncGetSubdivision()),
                    beatSyncGetHornTicksPerCycle(),
                    beatSyncGetDrumTicksPerCycle());
    }
  }
}

void setup() {
  Serial.begin(115200);

  inputQueue = xQueueCreate(32, sizeof(InputEvent));
  clockQueue = xQueueCreate(128, sizeof(ClockMsg)); // 24 PPQN; 128 is plenty

  midiParams = {inputQueue, clockQueue};

  BaseType_t midiOk = xTaskCreatePinnedToCore(midiListnerTask, "midi", 4096,
                                              &midiParams, 3, nullptr, 1);
  BaseType_t clkOk = xTaskCreatePinnedToCore(clockSyncTask, "clk", 4096,
                                             clockQueue, 2, nullptr, 1);

  if (midiOk != pdPASS || clkOk != pdPASS) {
    Serial.println("Failed to create tasks");
    while (true) {
      delay(1000);
    }
  }

  g_lastPrintMs = millis();
  Serial.printf("Ready - send MIDI clock + CC %u (ch 1) to select subdivision\n",
                MIDI_SUBDIV_CC);
}

void loop() {
  pumpInputQueue();

  // Run the wrap EVERY iteration (~1 ms), exactly like the 100 Hz controller, so
  // g_beatPrevCmd stays continuous (per-step move << 0.5 rev). Sampling it only
  // at the 200 ms print rate would alias the nearest-rev wrap and make dCmd junk.
  const bool running = clockSyncIsRunning();
  const bool locked = clockSyncIsLocked();
  const double bpm = clockSyncGetBpm();
  const uint64_t ticks = clockSyncGetTickCount();
  const double subPhase = clockSyncGetPhase();
  // The wrap demo tracks the HORN; the drum target is shown alongside.
  const double ticksPerCycle = (double)beatSyncGetHornTicksPerCycle();
  const double drumTicksPerCycle = (double)beatSyncGetDrumTicksPerCycle();

  // Mirrors controller.cpp BeatSync mode, sans encoder/slip + SetPosition().
  const double ticksPerSec = bpm / 60.0 * (double)MIDI_PPQN;
  const double targetRevS = ticksPerSec / ticksPerCycle;
  const double drumRevS = ticksPerSec / drumTicksPerCycle;
  const double clockPos = (ticks + subPhase) / ticksPerCycle;
  const double musicalTarget = clockPos + kBeatHomeFrac;

  // Re-seed the wrap when the clock isn't actively driving beat-sync, so a new
  // lock starts continuous (mirrors RotorCtrl::beatInit reset on mode change).
  if (!(running && locked))
    g_beatInit = false;
  if (!g_beatInit) {
    g_beatPrevCmd = musicalTarget;
    g_clockPosAtLastPrint = clockPos;
    g_cmdAtLastPrint = musicalTarget;
    g_beatInit = true;
  }

  // Nearest-face wrap: cmdEnc tracks clockPos continuously, but a playhead jump
  // or subdivision change re-snaps it <=0.5 rev instead of leaping.
  const double K = round(g_beatPrevCmd - musicalTarget);
  const double cmdEnc = musicalTarget + K;
  g_beatPrevCmd = cmdEnc;

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - g_lastPrintMs) >= kPrintPeriodMs) {
    g_lastPrintMs += kPrintPeriodMs;

    // How far each moved since the last print. Normal play: dClk ~= dCmd (small,
    // positive). On a playhead jump: dClk spikes huge, dCmd stays bounded -> fix.
    const double dClk = clockPos - g_clockPosAtLastPrint;
    const double dCmd = cmdEnc - g_cmdAtLastPrint;
    g_clockPosAtLastPrint = clockPos;
    g_cmdAtLastPrint = cmdEnc;

    Serial.printf("run=%d lock=%d bpm=%6.1f sub=%-5s | "
                  "horn tpr=%2.0f rpm=%6.1f | drum tpr=%2.0f rpm=%6.1f | "
                  "clockPos=%10.3f dClk=%+9.3f | cmd=%9.3f dCmd=%+7.3f\n",
                  (int)running, (int)locked, bpm,
                  subdivisionName(beatSyncGetSubdivision()), ticksPerCycle,
                  targetRevS * 60.0, drumTicksPerCycle, drumRevS * 60.0,
                  clockPos, dClk, cmdEnc, dCmd);
  }

  delay(1);
}
