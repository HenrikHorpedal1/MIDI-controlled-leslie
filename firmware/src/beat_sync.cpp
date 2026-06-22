// beat_sync.cpp
#include "beat_sync.h"

#include <Arduino.h>
#include "clock_sync.h"
#include "leslie_config.h"
#include "freertos/FreeRTOS.h"

static constexpr uint8_t SUBDIVISION_COUNT = 13;

// MIDI clocks (24 PPQN) per one full revolution of each rotor, per subdivision.
// hornTicks = 2x the perceived note (two horn mics -> 2 swells/rev). drumTicks
// follows from R = horn swells per drum swell: drumTicks = (R/2) * hornTicks
// (R=2 duple -> equal; R=3 triplet -> 1.5x; R=8/3 dotted -> 4/3x). The values are
// hardcoded (all exact integers); the R column is documentation only.
//
// Indexed by Subdivision; order MUST match the enum in beat_sync.h.
struct SubdivTicks { uint16_t horn; uint16_t drum; };
static constexpr SubdivTicks SUBDIV_TICKS[SUBDIVISION_COUNT] = {
  /* Rest                */ {  0,  0 }, // ticks=0; rotors park
  /* Half           R=2  */ { 96, 96 },
  /* Quarter        R=2  */ { 48, 48 },
  /* EighthDotted   R=8/3*/ { 36, 48 },
  /* QuarterTriplet R=3  */ { 32, 48 },
  /* Eighth         R=2  */ { 24, 24 },
  /* SixteenthDotted R=8/3*/{ 18, 24 },
  /* EighthTriplet  R=3  */ { 16, 24 },
  /* Sixteenth      R=2  */ { 12, 12 },
  /* 32ndDotted     R=8/3*/ {  9, 12 },
  /* SixteenthTriplet R=3*/ {  8, 12 },
  /* ThirtySecond   R=2  */ {  6,  6 },
  /* 32ndTriplet    R=3  */ {  4,  6 },
};

static SubdivTicks subdivTicks(Subdivision s) {
  const uint8_t i = static_cast<uint8_t>(s);
  return (i < SUBDIVISION_COUNT) ? SUBDIV_TICKS[i] : SUBDIV_TICKS[2]; // default Quarter
}

static uint16_t subdivisionHornTicks(Subdivision s) { return subdivTicks(s).horn; }
static uint16_t subdivisionDrumTicks(Subdivision s) { return subdivTicks(s).drum; }

// --------- Shared state (protected by a spinlock) ----------
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static Subdivision  s_subdivision = Subdivision::Quarter;

void beatSyncSetSubdivisionFromCC(uint8_t ccValue) {
  // CC value 0 is reserved (keyboard gate "off" position) and ignored; the
  // active subdivision is kept. Values 1..N map to subdivision index 0..N-1.
  if (ccValue == 0) return;
  uint8_t index = ccValue - 1;
  if (index >= SUBDIVISION_COUNT) index = SUBDIVISION_COUNT - 1;
  const Subdivision s = static_cast<Subdivision>(index);

  // Rest parks the rotors — no speed to cap.
  if (s != Subdivision::Rest) {
    const double bpm = clockSyncGetBpm();
    const uint16_t fastTicks =
        min(subdivisionHornTicks(s), subdivisionDrumTicks(s));
    const double rpm = bpm * (double)MIDI_PPQN / (double)fastTicks;
    if (rpm > BEAT_MAX_RPM) {
      Serial.printf("[beatsync] subdiv CC=%u denied: %.0f RPM > %.0f cap @ %.1f BPM\n",
                    ccValue, rpm, BEAT_MAX_RPM, bpm);
      return;
    }
  }

  portENTER_CRITICAL(&s_mux);
  s_subdivision = s;
  portEXIT_CRITICAL(&s_mux);
}

Subdivision beatSyncGetSubdivision() {
  portENTER_CRITICAL(&s_mux);
  const Subdivision s = s_subdivision;
  portEXIT_CRITICAL(&s_mux);
  return s;
}

uint16_t beatSyncGetHornTicksPerCycle() {
  return subdivisionHornTicks(beatSyncGetSubdivision());
}

uint16_t beatSyncGetDrumTicksPerCycle() {
  return subdivisionDrumTicks(beatSyncGetSubdivision());
}

static BeatTarget beatTarget(uint16_t ticksPerCycle, const ClockSnapshot& clk, double homeFrac) {
  const double tpc = (double)ticksPerCycle;
  if (tpc <= 0.0) return {0.0, 0.0}; // Rest (ticks=0): no speed/phase, rotors park

  // revs/s = (MIDI ticks/s) / (ticks per cycle): one rotor revolution per cycle.
  const double ticksPerSec = clk.bpm / 60.0 * (double)MIDI_PPQN;
  double velRevS = ticksPerSec / tpc;
  // Live backstop to the selection-time cap (beatSyncSetSubdivisionFromCC): if the
  // tempo rises after a subdivision is chosen, hold the rotor at BEAT_MAX_RPM
  // rather than letting it over-speed. Phase will slip, speed stays bounded.
  const double capRevS = BEAT_MAX_RPM / 60.0;
  if (velRevS > capRevS) velRevS = capRevS;

  // Retard the phase reference by a fixed TIME (BEAT_PHASE_LEAD_US) to cancel the
  // downstream actuation/acoustic lead. Converting us -> ticks here makes the
  // angular correction scale with tempo/subdivision automatically.
  const double leadTicks = BEAT_PHASE_LEAD_US * 1e-6 * ticksPerSec;
  return {velRevS, (clk.tickPosition - leadTicks) / tpc + homeFrac};
}

void beatSyncGetTargets(BeatTarget& horn, BeatTarget& drum) {
  ClockSnapshot clk;
  clockSyncGetSnapshot(clk);
  const Subdivision s = beatSyncGetSubdivision();
  horn = beatTarget(subdivisionHornTicks(s), clk, HORN_BEAT_HOME_FRAC);
  drum = beatTarget(subdivisionDrumTicks(s), clk, DRUM_BEAT_HOME_FRAC);
}
