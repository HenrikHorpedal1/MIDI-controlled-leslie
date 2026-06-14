// beat_sync.cpp
#include "beat_sync.h"

#include <Arduino.h>
#include "clock_sync.h"
#include "freertos/FreeRTOS.h"

// Rotor speed cap for beat-sync. A subdivision whose *faster* rotor at the
// current tempo would exceed this is silently denied (the active subdivision is
// kept). With every feel the horn is the faster (or equal) rotor, but the cap is
// checked against min(hornTicks, drumTicks) regardless so it stays correct if
// the ratios change.
static constexpr double BEAT_MAX_RPM = 450.0;

// Per-subdivision rotor geometry. hornTicks = MIDI clocks per horn revolution
// (= 2x the perceived note, two mics). R = rNum/rDen = horn swells per drum
// swell; drumTicks = (R/2) * hornTicks. rNum/rDen are chosen so drumTicks is an
// exact integer for every entry.
struct SubdivInfo {
  uint16_t hornTicks;
  uint8_t  rNum;
  uint8_t  rDen;
};

static SubdivInfo subdivInfo(Subdivision s) {
  switch (s) {
    case Subdivision::Rest:                return { 0, 2, 1}; // ticks=0; rotors park
    case Subdivision::Half:                return {96, 2, 1};
    case Subdivision::Quarter:             return {48, 2, 1};
    case Subdivision::EighthDotted:        return {36, 8, 3};
    case Subdivision::QuarterTriplet:      return {32, 3, 1};
    case Subdivision::Eighth:              return {24, 2, 1};
    case Subdivision::SixteenthDotted:     return {18, 8, 3};
    case Subdivision::EighthTriplet:       return {16, 3, 1};
    case Subdivision::Sixteenth:           return {12, 2, 1};
    case Subdivision::ThirtySecondDotted:  return { 9, 8, 3};
    case Subdivision::SixteenthTriplet:    return { 8, 3, 1};
    case Subdivision::ThirtySecond:        return { 6, 2, 1};
    case Subdivision::ThirtySecondTriplet: return { 4, 3, 1};
  }
  return {48, 2, 1};
}

uint16_t subdivisionHornTicks(Subdivision s) { return subdivInfo(s).hornTicks; }

uint16_t subdivisionDrumTicks(Subdivision s) {
  const SubdivInfo i = subdivInfo(s);
  // drumTicks = (rNum/rDen / 2) * hornTicks; exact integer for all entries.
  return (uint16_t)((uint32_t)i.hornTicks * i.rNum / (2u * i.rDen));
}

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

// Rotor face offset to align with the beat (0..1 rev).
static constexpr double BEAT_HOME_FRAC = 0.0;

static BeatTarget beatTarget(uint16_t ticksPerCycle) {
  const double tpc = (double)ticksPerCycle;
  // revs/s = (MIDI ticks/s) / (ticks per cycle): one rotor revolution per cycle.
  const double ticksPerSec = clockSyncGetBpm() / 60.0 * (double)MIDI_PPQN;
  // Continuous (unclamped) tick position: commanded as an absolute rotor face,
  // so it must stay C0-continuous across tick boundaries. clockSyncGetPhase()'s
  // [0,1] clamp would stair-step facePos and make the rotor hunt.
  const double clockTicks = clockSyncGetTickPosition();
  return {ticksPerSec / tpc, clockTicks / tpc + BEAT_HOME_FRAC};
}

BeatTarget beatSyncHornTarget() {
  return beatTarget(beatSyncGetHornTicksPerCycle());
}

BeatTarget beatSyncDrumTarget() {
  return beatTarget(beatSyncGetDrumTicksPerCycle());
}
