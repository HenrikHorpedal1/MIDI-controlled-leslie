// beat_sync.h
#pragma once
#include <stdint.h>

// Tempo-synced subdivisions for beat-sync mode, named by the *perceived* horn
// modulation rate (one mic swell of the horn). The Leslie is miked with two
// mics 180 deg apart at the top (horn) and one mic on the rotor (drum), so:
//   - horn: two mics -> 2 swells per horn revolution
//   - drum: one mic  -> 1 swell per drum revolution
// Each entry therefore carries TWO tick counts (MIDI clocks @ 24 PPQN per full
// revolution), one per rotor. The horn count is 2x the perceived note (e.g.
// perceived 1/4 = a swell every 24 ticks -> 48 ticks/rev). The drum count is
// derived as drumTicks = (R/2) * hornTicks, where R is how many horn swells
// fall in one drum swell:
//   - R = 2   (straight/duple)  -> drumTicks = hornTicks      (equal RPM, 2:1)
//   - R = 3   (triplet)         -> drumTicks = 1.5 * hornTicks (horn 1.5x, 3:1)
//   - R = 8/3 (dotted)          -> drumTicks = (4/3) * hornTicks (8:3; drum sits
//                                  on the slower straight beat, horn floats
//                                  dotted figures over it)
// Ordered slowest -> fastest. CC 0 is reserved (gate off); CC 1 maps to index
// 0 (Rest); CC 2..N map to subdivision indices 1..N-1.
enum class Subdivision : uint8_t {
  Rest,                // beat sync active but rotors parked
  Half,                // perceived 1/2   - horn 96 ticks @ 24 PPQN, R=2
  Quarter,             // perceived 1/4   - horn 48, R=2
  EighthDotted,        // perceived 1/8.  - horn 36, R=8/3
  QuarterTriplet,      // perceived 1/4T  - horn 32, R=3
  Eighth,              // perceived 1/8   - horn 24, R=2
  SixteenthDotted,     // perceived 1/16. - horn 18, R=8/3
  EighthTriplet,       // perceived 1/8T  - horn 16, R=3
  Sixteenth,           // perceived 1/16  - horn 12, R=2
  ThirtySecondDotted,  // perceived 1/32. - horn  9, R=8/3
  SixteenthTriplet,    // perceived 1/16T - horn  8, R=3
  ThirtySecond,        // perceived 1/32  - horn  6, R=2
  ThirtySecondTriplet  // perceived 1/32T - horn  4, R=3
};

static constexpr uint8_t SUBDIVISION_COUNT = 13;

// MIDI clock runs at 24 pulses per quarter note.
static constexpr uint16_t MIDI_PPQN = 24;

// MIDI clock ticks per one full revolution of each rotor at subdivision s.
uint16_t subdivisionHornTicks(Subdivision s);
uint16_t subdivisionDrumTicks(Subdivision s);

// Set the active subdivision from a discrete CC value. CC 0 is reserved
// (keyboard gate "off") and ignored. CC 1 selects Rest (rotors park). CC 2..N
// map to subdivision index 1..N-1, clamped at the top. Thread-safe.
void beatSyncSetSubdivisionFromCC(uint8_t ccValue);

// Thread-safe getters.
Subdivision beatSyncGetSubdivision();
uint16_t    beatSyncGetHornTicksPerCycle();
uint16_t    beatSyncGetDrumTicksPerCycle();

// Target rotor kinematics for beat-sync, derived from the live MIDI clock and
// the active subdivision: one full rotor revolution per synced subdivision
// cycle (e.g. 1 rev per quarter note -> RPM == BPM). Thread-safe.
struct BeatTarget {
  double velRevS; // target rotor speed (rev/s) from the current tempo
  double facePos; // absolute rotor face position the beat wants (revs)
};
BeatTarget beatSyncHornTarget();
BeatTarget beatSyncDrumTarget();
