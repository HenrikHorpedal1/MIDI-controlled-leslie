// clock_sync.h
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum class ClockMsgType : uint8_t {
  Tick,
  Start,
  Stop,
  ContinueMsg,
  SongPosition
};

struct ClockMsg {
  ClockMsgType type;
  int64_t      t_us;   // timestamp (esp_timer_get_time)
  uint16_t     spp;    // for SongPosition (in MIDI "beats" = 16th notes)
};

void clockSyncTask(void* pvQueueHandle);

// One coherent snapshot of the clock model, read under a single critical section.
// Use this (instead of the individual getters) when several fields must agree —
// e.g. beat_sync derives both rotor speed (from bpm) and phase (from tickPosition)
// and must read them from the same instant.
struct ClockSnapshot {
  bool   running;
  bool   locked;
  double bpm;          // smoothed BPM
  double tickPosition; // continuous absolute tick position (ticks + fraction)
};
void clockSyncGetSnapshot(ClockSnapshot& out);

// Thread-safe getters (call from any task, incl. loop()).
bool     clockSyncIsRunning();
double   clockSyncGetBpm();        // smoothed BPM
uint64_t clockSyncGetTickCount();  // MIDI clocks since last Start/Continue (or SPP)
float    clockSyncGetPhase();      // 0..1 phase within one MIDI clock tick (smoothed)
double   clockSyncGetTickPosition(); // continuous absolute tick position (ticks + fraction, unclamped)
double clockSyncGetPeriodUs();
double clockSyncGetLastErrUs();
bool   clockSyncIsLocked();
