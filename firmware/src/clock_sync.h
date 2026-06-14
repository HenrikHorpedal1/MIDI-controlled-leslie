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

// Thread-safe getters (call from any task, incl. loop()).
bool     clockSyncIsRunning();
double   clockSyncGetBpm();        // smoothed BPM
uint64_t clockSyncGetTickCount();  // MIDI clocks since last Start/Continue (or SPP)
float    clockSyncGetPhase();      // 0..1 phase within one MIDI clock tick (smoothed)
double   clockSyncGetTickPosition(); // continuous absolute tick position (ticks + fraction, unclamped)
double clockSyncGetPeriodUs();
double clockSyncGetLastErrUs();
bool   clockSyncIsLocked();
