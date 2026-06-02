// clock_sync.cpp
#include "clock_sync.h"

#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// --------- Tunables ----------
static constexpr double BPM_MIN = 30.0;
static constexpr double BPM_MAX = 300.0;

// Alpha-beta filter gains.
// One filter step == one MIDI clock tick (sample interval T = 1 tick), so the
// "velocity" state is exactly the tick period in microseconds and the velocity
// update needs no /T term.
//   g (alpha) -> position/phase correction
//   h (beta)  -> velocity/period correction
static constexpr double G_ALPHA = 0.16;
static constexpr double H_BETA  = 0.014;

static inline double clampd(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}


static inline double bpmToTickPeriodUs(double bpm) {
  // MIDI clock = 24 ticks / quarter note
  return 60.0e6 / (bpm * 24.0);
}

static inline double tickPeriodUsToBpm(double period_us) {
  return 60.0e6 / (period_us * 24.0);
}

// --------- Shared state (protected by a spinlock) ----------
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static bool     s_running   = false;
static bool     s_locked    = false;
static uint64_t s_tickCount = 0;

static int64_t  s_lastTickUs = 0;     // last observed tick time
static double   s_periodUs   = bpmToTickPeriodUs(120.0); // initial guess
static double   s_nextPredUs = 0;     // predicted time for next tick

static uint16_t s_pendingSpp = 0;
static bool     s_haveSpp    = false;

static double s_lastErrUs = 0; // add near shared state

static void resetLockKeepPeriod() {
  s_locked     = false;
  s_lastTickUs = 0;
  s_nextPredUs = 0;
}

static void onStartCommon(bool resetTickCount) {
  s_running = true;
  resetLockKeepPeriod();
  if (resetTickCount) s_tickCount = 0;
  if (s_haveSpp) {
    // SPP unit = 16th notes; 1/16 note = 6 MIDI clocks (24/4)
    s_tickCount = (uint64_t)s_pendingSpp * 6ULL;
    s_haveSpp = false;
  }
}

static void processTick(int64_t t_us) {
  // Ignore ticks if not running; some devices still send realtime when stopped,
  // but most musical use wants to track only while running.
  if (!s_running) return;

  const double periodMin = bpmToTickPeriodUs(BPM_MAX);
  const double periodMax = bpmToTickPeriodUs(BPM_MIN);

  if (!s_locked) {
    // Need 2 ticks to lock. First tick just arms.
    if (s_lastTickUs == 0) {
      s_lastTickUs = t_us;
      return;
    }
    const int64_t dt = t_us - s_lastTickUs;
    s_lastTickUs = t_us;

    // Sanity: reject nonsense
    if (dt < 1000 || dt > 200000) {
      return;
    }

    s_periodUs   = clampd((double)dt, periodMin, periodMax);
    s_nextPredUs = (double)t_us + s_periodUs;
    s_locked     = true;
    s_tickCount++;
    return;
  }

  // Locked: alpha-beta filter update.
  // s_nextPredUs is the a-priori prediction x_pred = x_prev + v_prev for the
  // arrival time of this tick. The residual drives both state corrections.
  const double xPred = s_nextPredUs;
  double err = (double)t_us - xPred;          // residual: positive if tick late
  s_lastErrUs = err;

  // Clamp residual to reject USB jitter / outliers.
  const double errClamp = 0.50 * s_periodUs;
  err = clampd(err, -errClamp, errClamp);

  // Velocity (period) update:  v += h * err
  s_periodUs = clampd(s_periodUs + (H_BETA * err), periodMin, periodMax);

  // Position update:  x = x_pred + g * err  (smoothed estimate of this tick)
  const double xEst = xPred + (G_ALPHA * err);

  // Predict next tick: x + v
  s_lastTickUs = t_us;
  s_nextPredUs = xEst + s_periodUs;
  s_tickCount++;
}

void clockSyncTask(void* pvQueueHandle) {
  QueueHandle_t q = static_cast<QueueHandle_t>(pvQueueHandle);
  ClockMsg msg{};

  for (;;) {
    // Timeout lets us detect "lost clock" if no Stop message arrives.
    if (xQueueReceive(q, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
      portENTER_CRITICAL(&s_mux);
      switch (msg.type) {
        case ClockMsgType::SongPosition:
          s_pendingSpp = msg.spp;
          s_haveSpp = true;
          break;

        case ClockMsgType::Start:
          onStartCommon(true);
          break;

        case ClockMsgType::ContinueMsg:
          onStartCommon(false);
          break;

        case ClockMsgType::Stop:
          s_running = false;
          resetLockKeepPeriod();
          break;

        case ClockMsgType::Tick:
          processTick(msg.t_us);
          break;
      }
      portEXIT_CRITICAL(&s_mux);
    } else {
      // Timeout: if running but no ticks for a while, assume clock stopped.
      const int64_t now = esp_timer_get_time();
      portENTER_CRITICAL(&s_mux);
      if (s_running && s_lastTickUs != 0) {
        // If no tick for >500ms, consider it stopped/lost
        if ((now - s_lastTickUs) > 500000) {
          s_running = false;
          resetLockKeepPeriod();
        }
      }
      portEXIT_CRITICAL(&s_mux);
    }
  }
}

// --------- Public getters ----------
bool clockSyncIsRunning() {
  portENTER_CRITICAL(&s_mux);
  const bool r = s_running;
  portEXIT_CRITICAL(&s_mux);
  return r;
}

double clockSyncGetBpm() {
  portENTER_CRITICAL(&s_mux);
  const double p = s_periodUs;
  portEXIT_CRITICAL(&s_mux);
  return tickPeriodUsToBpm(p);
}

uint64_t clockSyncGetTickCount() {
  portENTER_CRITICAL(&s_mux);
  const uint64_t n = s_tickCount;
  portEXIT_CRITICAL(&s_mux);
  return n;
}

float clockSyncGetPhase() {
  // Phase within one MIDI clock tick (0..1). Useful for smooth modulation.
  const int64_t now = esp_timer_get_time();

  portENTER_CRITICAL(&s_mux);
  const bool locked = s_locked;
  const double period = s_periodUs;
  const double nextPred = s_nextPredUs;
  portEXIT_CRITICAL(&s_mux);

  if (!locked || period <= 0.0 || nextPred <= 0.0) return 0.0f;

  const double timeToNext = nextPred - (double)now;
  double phase = 1.0 - (timeToNext / period);
  phase = clampd(phase, 0.0, 1.0);
  return (float)phase;
}

bool clockSyncIsLocked() {
  portENTER_CRITICAL(&s_mux);
  bool v = s_locked;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

double clockSyncGetPeriodUs() {
  portENTER_CRITICAL(&s_mux);
  double v = s_periodUs;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

double clockSyncGetLastErrUs() {
  portENTER_CRITICAL(&s_mux);
  double v = s_lastErrUs;
  portEXIT_CRITICAL(&s_mux);
  return v;
}
