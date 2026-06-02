// jitter-logger.ino
//
// Measures the per-tick timing jitter (sigma_t) of the incoming USB-MIDI clock,
// i.e. the rms deviation of the tick arrival times from a best-fit constant-tempo
// grid. This is the measurement-noise standard deviation used to design the
// alpha-beta clock-recovery filter (see the thesis, design chapter).
//
// Procedure:
//   1. Flash to the Arduino Nano ESP32 (USB stack: TinyUSB).
//   2. Open the Serial Monitor at 115200 baud (capture the output to a file).
//   3. From a DAW, set a FIXED tempo, send Start, run ~60 s, then Stop.
//   4. On Stop the sketch prints period, BPM and sigma_t, then dumps the raw
//      timestamps and residuals as CSV for offline plotting/histograms.
//
// Notes:
//   - The tick is timestamped in the MIDI callback, polled at POLL_DELAY_MS like
//     the firmware, so sigma_t reflects what the loop actually sees (USB frame
//     quantization + poll latency). Set POLL_DELAY_MS to 0 to busy-poll and
//     isolate the raw USB jitter for comparison.
//   - sigma_t is the rms deviation from the fitted grid, NOT the std of the
//     inter-tick intervals (which would be sqrt(2)*sigma_t and include drift).

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "esp_timer.h"
#include <math.h>

static constexpr uint32_t POLL_DELAY_MS = 0;     // 1 = match firmware; 0 = busy-poll
static constexpr size_t   MAX_TICKS     = 8192;  // ~170 s at 120 BPM (48 ticks/s)

static Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

static int64_t s_t[MAX_TICKS];   // tick arrival timestamps [us]
static size_t  s_n       = 0;
static bool    s_running = false;

// ---- MIDI callbacks (fire inside MIDI.read() in loop) ----
static void onClock() {
  const int64_t now = esp_timer_get_time();   // timestamp first; keep this minimal
  if (s_running && s_n < MAX_TICKS) {
    s_t[s_n++] = now;
  }
}

static void onStart()    { s_n = 0; s_running = true; Serial.println("# START - collecting"); }
static void onContinue() {          s_running = true; Serial.println("# CONTINUE - collecting"); }

static void analyzeAndDump() {
  if (s_n < 3) { Serial.println("# too few ticks to analyse"); return; }

  // Least-squares fit  t_k = a + b*k   (k = 0..n-1), b = period [us/tick]
  const double n = (double)s_n;
  double sk = 0, skk = 0, st = 0, skt = 0;
  for (size_t k = 0; k < s_n; ++k) {
    const double kk = (double)k;
    const double tk = (double)s_t[k];
    sk  += kk;
    skk += kk * kk;
    st  += tk;
    skt += kk * tk;
  }
  const double b = (n * skt - sk * st) / (n * skk - sk * sk);  // period [us/tick]
  const double a = (st - b * sk) / n;                          // intercept [us]

  // Residuals -> sigma_t (rms deviation from the fitted grid)
  double ss = 0;
  for (size_t k = 0; k < s_n; ++k) {
    const double resid = (double)s_t[k] - (a + b * (double)k);
    ss += resid * resid;
  }
  const double sigma = sqrt(ss / (n - 2.0));   // 2 fitted parameters
  const double bpm   = 60.0e6 / (b * 24.0);

  Serial.println("# ---- jitter result ----");
  Serial.printf("# ticks   = %u\n", (unsigned)s_n);
  Serial.printf("# period  = %.3f us/tick\n", b);
  Serial.printf("# bpm     = %.3f\n", bpm);
  Serial.printf("# sigma_t = %.3f us\n", sigma);
  Serial.println("# k,timestamp_us,residual_us");
  for (size_t k = 0; k < s_n; ++k) {
    const double resid = (double)s_t[k] - (a + b * (double)k);
    Serial.printf("%u,%lld,%.1f\n", (unsigned)k, (long long)s_t[k], resid);
  }
  Serial.println("# ---- end ----");
}

static void onStop() {
  s_running = false;
  Serial.println("# STOP");
  analyzeAndDump();
}

void setup() {
  Serial.begin(115200);

  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleClock(onClock);
  MIDI.setHandleStart(onStart);
  MIDI.setHandleStop(onStop);
  MIDI.setHandleContinue(onContinue);

  Serial.println("# jitter-logger ready - send a steady MIDI clock (Start)");
}

void loop() {
  while (MIDI.read()) {}

  // Auto-dump if the buffer fills while still running.
  if (s_running && s_n >= MAX_TICKS) {
    s_running = false;
    Serial.println("# buffer full");
    analyzeAndDump();
  }

  if (POLL_DELAY_MS) delay(POLL_DELAY_MS);
}
