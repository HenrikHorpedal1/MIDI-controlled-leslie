// midi-listener.cpp
#include "midi-listener.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "tusb.h"

#include "input_event.h"
#include "clock_sync.h"
#include "esp_timer.h"


static constexpr uint8_t MIDI_TARGET_CHANNEL = 1;
// CC numbers (MIDI_RATE_CC, MIDI_SUBDIV_CC, MIDI_BUTTON_CC, MIDI_SUSTAIN_CC) are
// declared in midi-listener.h.

static Adafruit_USBD_MIDI usb_midi;
static MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

static QueueHandle_t s_inputQueue = nullptr;

static inline void pushEvent(const InputEvent& ev) {
  if (s_inputQueue) {
    (void)xQueueSend(s_inputQueue, &ev, 0);
  }
}

// -------------------- MIDI callbacks --------------------
//
static QueueHandle_t s_clockQueue = nullptr;

static inline void pushClockMsg(ClockMsgType type, uint16_t spp = 0) {
  if (!s_clockQueue) return;
  ClockMsg m{};
  m.type = type;
  m.t_us = esp_timer_get_time();
  m.spp  = spp;
  (void)xQueueSend(s_clockQueue, &m, 0); // never block in MIDI callbacks
}

// MIDI realtime/transport callbacks
static void onClock()    { pushClockMsg(ClockMsgType::Tick); }
static void onStart()    { pushClockMsg(ClockMsgType::Start); }
static void onStop()     { pushClockMsg(ClockMsgType::Stop); }
static void onContinue() { pushClockMsg(ClockMsgType::ContinueMsg); }
static void onSongPos(unsigned int spp) {
  // MIDI SPP is 14-bit; it fits in uint16_t, but the lib wants unsigned int
  pushClockMsg(ClockMsgType::SongPosition, (uint16_t)spp);
}
static void onControlChange(uint8_t channel, uint8_t control, uint8_t value) {
  if (channel != MIDI_TARGET_CHANNEL) return;

  // Keyboard-mode buttons arrive on one CC: value 1/2/3 = press of button 0/1/2,
  // value 0 = release (ignored — the buttons are momentary, the handler latches).
  if (control == MIDI_BUTTON_CC) {
    if (value < 1 || value > 3) return; // 0 = release, or out of range
    InputEvent ev{};
    ev.type = EventType::MidiButton;
    ev.data.midiButton = static_cast<MidiButtonEvent>(value - 1); // 1..3 -> BUTTON0..2
    pushEvent(ev);
    return;
  }

  if (control != MIDI_RATE_CC && control != MIDI_SUBDIV_CC &&
      control != MIDI_SUSTAIN_CC)
    return;

  // Routing (rate vs. subdivision vs. sustain) is decided by the input handler
  // based on the active mode, so just forward which CC and its value.
  InputEvent ev{};
  ev.type = EventType::MidiCC;
  ev.data.midiCC.control = control;
  ev.data.midiCC.value   = value; // 0..127
  pushEvent(ev);
}

static TaskHandle_t s_midiTaskHandle = nullptr;

void midiListenerTask(void *pvParameters) {
  auto* p = static_cast<MidiTaskParams*>(pvParameters);
  s_inputQueue = p->inputQueue;
  s_clockQueue = p->clockQueue;

  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();

  MIDI.setHandleControlChange(onControlChange);

  MIDI.setHandleClock(onClock);
  MIDI.setHandleStart(onStart);
  MIDI.setHandleStop(onStop);
  MIDI.setHandleContinue(onContinue);
  MIDI.setHandleSongPosition(onSongPos);

  for (;;) {
    // Drain parser; callbacks fire here
    while (MIDI.read()) {}

    // Polling delay: for USB MIDI, 1ms is usually totally fine. (Busy-waiting a
    // shorter period here starves the low-priority telemetry/UDP task.)
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
