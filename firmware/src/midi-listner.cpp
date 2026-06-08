// midi.cpp
#include "midi-listner.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "tusb.h"

#include "input_event.h"
#include "clock_sync.h"
#include "esp_timer.h"


volatile uint32_t g_usbMidiNotifies = 0;
volatile uint32_t g_noteOnSeen      = 0;
volatile uint32_t g_noteOnMatched   = 0;

static constexpr uint8_t MIDI_TARGET_CHANNEL = 1;
static constexpr uint8_t MIDI_TARGET_CC      = 7;

static constexpr uint8_t PAD_CHANNEL   = 10;
static constexpr uint8_t PAD_NOTE_E1   = 40;
static constexpr uint8_t PAD_NOTE_F1   = 41;
static constexpr uint8_t PAD_NOTE_FS1  = 42;

static Adafruit_USBD_MIDI usb_midi;
static MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

static QueueHandle_t s_inputQueue = nullptr;

static inline bool isPadNote(uint8_t n) {
  return n == PAD_NOTE_E1 || n == PAD_NOTE_F1 || n == PAD_NOTE_FS1;
}

static MidiButtonEvent noteToButton(uint8_t note) {
  if (note == PAD_NOTE_E1)  return MidiButtonEvent::BUTTON0;
  if (note == PAD_NOTE_F1)  return MidiButtonEvent::BUTTON1;
  if (note == PAD_NOTE_FS1) return MidiButtonEvent::BUTTON2;
  return MidiButtonEvent::BUTTON0;
}

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
  if (control != MIDI_TARGET_CC) return;

  InputEvent ev{};
  ev.source = InputSource::MidiCC;
  ev.data.midiCC.value = value; // 0..127
  pushEvent(ev);

}

// Toggle logic: only act on real presses (NoteOn with velocity > 0).
// Many devices send NoteOff as NoteOn with velocity == 0.
static void onNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  g_noteOnSeen++;
  if (channel != PAD_CHANNEL) return;
  if (!isPadNote(note)) return;
  if (velocity == 0) return; // treat as release, ignore

  g_noteOnMatched++;
  InputEvent ev{};
  ev.source = InputSource::MidiButton;
  ev.data.midiButton = noteToButton(note);
  pushEvent(ev);

}

static TaskHandle_t s_midiTaskHandle = nullptr;

void midiListnerTask(void *pvParameters) {
  auto* p = static_cast<MidiTaskParams*>(pvParameters);
  s_inputQueue = p->inputQueue;
  s_clockQueue = p->clockQueue;

  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();

  MIDI.setHandleControlChange(onControlChange);
  MIDI.setHandleNoteOn(onNoteOn);

  MIDI.setHandleClock(onClock);
  MIDI.setHandleStart(onStart);
  MIDI.setHandleStop(onStop);
  MIDI.setHandleContinue(onContinue);
  MIDI.setHandleSongPosition(onSongPos);

  for (;;) {
    // Drain parser; callbacks fire here
    while (MIDI.read()) {}

    // Polling delay: for USB MIDI, 1ms is usually totally fine.
    vTaskDelay(1);
  }
}
