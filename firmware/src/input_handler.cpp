// input_handler.cpp
//
// The input handler is the SINGLE producer of operator intent. Every input —
// footswitch, MIDI, and rotary mode changes — arrives here as an InputEvent.
// The handler tracks the active input source, maps events to a working
// Reference, and publishes the whole Reference as one coherent snapshot. The
// controller reads only that snapshot (plus the clock and its own feedback).
#include "input_event.h"
#include "source-selector.h"
#include "reference.h"
#include "beat_sync.h"
#include "midi-listner.h"
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

constexpr float HORN_CHORALE_RPM = 40.0f;
constexpr float HORN_TREMOLO_RPM = 420.0f;
constexpr float DRUM_CHORALE_RPM = 35.0f;
constexpr float DRUM_TREMOLO_RPM = 350.0f;

// Footswitch Chorale runs a touch faster than the MIDI-triggered Chorale.
constexpr float FOOT_HORN_CHORALE_RPM = 50.0f;
constexpr float FOOT_DRUM_CHORALE_RPM = 44.0f;

static QueueHandle_t g_inputQueue = nullptr;

static float midiCCToRPM(uint8_t ccVal) {
  constexpr float RPM_MAX = 400.0f;
  constexpr uint8_t CC_MAX = 127;
  constexpr uint8_t CC_DEADBAND = 10;

  if (ccVal <= CC_DEADBAND) {
    return 0.0f;
  }

  float t = static_cast<float>(ccVal - CC_DEADBAND) /
            static_cast<float>(CC_MAX - CC_DEADBAND); // 0..1

  float rpm = t * RPM_MAX;
  if (rpm > RPM_MAX)
    rpm = RPM_MAX; // safety clamp

  return rpm;
}

// ---------------------------------------------------------------------------
// Per-mode handlers. Dispatch is mode-first: the active source picks the
// handler, and each handler interprets only the input events it cares about
// (ignoring the rest, e.g. stray MIDI while the footswitch is active). They
// update the working Reference's RPM fields; the caller publishes it.
// ---------------------------------------------------------------------------

static void handleFootswitch(Reference &ref, const InputEvent &ev) {
  if (ev.type != EventType::Footswitch)
    return;

  const FootswitchState &fs = ev.data.foot;
  if (fs.swA && fs.swB) {
    ref.hornRPM = HORN_TREMOLO_RPM;
    ref.drumRPM = DRUM_TREMOLO_RPM;
  } else if (fs.swA) {
    ref.hornRPM = FOOT_HORN_CHORALE_RPM;
    ref.drumRPM = FOOT_DRUM_CHORALE_RPM;
  } else {
    ref.hornRPM = ref.drumRPM = 0.0f;
  }
}

static void handleExpressionPedal(Reference &ref, const InputEvent &ev) {
  if (ev.type != EventType::ExpPedal)
    return;
  // TODO: Implement when exp-pedal driver is implemented.
}

static void handleMidiKeyboard(Reference &ref, const InputEvent &ev) {
  switch (ev.type) {
  case EventType::MidiButton:
    switch (ev.data.midiButton) {
    case MidiButtonEvent::BUTTON0:
      ref.hornRPM = HORN_CHORALE_RPM;
      ref.drumRPM = DRUM_CHORALE_RPM;
      break;
    case MidiButtonEvent::BUTTON1:
      ref.hornRPM = ref.drumRPM = 0.0f;
      break;
    case MidiButtonEvent::BUTTON2:
      ref.hornRPM = HORN_TREMOLO_RPM;
      ref.drumRPM = DRUM_TREMOLO_RPM;
      break;
    }
    break;

  case EventType::MidiCC: {
    const MidiCCEvent &cc = ev.data.midiCC;
    if (cc.control == MIDI_RATE_CC) {
      // Free-running rate -> RPM.
      const float rpm = midiCCToRPM(cc.value);
      ref.hornRPM = rpm;
      ref.drumRPM = rpm * (DRUM_TREMOLO_RPM / HORN_TREMOLO_RPM);
    } else if (cc.control == MIDI_SUSTAIN_CC) {
      // TODO: define sustain pedal behavior.
    }
    // Other CCs (e.g. subdivision) are ignored in keyboard mode.
    break;
  }

  default:
    // Footswitch / exp-pedal events ignored in MIDI keyboard mode.
    break;
  }
}

static void handleBeatSync(const InputEvent &ev) {
  // Only the subdivision CC is meaningful here; it is committed to beat_sync's
  // own validated state. Rotor speed comes from the MIDI clock, consumed by the
  // controller task directly — so nothing in the Reference changes here.
  if (ev.type != EventType::MidiCC)
    return;
  if (ev.data.midiCC.control != MIDI_SUBDIV_CC)
    return;
  beatSyncSetSubdivisionFromCC(ev.data.midiCC.value);
}

void inputHandlerTask(void *pvParameters) {
  g_inputQueue = static_cast<QueueHandle_t>(pvParameters);

  // Active input source is learned from SourceChange events (the source
  // selector emits one at startup), so this initial value is only used until then.
  InputSource activeSource = InputSource::Footswitch;
  Reference   ref          = {DriveMode::Velocity, 0.0f, 0.0f};

  InputEvent ev;

  for (;;) {
    if (xQueueReceive(g_inputQueue, &ev, portMAX_DELAY) == pdTRUE) {
      if (ev.type == EventType::SourceChange) {
        activeSource = ev.data.source;
        ref.mode = (activeSource == InputSource::MidiBeatSync)
                       ? DriveMode::BeatSync
                       : DriveMode::Velocity;
        // RPM is intentionally preserved across a source switch: the rotor keeps
        // doing what it did until the newly-selected source issues a command.
      } else {
        switch (activeSource) {
        case InputSource::Footswitch:
          handleFootswitch(ref, ev);
          break;
        case InputSource::ExpressionPedal:
          handleExpressionPedal(ref, ev);
          break;
        case InputSource::MidiKeyboard:
          handleMidiKeyboard(ref, ev);
          break;
        case InputSource::MidiBeatSync:
          handleBeatSync(ev);
          break;
        }
      }

      referenceSet(ref); // publish one coherent snapshot
    }
  }
}
