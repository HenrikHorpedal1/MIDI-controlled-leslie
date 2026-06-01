// input_handler.cpp
#include "input_event.h"
#include "mode-selector.h"
#include "reference.h"
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

constexpr float HORN_CHORALE_RPM = 30.0f;
constexpr float HORN_TREMOLO_RPM = 50.0f;
constexpr float DRUM_CHORALE_RPM = 40.0f;
constexpr float DRUM_TREMOLO_RPM = 100.0f;
constexpr int CC_DEADBAND = 15;

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

void inputHandlerTask(void *pvParameters) {
  g_inputQueue = static_cast<QueueHandle_t>(pvParameters);

  InputEvent ev;

  for (;;) {
    if (xQueueReceive(g_inputQueue, &ev, portMAX_DELAY) == pdTRUE) {

      // const ControlSource activeSource = modeSelectorGetSource();
      // for testing now:
      const ControlSource activeSource = ControlSource::Footswitch;

      switch (ev.source) {

      case InputSource::Footswitch: {
        if (activeSource != ControlSource::Footswitch)
          break;

        const FootswitchState &fs = ev.data.foot;
        ReferenceState horn, drum;
        horn.angleDeg = drum.angleDeg = 0.0f;
        if (fs.swA && fs.swB) {
          horn.velRPM = HORN_TREMOLO_RPM;
          drum.velRPM = DRUM_TREMOLO_RPM;
        } else if (fs.swA) {
          horn.velRPM = HORN_CHORALE_RPM;
          drum.velRPM = DRUM_CHORALE_RPM;
        } else {
          horn.velRPM = drum.velRPM = 0.0f;
        }
        referenceSet(Rotor::Horn, horn);
        referenceSet(Rotor::Drum, drum);
        break;
      }

      case InputSource::ExpPedal: {
        if (activeSource != ControlSource::ExpressionPedal)
          break;
        // TODO: Implement when driver is implemented.
        break;
      }

      case InputSource::MidiCC: {
        if (activeSource != ControlSource::MidiKeyboard)
          break;

        float rpm = midiCCToRPM(ev.data.midiCC.value);
        ReferenceState horn = {0.0f, rpm};
        ReferenceState drum = {0.0f,
                               rpm * (DRUM_TREMOLO_RPM / HORN_TREMOLO_RPM)};
        referenceSet(Rotor::Horn, horn);
        referenceSet(Rotor::Drum, drum);
        break;
      }

      case InputSource::MidiButton: {
        if (activeSource != ControlSource::MidiKeyboard)
          break;

        ReferenceState horn = {0.0f, 0.0f};
        ReferenceState drum = {0.0f, 0.0f};
        switch (ev.data.midiButton) {
        case MidiButtonEvent::BUTTON0:
          horn.velRPM = HORN_CHORALE_RPM;
          drum.velRPM = DRUM_CHORALE_RPM;
          break;
        case MidiButtonEvent::BUTTON1:
          horn.velRPM = drum.velRPM = 0.0f;
          break;
        case MidiButtonEvent::BUTTON2:
          horn.velRPM = HORN_TREMOLO_RPM;
          drum.velRPM = DRUM_TREMOLO_RPM;
          break;
        }
        referenceSet(Rotor::Horn, horn);
        referenceSet(Rotor::Drum, drum);
        break;
      }
      }
    }
  }
}
