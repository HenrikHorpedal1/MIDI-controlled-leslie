#include "Arduino.h"
#include "source-selector.h"
#include "input_event.h"

static const char* sourceName(InputSource s) {
  switch (s) {
    case InputSource::Footswitch:      return "Footswitch";
    case InputSource::ExpressionPedal: return "ExpressionPedal";
    case InputSource::MidiKeyboard:    return "MidiKeyboard";
    case InputSource::MidiBeatSync:    return "MidiBeatSync";
  }
  return "Unknown";
}

static QueueHandle_t g_inputQueue = nullptr;

void setup() {
  Serial.begin(115200);
  delay(50);

  // sourceSelectorTask reads the rotary switch ADC ladder, debounces it, drives
  // the SPDT that routes the TRS socket, and pushes a SourceChange InputEvent
  // onto the queue whenever the position changes (including once at startup).
  g_inputQueue = xQueueCreate(8, sizeof(InputEvent));
  xTaskCreatePinnedToCore(
      sourceSelectorTask, "SourceSel", 2048, g_inputQueue, 1, NULL, 1);
}

// Raw ADC of the rotary ladder. Must match PIN_ROTARY_ADC in source-selector.cpp
// (A7 / GPIO 14). Printed alongside the resolved source so you can see whether
// the ladder voltage actually changes as the rotary is turned.
//   thresholds (source-selector.cpp): >2692 Footswitch | >1897 ExpPedal |
//                                      >1127 MidiKeyboard | else MidiBeatSync
static constexpr int DBG_ROTARY_ADC = 14;

void loop() {
  // Drain any SourceChange events the selector pushed.
  InputEvent ev;
  while (xQueueReceive(g_inputQueue, &ev, 0) == pdTRUE) {
    if (ev.type == EventType::SourceChange) {
      Serial.printf("[event] source → %s\n", sourceName(ev.data.source));
    }
  }

  const int raw = analogRead(DBG_ROTARY_ADC);
  Serial.printf("raw=%4d\n", raw);
  delay(250);
}
