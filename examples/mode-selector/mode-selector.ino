#include "Arduino.h"
#include "mode-selector.h"

static const char* sourceName(ControlSource src) {
  switch (src) {
    case ControlSource::Footswitch:      return "Footswitch";
    case ControlSource::ExpressionPedal: return "ExpressionPedal";
    case ControlSource::MidiKeyboard:    return "MidiKeyboard";
    case ControlSource::MidiBeatSync:    return "MidiBeatSync";
  }
  return "Unknown";
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // modeSelectorTask reads the rotary switch ADC ladder, debounces it,
  // and drives the SPDT that routes the TRS socket. The active source is
  // exposed thread-safely via modeSelectorGetSource().
  xTaskCreatePinnedToCore(
      modeSelectorTask, "ModeSel", 2048, NULL, 1, NULL, 1);
}

void loop() {
  Serial.println(String("Active source: ") + sourceName(modeSelectorGetSource()));
  delay(1000);
}
