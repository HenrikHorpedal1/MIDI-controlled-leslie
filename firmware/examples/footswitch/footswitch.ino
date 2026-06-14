#include "Arduino.h"
#include "footswitch.h"
#include "input_event.h"

static QueueHandle_t g_inputQueue = nullptr;

void setup() {
  Serial.begin(115200);
  delay(50);

  // footSwitchTask reports state changes via an InputEvent queue.
  g_inputQueue = xQueueCreate(16, sizeof(InputEvent));

  xTaskCreatePinnedToCore(
      footSwitchTask, "SwTask", 2048, g_inputQueue, 1, NULL, 1);
}

void loop() {
  InputEvent ev;
  // Block until the footswitch task reports a change.
  if (xQueueReceive(g_inputQueue, &ev, portMAX_DELAY) == pdTRUE &&
      ev.type == EventType::Footswitch) {
    Serial.println("Switch A: " + String(ev.data.foot.swA));
    Serial.println("Switch B: " + String(ev.data.foot.swB));
  }
}
