#include "footswitch.h"
#include "midi-listener.h"
#include "input_handler.h"
#include "input_event.h"
#include "reference.h"
#include "controller.h"

static QueueHandle_t g_inputQueue = nullptr;

void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("Started");

  referenceInit();

  g_inputQueue = xQueueCreate(16, sizeof(InputEvent));

  xTaskCreate(
      footSwitchTask,
      "footSwitchTask",
      4096,
      g_inputQueue,
      3,
      nullptr
  );

  xTaskCreate(
      midiListenerTask,
      "midiListenerTask",
      4096,
      g_inputQueue,
      3,
      nullptr
  );

  xTaskCreate(
      inputHandlerTask,
      "inputHandlerTask",
      4096,
      g_inputQueue,
      4,
      nullptr
  );

  xTaskCreate(
      controllerTask,
      "ControllerTask",
      4096,
      nullptr,
      5,
      nullptr
  );
}

void loop(){
    vTaskDelay(portMAX_DELAY);
}
