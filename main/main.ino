#include "footswitch.h"
#include "midi-listner.h"
#include "input_handler.h"
#include "input_event.h"
#include "reference.h"
#include "clock_sync.h"
#include "controller.h"
#include "mode-selector.h"

static QueueHandle_t g_inputQueue = nullptr;
static QueueHandle_t g_clockQueue = nullptr;
static MidiTaskParams g_midiParams;

void setup() {
    Serial.begin(115200);

    referenceInit();

    g_inputQueue = xQueueCreate(16, sizeof(InputEvent));
    g_clockQueue = xQueueCreate(32, sizeof(ClockMsg)); //TODO: does it make sense to have a greater clock queue than midi queue if all signals goes through midi first?

    g_midiParams.inputQueue = g_inputQueue;
    g_midiParams.clockQueue = g_clockQueue;

    xTaskCreate(
        footSwitchTask,
        "footSwitchTask",
        4096,
        g_inputQueue,
        3,      // TODO: analyze to find reasonable priorities.
        nullptr
    );

    xTaskCreate(
        midiListnerTask,
        "midiListnerTask",
        4096,
        &g_midiParams,
        3,      // priority
        nullptr
    );

    xTaskCreate(
        clockSyncTask,
        "clockSyncTask",
        4096,
        g_clockQueue,
        4,      // priority
        nullptr
    );

    xTaskCreate(
        inputHandlerTask,
        "inputHandlerTask",
        4096,
        g_inputQueue,
        4,      // priority
        nullptr
    );

    xTaskCreate(
        controllerTask,
        "ControllerTask",
        4096,
        nullptr,
        5,      // priority
        nullptr
    );

    xTaskCreate(
        modeSelectorTask,
        "modeSelectorTask",
        2048,
        nullptr,
        2,      // low priority — polls at 20 Hz
        nullptr
    );
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
