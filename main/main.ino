#include "footswitch.h"
#include "midi-listner.h"
#include "input_handler.h"
#include "input_event.h"
#include "reference.h"
#include "clock_sync.h"
#include "controller.h"
#include "mode-selector.h"

// Define CLOCK_SIM to replace the MIDI clock source with a simulated 40 BPM clock.
// #define CLOCK_SIM
#ifdef CLOCK_SIM
#include "clock_sim.h"
#endif

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

#ifndef CLOCK_SIM
    xTaskCreate(
        footSwitchTask,
        "footSwitchTask",
        4096,
        g_inputQueue,
        3,
        nullptr
    );

    xTaskCreate(
        midiListnerTask,
        "midiListnerTask",
        4096,
        &g_midiParams,
        3,
        nullptr
    );
#else
    xTaskCreate(
        clockSimTask,
        "clockSimTask",
        4096,
        g_clockQueue,
        3,
        nullptr
    );
#endif

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
