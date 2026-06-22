#include "footswitch.h"
#include "midi-listener.h"
#include "input_handler.h"
#include "input_event.h"
#include "reference.h"
#include "clock_sync.h"
#include "controller.h"
#include "source-selector.h"
#include "udp_telemetry.h"
#include "telemetry_log.h"
#include "wifi_config.h"
#include <WiFi.h>

// Define CLOCK_SIM to replace the MIDI clock source with a simulated 40 BPM clock.
// #define CLOCK_SIM
#ifdef CLOCK_SIM
#include "clock_sim.h"
#endif

static QueueHandle_t g_inputQueue = nullptr;
static QueueHandle_t g_clockQueue = nullptr;
static MidiTaskParams g_midiParams;

static void wifiTelemetryTask(void*) {
    // Retry until connected, so telemetry recovers if the AP appears late or
    // WiFi drops during boot. Once up, start telemetry and self-delete.
    for (;;) {
        WiFi.begin(TELEM_WIFI_SSID, TELEM_WIFI_PASSWORD);
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[wifi] connected: %s\n", WiFi.localIP().toString().c_str());
            Telemetry.begin(TELEM_HOST, TELEM_PORT);
            vTaskDelete(nullptr);
        }
        Serial.println("[wifi] not connected — retrying");
        WiFi.disconnect();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void setup() {
    Serial.begin(115200);

    referenceInit();
    telemetryLogInit(); // create the telemetry bus before any producer runs

    g_inputQueue = xQueueCreate(16, sizeof(InputEvent));
    // Clock queue is deeper than the input queue on purpose: MIDI clock ticks
    // (up to ~120/s at 24 PPQN) are far higher-rate than discrete input events
    // and ride their own queue, so they need more burst headroom.
    g_clockQueue = xQueueCreate(32, sizeof(ClockMsg));

    g_midiParams.inputQueue = g_inputQueue;
    g_midiParams.clockQueue = g_clockQueue;

#ifndef CLOCK_SIM
    xTaskCreatePinnedToCore(
        footSwitchTask,
        "footSwitchTask",
        4096,
        g_inputQueue,
        2,
        nullptr,
        0       // core 0
    );

    // Higher priority than clockSyncTask so timestamps are captured before
    // the PLL consumes them.
    xTaskCreatePinnedToCore(
        midiListenerTask,
        "midiListenerTask",
        4096,
        &g_midiParams,
        5,
        nullptr,
        0       // core 0
    );
#else
    xTaskCreatePinnedToCore(
        clockSimTask,
        "clockSimTask",
        4096,
        g_clockQueue,
        3,
        nullptr,
        0       // core 0
    );
#endif

    xTaskCreatePinnedToCore(
        clockSyncTask,
        "clockSyncTask",
        4096,
        g_clockQueue,
        4,
        nullptr,
        0       // core 0
    );

    xTaskCreatePinnedToCore(
        inputHandlerTask,
        "inputHandlerTask",
        4096,
        g_inputQueue,
        3,
        nullptr,
        0       // core 0
    );

    // Dedicated core — no competition from other app tasks.
    xTaskCreatePinnedToCore(
        controllerTask,
        "ControllerTask",
        4096,
        nullptr,
        5,
        nullptr,
        1       // core 1 (APP_CPU), dedicated
    );

    xTaskCreatePinnedToCore(
        telemetryLogTask,
        "telemetryTask",
        4096,
        nullptr,
        1,
        nullptr,
        0       // core 0
    );

    xTaskCreatePinnedToCore(
        sourceSelectorTask,
        "sourceSelectorTask",
        2048,
        g_inputQueue,
        2,
        nullptr,
        0       // core 0
    );

    xTaskCreatePinnedToCore(
        wifiTelemetryTask,
        "wifiTelemetryTask",
        4096,
        nullptr,
        1,
        nullptr,
        0       // core 0 — keeps WiFi work on the WiFi core
    );
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
