#include "quadrature-encoder.h"
#include "velocity.h"
#include "footswitch.h"
#include "midi-listner.h"
#include "input_handler.h"
#include "input_event.h"
#include "reference.h"
#include "ramp-trajectory.h"
#include "controller.h"
#include "motor.h"
#include "sd_logger.h"

static QueueHandle_t g_inputQueue = nullptr;

void setup() {
    Serial.begin(115200);

    encoderInit();
    velocityInit();
    motorInit();
    referenceInit();
    rampTrajectoryInit();

    // Initialize Input queue and pass to publishers and subscribers
    g_inputQueue = xQueueCreate(16, sizeof(InputEvent));

    xTaskCreate(
        footSwitchTask,
        "footSwitchTask",
        4096,
        g_inputQueue, //parameter
        3,      // priority
        nullptr
    );

    xTaskCreate(
        midiListnerTask,
        "midiListnerTask",
        4096,
        g_inputQueue,
        3,      // priority
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

    rampTrajectoryStartTask(3);

    sdLoggerBegin();
    
    xTaskCreate(
        controllerTask,
        "ControllerTask",
        4096,
        nullptr,
        5,      // priority
        nullptr
    );
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
