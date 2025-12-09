#include "quadrature-encoder.h"
#include "velocity.h"
#include "footswitch.h"
#include "midi-listner.h"
#include "input_handler.h"
#include "input_event.h"
#include "reference.h"
#include "controller.h"
#include "motor.h"

static QueueHandle_t g_inputQueue = nullptr;

void setup() {
    Serial.begin(115200);

    encoderInit();       // whatever your encoder init is called
    velocityInit();      // with CPR if needed
    motorInit(); // set pins and 
    referenceInit(); //securely start with zeros for all references

    // Initialize Input queue and pass to publishers and subscribers
    g_inputQueue = xQueueCreate(16, sizeof(InputEvent));
    //footswitch
    xTaskCreate(
        footSwitchTask,
        "footSwitchTask",
        4096,
        g_inputQueue, //parameter
        3,      // priority, tune as needed
        nullptr
    );
    //midi listner
    xTaskCreate(
        midiListnerTask,
        "midiListnerTask",
        4096,
        g_inputQueue,
        3,      // priority, tune as needed
        nullptr
    );
    //input handler
    xTaskCreate(
        inputHandlerTask,
        "inputHandlerTask",
        4096,
        g_inputQueue,
        4,      // priority, tune as needed
        nullptr
    );
    
    xTaskCreate(
        controllerTask,
        "ControllerTask",
        4096,
        nullptr,
        5,      // priority, tune as needed
        nullptr
    );
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
