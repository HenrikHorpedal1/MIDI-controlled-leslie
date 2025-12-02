#include "quadrature-encoder.h"
#include "velocity.h"
#include "footswitch.h"
#include "midi-listner.h"
#include "input_handler.h"
#include "reference.h"
#include "controller.h"
#include "motor.h"

void controllerTask(void *pvParameters);

void setup() {
    Serial.begin(115200);

    motorInit();
    encoderInit();       // whatever your encoder init is called
    velocityInit();      // with CPR if needed
    referenceInit();
    startInputHandler(); // your input/ref module

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
