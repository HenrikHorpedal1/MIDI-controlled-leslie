#include "motor.h"
#include "quadrature-encoder.h"
#include "velocity.h"
#include "reference.h"
#include "controller.h"

void controllerTask(void *pvParameters);

void setup() {
    Serial.begin(115200);

    motorInit();
    encoderInit();       // whatever your encoder init is called
    velocityInit();      // with CPR if needed
    referenceInit();
    //startInputHandler(); // your input/ref module

    xTaskCreate(
        controllerTask,
        "ControllerTask",
        4096,
        nullptr,
        5,      // priority, tune as needed
        nullptr
    );
}


ReferenceState ref{};

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
    ref.angleDeg = 0.0f;
    ref.velRPM = 40;
    ref.enabled  = true;
    ref.valid    = true;


    referenceSetFrom(RefSource::FootSwitch, ref);
    referenceSetMode(RefSource::FootSwitch);

    


    vTaskDelay(portMAX_DELAY);

}
