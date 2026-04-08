// input_handler.cpp
#include <Arduino.h>
#include "input_event.h"
#include "reference.h"
#include "mode-selector.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

constexpr float CHORALE_RPM = 40.0f;
constexpr float TREMOLO_RPM = 400.0f;
constexpr int CC_DEADBAND = 15;

static QueueHandle_t g_inputQueue = nullptr;


static float midiCCToRPM(uint8_t ccVal)
{
    constexpr float RPM_MAX          = 400.0f;
    constexpr uint8_t CC_MAX         = 127;
    constexpr uint8_t CC_DEADBAND    = 10;    

    if (ccVal <= CC_DEADBAND) {
        return 0.0f;
    }

    float t = static_cast<float>(ccVal - CC_DEADBAND) /
              static_cast<float>(CC_MAX - CC_DEADBAND);   // 0..1

    float rpm = t * RPM_MAX;
    if (rpm > RPM_MAX) rpm = RPM_MAX; // safety clamp

    return rpm;
}

void inputHandlerTask(void *pvParameters)
{
    g_inputQueue = static_cast<QueueHandle_t>(pvParameters);

    InputEvent ev;

    for (;;)
    {
        if (xQueueReceive(g_inputQueue, &ev, portMAX_DELAY) == pdTRUE) {

            const ControlSource activeSource = modeSelectorGetSource();

            switch (ev.source) {

                case InputSource::Footswitch:
                {
                    if (activeSource != ControlSource::Footswitch) break;

                    const FootswitchState &fs = ev.data.foot;
                    ReferenceState r;
                    r.angleDeg = 0.0f;
                    r.velRPM   = fs.swA ? (fs.swB ? TREMOLO_RPM : CHORALE_RPM) : 0.0f;

                    Serial.println("reference:");
                    Serial.println(r.velRPM);

                    referenceSet(r);
                    break;
                }

                case InputSource::ExpPedal:
                {
                    if (activeSource != ControlSource::ExpressionPedal) break;
                    //TODO: Implement when driver is implemented.
                    break;
                }

                case InputSource::MidiCC:
                {
                    if (activeSource != ControlSource::MidiKeyboard) break;

                    ReferenceState r;
                    r.angleDeg = 0.0f;
                    r.velRPM   = midiCCToRPM(ev.data.midiCC.value);

                    referenceSet(r);
                    break;
                }

                case InputSource::MidiButton:
                {
                    if (activeSource != ControlSource::MidiKeyboard) break;

                    ReferenceState r;
                    r.angleDeg = 0.0f;
                    switch (ev.data.midiButton) {
                        case MidiButtonEvent::BUTTON0: r.velRPM = CHORALE_RPM; break;
                        case MidiButtonEvent::BUTTON1: r.velRPM = 0.0f;        break;
                        case MidiButtonEvent::BUTTON2: r.velRPM = TREMOLO_RPM; break;
                    }
                    referenceSet(r);
                    break;
                }
            }
        }
    }
}
