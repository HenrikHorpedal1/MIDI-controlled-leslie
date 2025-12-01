// input_handler.cpp
#include <Arduino.h>

#include "input_handler.h"
#include "input_event.h"
#include "footswitch.h"
//#include "exp_pedal.h"
#include "reference.h"
#include "midi-listner.h"

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
    constexpr uint8_t CC_DEADBAND    = 10;    // 0..10 -> 0 rpm, tweak as desired

    if (ccVal <= CC_DEADBAND) {
        return 0.0f;
    }

    float t = static_cast<float>(ccVal - CC_DEADBAND) /
              static_cast<float>(CC_MAX - CC_DEADBAND);   // 0..1

    float rpm = t * RPM_MAX;
    if (rpm > RPM_MAX) rpm = RPM_MAX; // safety clamp

    return rpm;
}

static void inputHandlerTask(void *pvParameters)
{
    (void)pvParameters;

    InputEvent ev;

    for (;;)
    {
        if (xQueueReceive(g_inputQueue, &ev, portMAX_DELAY) == pdTRUE) {

            switch (ev.source) {

                case InputSource::Footswitch:
                {
                    const FootswitchState &fs = ev.data.foot;   // fs.swA == switchA, fs.swB == switchB
                    //Serial.print("Press at");
                    Serial.println(micros());


                    ReferenceState ref{};
                    ref.angleDeg = 0.0f;   // not used here
                    ref.enabled  = true;
                    ref.valid    = true;

                    if (fs.swA)
                    {
                        // A pressed → choose speed based on B
                        if (fs.swB)
                        {
                            ref.velRPM = TREMOLO_RPM;
                        }
                        else if (!fs.swB)
                        {
                            ref.velRPM = CHORALE_RPM;
                        }
                    }
                    else  // !fs.swA
                    {
                        // A not pressed → stop
                        ref.velRPM = 0.0f;
                    }

                    referenceSetFrom(RefSource::FootSwitch, ref);
                    referenceSetMode(RefSource::FootSwitch);
                    Serial.println("Set reference from foot switch.");
                    break;
                }

                case InputSource::ExpPedal:
                {
                    //TODO: Implement when driver is implemented.
                    break;
                }

                case InputSource::Midi:
                {
                    const MidiCCEvent &m = ev.data.midi;

                    // Map CC value to RPM reference:
                    
                    Serial.print("midi received in input-handler.");
                    ReferenceState r;
                    r.angleDeg      = 0.0f;
                    r.velRPM        = midiCCToRPM(m.value);
                    r.enabled       = true;
                    r.valid         = true;

                    referenceSetFrom(RefSource::Midi, r);
                    referenceSetMode(RefSource::Midi);
                    break;
                }
            }
        }
    }
}

void startInputHandler()
{
    // One queue for all input events from all drivers
    g_inputQueue = xQueueCreate(16, sizeof(InputEvent));

    // Start drivers, sharing this queue
    footSwitchInit(g_inputQueue);
    //expPedalInit(g_inputQueue);
    midiInit(g_inputQueue); // if you have a MIDI driver using InputEvent

    // Start the input handler task
    xTaskCreate(
        inputHandlerTask,
        "InputHandlerTask",
        4096,
        nullptr,
        4,          // priority (tweak as needed)
        nullptr
    );
}
