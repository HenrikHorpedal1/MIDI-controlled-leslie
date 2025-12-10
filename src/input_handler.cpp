// input_handler.cpp
#include <Arduino.h>
#include "input_event.h"
#include "reference.h"
#include "ramp-trajectory.h"

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

            switch (ev.source) {

                case InputSource::Footswitch:
                {
                    const FootswitchState &fs = ev.data.foot;   
                    //Serial.print("Press at");
                    //Serial.println(micros());

                    if (fs.swA)
                    {
                        // A pressed → choose speed based on B
                        if (fs.swB)
                        {
                            rampTrajectoryCommand(SpeedCommand::TREMOLO, RefSource::Footswitch);
                        }
                        else if (!fs.swB)
                        {
                            rampTrajectoryCommand(SpeedCommand::CHORALE, RefSource::Footswitch);
                        }
                    }
                    else  // !fs.swA -> stop
                    {
                        rampTrajectoryCommand(SpeedCommand::STOP,RefSource::Footswitch);
                    }

                    referenceSetMode(RefSource::Footswitch);
                    //Serial.println("Set reference from foot switch.");
                    break;
                }

                case InputSource::ExpPedal:
                {
                    //TODO: Implement when driver is implemented.
                    break;
                }

                case InputSource::MidiCC:
                {
                    const MidiCCEvent &m = ev.data.midiCC;

                    // Map CC value to RPM reference:
                    
                    //Serial.print("midi received in input-handler.");
                    ReferenceState r;
                    r.angleDeg      = 0.0f;
                    r.velRPM        = midiCCToRPM(m.value);

                    referenceSetFrom(RefSource::MidiCC, r);
                    referenceSetMode(RefSource::MidiCC);
                    break;
                }

                case InputSource::MidiButton:
                {
                    switch (ev.data.midiButton) {
                            case MidiButtonEvent::BUTTON0: rampTrajectoryCommand(SpeedCommand::CHORALE, RefSource::MidiButton); break;
                            case MidiButtonEvent::BUTTON1: rampTrajectoryCommand(SpeedCommand::STOP,RefSource::MidiButton);    break;
                            case MidiButtonEvent::BUTTON2: rampTrajectoryCommand(SpeedCommand::TREMOLO,RefSource::MidiButton); break;
                        }
                    referenceSetMode(RefSource::MidiButton);
                    break;
                }
            }
        }
    }
}
