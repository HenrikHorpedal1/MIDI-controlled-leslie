#include <Arduino.h>
#include "footswitch.h"
#include "input_event.h"

#define PIN_A 8
#define PIN_B 9
#define PIN_C 7

const TickType_t LED_ON_TIME  = 1;
const TickType_t LED_OFF_TIME = 9;

volatile bool switchA = false;
volatile bool switchB = false;

static QueueHandle_t s_inputQueue = nullptr;

static bool readSwitchA() {
    pinMode(PIN_B, OUTPUT);
    digitalWrite(PIN_B, LOW);
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_C, INPUT);

    return (digitalRead(PIN_A) == LOW);
}

static bool readSwitchB(){
    pinMode(PIN_C, OUTPUT);
    digitalWrite(PIN_C, LOW);
    pinMode(PIN_B, INPUT_PULLUP);
    pinMode(PIN_A, INPUT);

    return (digitalRead(PIN_B) == LOW); 
}

static void lightLedA(){
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_C, OUTPUT);
    pinMode(PIN_B, INPUT);

    digitalWrite(PIN_A, HIGH);
    digitalWrite(PIN_C, LOW);

    vTaskDelay(LED_ON_TIME);

    //reset
    pinMode(PIN_A, INPUT);
    pinMode(PIN_C, INPUT);
}

static void lightLedB1(){
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_C, INPUT);

    digitalWrite(PIN_B, HIGH);
    digitalWrite(PIN_A, LOW);

    vTaskDelay(LED_ON_TIME);

    //reset
    pinMode(PIN_B, INPUT);
    pinMode(PIN_A, INPUT);
}

static void lightLedB2(){
    pinMode(PIN_C, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_A, INPUT);

    digitalWrite(PIN_C, HIGH);
    digitalWrite(PIN_B, LOW);

    vTaskDelay(LED_ON_TIME);

    //reset
    pinMode(PIN_C, INPUT);
    pinMode(PIN_B, INPUT);
}

void footSwitchTask(void *pvParameters)
{

    s_inputQueue = static_cast<QueueHandle_t>(pvParameters);

    bool lastA = false;
    bool lastB = false;

    for (;;)
    {
        // --- DEBUG: serial footswitch sim — '1'=chorale(A) '2'=tremolo(A+B) '0'=stop ---
        if (Serial.available() && s_inputQueue != nullptr) {
            char c = Serial.read();
            bool sa = false, sb = false;
            if      (c == '1') { sa = true;  sb = false; }
            else if (c == '2') { sa = true;  sb = true;  }
            else if (c == '0') { sa = false; sb = false; }
            if (c == '0' || c == '1' || c == '2') {
                InputEvent ev;
                ev.source    = InputSource::Footswitch;
                ev.data.foot = FootswitchState{ sa, sb };
                xQueueSend(s_inputQueue, &ev, 0);
            }
        }
        // --- END DEBUG ---

        bool a = readSwitchA();
        bool b = readSwitchB();

        //notify if there was a change
        if ((a != lastA || b != lastB) && s_inputQueue != nullptr) {
            InputEvent ev;
            ev.source    = InputSource::Footswitch;
            ev.data.foot = FootswitchState{ a, b };

            xQueueSend(s_inputQueue, &ev, 0);
        }

        lastA = a;
        lastB = b;

        // LED control
        if (a) {
            lightLedA();
        } else {
            vTaskDelay(LED_ON_TIME);
        }

        if (b) {
            lightLedB1();
        } else {
            lightLedB2();
        }

        vTaskDelay(LED_OFF_TIME);
    }
}

