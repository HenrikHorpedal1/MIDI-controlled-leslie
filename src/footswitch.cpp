#include <Arduino.h>
#include "footswitch.h"

#define PIN_A 13
#define PIN_B 12
#define PIN_C 14

const TickType_t LED_ON_TIME  = 1;
const TickType_t LED_OFF_TIME = 9;

volatile bool switchA = false;
volatile bool switchB = false;

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

void footSwitchTask(void *pvParameters) {

  for (;;) {
    switchA = readSwitchA();
    switchB = readSwitchB();

    if (switchA) {
        lightLedA();
    } else{
        vTaskDelay(LED_ON_TIME);
    }

    if (switchB) {
        lightLedB1();
    } else {
        lightLedB2();
    }
    vTaskDelay(LED_OFF_TIME);
  }
}

