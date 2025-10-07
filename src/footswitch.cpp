#include <Arduino.h>
#include "footswitch.h"

#define PIN_A 13
#define PIN_B 12
#define PIN_C 14

volatile bool switchA = false;
volatile bool switchB = false;

void footSwitchTask(void *pvParameters) {
  const TickType_t ledOnTime  = 1;
  const TickType_t ledOffTime = 9;

  for (;;) {
    pinMode(PIN_B, OUTPUT);
    digitalWrite(PIN_B, LOW);
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_C, INPUT);
    switchA = (digitalRead(PIN_A) == LOW);

    pinMode(PIN_C, OUTPUT);
    digitalWrite(PIN_C, LOW);
    pinMode(PIN_B, INPUT_PULLUP);
    pinMode(PIN_A, INPUT);
    switchB = (digitalRead(PIN_B) == LOW);

    if (switchA) {
      pinMode(PIN_A, OUTPUT);
      pinMode(PIN_C, OUTPUT);
      pinMode(PIN_B, INPUT);
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_C, LOW);
      vTaskDelay(ledOnTime);
      pinMode(PIN_A, INPUT);
      pinMode(PIN_C, INPUT);
    } else vTaskDelay(ledOnTime);

    if (switchB) {
      pinMode(PIN_B, OUTPUT);
      pinMode(PIN_A, OUTPUT);
      pinMode(PIN_C, INPUT);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_A, LOW);
      vTaskDelay(ledOnTime);
      pinMode(PIN_B, INPUT);
      pinMode(PIN_A, INPUT);
    } else {
      pinMode(PIN_C, OUTPUT);
      pinMode(PIN_B, OUTPUT);
      pinMode(PIN_A, INPUT);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_B, LOW);
      vTaskDelay(ledOnTime);
      pinMode(PIN_C, INPUT);
      pinMode(PIN_B, INPUT);
    }
    vTaskDelay(ledOffTime);
  }
}

