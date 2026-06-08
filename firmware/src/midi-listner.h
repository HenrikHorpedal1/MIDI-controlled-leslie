//midi.h
#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern volatile uint32_t g_usbMidiNotifies;
extern volatile uint32_t g_noteOnSeen;
extern volatile uint32_t g_noteOnMatched;

struct MidiTaskParams {
  QueueHandle_t inputQueue;
  QueueHandle_t clockQueue;
};

void midiListnerTask(void *pvParameters);
