//midi.h
#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Start the MIDI driver.
// - inputQueue: shared queue of InputEvent (from input_events.h)
void midiInit(QueueHandle_t inputQueue);
