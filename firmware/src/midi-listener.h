// midi.h
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <Arduino.h>

// MIDI Control Change assignments (channel MIDI_TARGET_CHANNEL).
// Shared with input_handler so both agree on the numbers.
static constexpr uint8_t MIDI_RATE_CC = 23;   // free-running speed -> RPM
static constexpr uint8_t MIDI_SUBDIV_CC = 20; // beat-sync subdivision select
static constexpr uint8_t MIDI_BUTTON_CC =
    21; // keyboard mode buttons (val 1/2/3; 0 = release)
static constexpr uint8_t MIDI_SUSTAIN_CC =
    64; // sustain/damper pedal (keyboard mode)

struct MidiTaskParams {
  QueueHandle_t inputQueue;
  QueueHandle_t clockQueue;
};

void midiListenerTask(void *pvParameters);
