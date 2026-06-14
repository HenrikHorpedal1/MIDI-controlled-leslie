//midi.h
#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// MIDI Control Change assignments (channel MIDI_TARGET_CHANNEL).
// Shared with input_handler so both agree on the numbers.
static constexpr uint8_t MIDI_RATE_CC    = 7;   // free-running speed -> RPM
static constexpr uint8_t MIDI_SUBDIV_CC  = 20;  // beat-sync subdivision select
static constexpr uint8_t MIDI_SUSTAIN_CC = 64;  // sustain/damper pedal (keyboard mode)

extern volatile uint32_t g_usbMidiNotifies;
extern volatile uint32_t g_noteOnSeen;
extern volatile uint32_t g_noteOnMatched;

struct MidiTaskParams {
  QueueHandle_t inputQueue;
  QueueHandle_t clockQueue;
};

void midiListnerTask(void *pvParameters);
