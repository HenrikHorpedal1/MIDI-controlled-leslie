// clock_sim.h
// Simulated MIDI clock — pushes Tick messages to the clock queue at a fixed BPM.
// Use in place of the MIDI listener when testing without a real clock source.
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void clockSimTask(void* pvQueueHandle);
