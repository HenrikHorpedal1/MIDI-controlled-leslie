// input_handler.h
#pragma once

#include <Arduino.h>

// Call once from setup() to start the input handler + drivers.
// After this, the input handler task will run and push references into
// the reference module based on events from footswitch / exp pedal / midi.
void startInputHandler();
