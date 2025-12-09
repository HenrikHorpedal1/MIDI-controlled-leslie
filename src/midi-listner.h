//midi.h
#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void midiListnerTask(void *pvParameters);
