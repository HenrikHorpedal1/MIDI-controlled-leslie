// clock_sim.cpp
#include "clock_sim.h"
#include "clock_sync.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

// 40 BPM × 24 ticks/beat = 960 ticks/min → 62500 µs/tick = 62.5 ms/tick.
// FreeRTOS can't schedule at 62.5 ms, so alternate 62 ms / 63 ms to average exactly.
static constexpr double   SIM_BPM          = 40.0;
static constexpr uint32_t TICK_PERIOD_A_MS = 62;
static constexpr uint32_t TICK_PERIOD_B_MS = 63;

void clockSimTask(void* pvQueueHandle) {
  QueueHandle_t q = static_cast<QueueHandle_t>(pvQueueHandle);

  // Give the rest of the system 500 ms to finish initialising.
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.printf("[clock_sim] starting at %.1f BPM (alternating %u/%u ms)\n",
                SIM_BPM, TICK_PERIOD_A_MS, TICK_PERIOD_B_MS);

  // Send Start so clockSyncTask considers the clock running.
  {
    ClockMsg msg{};
    msg.type = ClockMsgType::Start;
    msg.t_us = esp_timer_get_time();
    xQueueSend(q, &msg, 0);
  }

  bool useA = true;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const uint32_t ms = useA ? TICK_PERIOD_A_MS : TICK_PERIOD_B_MS;
    useA = !useA;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ms));

    ClockMsg msg{};
    msg.type = ClockMsgType::Tick;
    msg.t_us = esp_timer_get_time();
    xQueueSend(q, &msg, 0);
  }
}
