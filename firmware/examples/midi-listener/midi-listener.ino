#include "clock_sync.h"
#include "midi-listener.h"
#include "input_event.h"

static QueueHandle_t inputQueue;
static QueueHandle_t clockQueue;

static MidiTaskParams midiParams;

void setup() {
  Serial.begin(115200);

  inputQueue = xQueueCreate(32, sizeof(InputEvent));
  clockQueue = xQueueCreate(128, sizeof(ClockMsg));  // 24ppqn; 128 is plenty

  midiParams = { inputQueue, clockQueue };

  xTaskCreatePinnedToCore(midiListenerTask, "midi", 4096, &midiParams, 3, nullptr, 1);
  xTaskCreatePinnedToCore(clockSyncTask,   "clk",  4096, clockQueue,  2, nullptr, 1);
}

void loop() {
static uint32_t last = 0;
if (millis() - last >= 100) {
  last += 100;

  Serial.printf("run=%d lock=%d bpm=%.2f period=%.0fus err=%.0fus phase=%.2f tick=%llu\n",
    (int)clockSyncIsRunning(),
    (int)clockSyncIsLocked(),
    clockSyncGetBpm(),
    clockSyncGetPeriodUs(),
    clockSyncGetLastErrUs(),
    clockSyncGetPhase(),
    (unsigned long long)clockSyncGetTickCount()
  );
}

  // Your normal app loop...
}
