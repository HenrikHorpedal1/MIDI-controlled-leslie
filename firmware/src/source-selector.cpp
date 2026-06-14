#include "source-selector.h"

#include "input_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// --- Pin assignments ---
static constexpr int PIN_ROTARY_ADC = 14;   // A7 — resistance ladder input

// --- ADC thresholds (midpoints between measured positions) ---
// Measured with 10k ladder resistors: voltage levels ~745 / 1509 / 2285 / 3099.
// Per the schematic, rotary detent 1 taps the node nearest 3V3, so the ladder
// reads DESCENDING: pos1 highest, pos4 lowest.
static constexpr int THR_3_4 = 2692;  // above → pos 1 (Footswitch)
static constexpr int THR_2_3 = 1897;  // above → pos 2 (ExpressionPedal)
static constexpr int THR_1_2 = 1127;  // above → pos 3 (MidiKeyboard), below → pos 4 (MidiBeatSync)

// --- Debounce: require this many consecutive stable readings before committing ---
static constexpr int DEBOUNCE_COUNT = 3;

static const char* sourceName(InputSource s) {
    return s == InputSource::Footswitch      ? "Footswitch"      :
           s == InputSource::ExpressionPedal ? "ExpressionPedal" :
           s == InputSource::MidiKeyboard    ? "MidiKeyboard"    :
                                               "MidiBeatSync";
}

static InputSource adcToSource(int raw) {
    if (raw > THR_3_4) return InputSource::Footswitch;
    if (raw > THR_2_3) return InputSource::ExpressionPedal;
    if (raw > THR_1_2) return InputSource::MidiKeyboard;
    return InputSource::MidiBeatSync;
}

void sourceSelectorTask(void* pvParameters) {
    QueueHandle_t inputQueue = static_cast<QueueHandle_t>(pvParameters);

    pinMode(PIN_ROTARY_ADC, INPUT);

    InputSource pending   = InputSource::Footswitch;
    int         stableFor = 0;

    // No source committed yet: the first stable reading always counts as a
    // change so the handler learns the position the knob booted in.
    InputSource current   = InputSource::Footswitch;
    bool        haveSource = false;

    for (;;) {
        const int raw = analogRead(PIN_ROTARY_ADC);
        const InputSource candidate = adcToSource(raw);

        if (candidate == pending) {
            stableFor++;
        } else {
            pending   = candidate;
            stableFor = 1;
        }

        if (stableFor >= DEBOUNCE_COUNT) {
            if (!haveSource || candidate != current) {
                current    = candidate;
                haveSource = true;

                if (inputQueue) {
                    InputEvent ev{};
                    ev.type        = EventType::SourceChange;
                    ev.data.source = current;
                    (void)xQueueSend(inputQueue, &ev, 0);
                }

                Serial.printf("[source] → %s\n", sourceName(current));
            }
            stableFor = DEBOUNCE_COUNT; // clamp to avoid overflow
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // poll at 5 Hz
    }
}
