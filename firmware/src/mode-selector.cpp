#include "mode-selector.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Pin assignments ---
static constexpr int PIN_ROTARY_ADC = 14;   // A7 — resistance ladder input
static constexpr int PIN_SPDT_CTRL  = 5;    // D2 — routes TRS socket (HIGH=footswitch, LOW=exp pedal)

// --- ADC thresholds (midpoints between measured positions) ---
// Measured with 10k ladder resistors: pos1=745, pos2=1509, pos3=2285, pos4=3099
// Note: the ladder now reads ascending (pos1 lowest, pos4 highest).
static constexpr int THR_1_2 = 1127;  // below → pos 1 (Footswitch)
static constexpr int THR_2_3 = 1897;  // below → pos 2 (ExpressionPedal)
static constexpr int THR_3_4 = 2692;  // below → pos 3 (MidiKeyboard), above → pos 4 (MidiBeatSync)

// --- Debounce: require this many consecutive stable readings before committing ---
static constexpr int DEBOUNCE_COUNT = 3;

static portMUX_TYPE  s_mux    = portMUX_INITIALIZER_UNLOCKED;
static ControlSource s_source = ControlSource::Footswitch;

ControlSource modeSelectorGetSource() {
    portENTER_CRITICAL(&s_mux);
    ControlSource src = s_source;
    portEXIT_CRITICAL(&s_mux);
    return src;
}

static ControlSource adcToSource(int raw) {
    if (raw < THR_1_2) return ControlSource::Footswitch;
    if (raw < THR_2_3) return ControlSource::ExpressionPedal;
    if (raw < THR_3_4) return ControlSource::MidiKeyboard;
    return ControlSource::MidiBeatSync;
}

static void applySPDT(ControlSource src) {
    digitalWrite(PIN_SPDT_CTRL, src == ControlSource::Footswitch ? HIGH : LOW);
}

void modeSelectorTask(void* pvParameters) {
    pinMode(PIN_ROTARY_ADC, INPUT);
    pinMode(PIN_SPDT_CTRL, OUTPUT);

    ControlSource pending   = ControlSource::Footswitch;
    int           stableFor = 0;

    applySPDT(s_source);

    for (;;) {
        const int raw = analogRead(PIN_ROTARY_ADC);
        const ControlSource candidate = adcToSource(raw);

        if (candidate == pending) {
            stableFor++;
        } else {
            pending   = candidate;
            stableFor = 1;
        }

        if (stableFor >= DEBOUNCE_COUNT) {
            portENTER_CRITICAL(&s_mux);
            const bool changed = (candidate != s_source);
            if (changed) {
                s_source = candidate;
            }
            portEXIT_CRITICAL(&s_mux);

            if (changed) {
                applySPDT(candidate);
                Serial.printf("[mode] source → %s\n",
                    candidate == ControlSource::Footswitch      ? "Footswitch"      :
                    candidate == ControlSource::ExpressionPedal ? "ExpressionPedal" :
                    candidate == ControlSource::MidiKeyboard    ? "MidiKeyboard"    :
                                                                  "MidiBeatSync");
            }
            stableFor = DEBOUNCE_COUNT; // clamp to avoid overflow
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // poll at 5 Hz
    }
}
