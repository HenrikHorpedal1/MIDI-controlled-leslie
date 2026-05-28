#include "reference.h"

#include "freertos/FreeRTOS.h"

static ReferenceState g_ref[2] = {{ 0.0f, 0.0f }, { 0.0f, 0.0f }};
static portMUX_TYPE   refMux   = portMUX_INITIALIZER_UNLOCKED;

void referenceInit()
{
    portENTER_CRITICAL(&refMux);
    g_ref[0] = { 0.0f, 0.0f };
    g_ref[1] = { 0.0f, 0.0f };
    portEXIT_CRITICAL(&refMux);
}

void referenceSet(Rotor rotor, const ReferenceState &ref)
{
    portENTER_CRITICAL(&refMux);
    g_ref[static_cast<uint8_t>(rotor)] = ref;
    portEXIT_CRITICAL(&refMux);
}

void referenceGet(Rotor rotor, ReferenceState &out)
{
    portENTER_CRITICAL(&refMux);
    out = g_ref[static_cast<uint8_t>(rotor)];
    portEXIT_CRITICAL(&refMux);
}
