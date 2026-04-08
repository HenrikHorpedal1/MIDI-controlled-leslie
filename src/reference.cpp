#include "reference.h"

#include "freertos/FreeRTOS.h"

static ReferenceState g_ref  = { 0.0f, 0.0f };
static portMUX_TYPE   refMux = portMUX_INITIALIZER_UNLOCKED;

void referenceInit()
{
    portENTER_CRITICAL(&refMux);
    g_ref = { 0.0f, 0.0f };
    portEXIT_CRITICAL(&refMux);
}

void referenceSet(const ReferenceState &ref)
{
    portENTER_CRITICAL(&refMux);
    g_ref = ref;
    portEXIT_CRITICAL(&refMux);
}

void referenceGet(ReferenceState &out)
{
    portENTER_CRITICAL(&refMux);
    out = g_ref;
    portEXIT_CRITICAL(&refMux);
}
