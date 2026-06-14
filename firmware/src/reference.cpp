#include "reference.h"

#include "freertos/FreeRTOS.h"

static Reference    g_ref  = { DriveMode::Velocity, 0.0f, 0.0f };
static portMUX_TYPE refMux = portMUX_INITIALIZER_UNLOCKED;

void referenceInit()
{
    portENTER_CRITICAL(&refMux);
    g_ref = { DriveMode::Velocity, 0.0f, 0.0f };
    portEXIT_CRITICAL(&refMux);
}

void referenceSet(const Reference &ref)
{
    portENTER_CRITICAL(&refMux);
    g_ref = ref;
    portEXIT_CRITICAL(&refMux);
}

void referenceGet(Reference &out)
{
    portENTER_CRITICAL(&refMux);
    out = g_ref;
    portEXIT_CRITICAL(&refMux);
}
