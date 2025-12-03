#include "reference.h"

struct AllReferences {
    ReferenceState midi;
    ReferenceState expPedal;
    ReferenceState footSwitch;
    RefSource      activeSource;
};

static AllReferences g_refs;
static portMUX_TYPE  refMux = portMUX_INITIALIZER_UNLOCKED;

void referenceInit()
{
    portENTER_CRITICAL(&refMux);
    g_refs.midi       = { 0.0f, 0.0f, false, false };
    g_refs.expPedal   = { 0.0f, 0.0f, false, false };
    g_refs.footSwitch = { 0.0f, 0.0f, false, false };
    g_refs.activeSource = RefSource::Midi;  // default
    portEXIT_CRITICAL(&refMux);
}

void referenceSetFrom(RefSource src, const ReferenceState &ref)
{
    portENTER_CRITICAL(&refMux);
    switch (src) {
        case RefSource::Midi:       g_refs.midi       = ref; break;
        case RefSource::ExpPedal:   g_refs.expPedal   = ref; break;
        case RefSource::FootSwitch: g_refs.footSwitch = ref; break;
    }
    portEXIT_CRITICAL(&refMux);
}

void referenceSetMode(RefSource mode)
{
    portENTER_CRITICAL(&refMux);
    g_refs.activeSource = mode;
    portEXIT_CRITICAL(&refMux);
}

RefSource referenceGetMode()
{
    portENTER_CRITICAL(&refMux);
    RefSource m = g_refs.activeSource;
    portEXIT_CRITICAL(&refMux);
    return m;
}

void referenceGetActive(ReferenceState &out)
{
    portENTER_CRITICAL(&refMux);
    RefSource mode = g_refs.activeSource;
    ReferenceState chosen;

    switch (mode) {
        case RefSource::Midi:       chosen = g_refs.midi;       break;
        case RefSource::ExpPedal:   chosen = g_refs.expPedal;   break;
        case RefSource::FootSwitch: chosen = g_refs.footSwitch; break;
    }

    portEXIT_CRITICAL(&refMux);
    out = chosen;
}
