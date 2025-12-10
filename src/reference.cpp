#include "reference.h"

struct AllReferences {
    ReferenceState midiCC;
    ReferenceState midiButton;
    ReferenceState expPedal;
    ReferenceState footswitch;
    RefSource      activeSource;
};

static AllReferences g_refs;
static portMUX_TYPE  refMux = portMUX_INITIALIZER_UNLOCKED;

void referenceInit()
{
    portENTER_CRITICAL(&refMux);
    g_refs.midiCC       = { 0.0f, 0.0f };
    g_refs.midiButton   = { 0.0f, 0.0f };
    g_refs.expPedal     = { 0.0f, 0.0f };
    g_refs.footswitch   = { 0.0f, 0.0f };
    g_refs.activeSource = RefSource::Footswitch;  // default
    portEXIT_CRITICAL(&refMux);
}

void referenceSetFrom(RefSource src, const ReferenceState &ref)
{
    portENTER_CRITICAL(&refMux);
    switch (src) {
        case RefSource::MidiCC:     g_refs.midiCC     = ref; break;
        case RefSource::MidiButton: g_refs.midiButton = ref; break;
        case RefSource::ExpPedal:   g_refs.expPedal   = ref; break;
        case RefSource::Footswitch: g_refs.footswitch = ref; break;
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
        case RefSource::MidiCC:     chosen = g_refs.midiCC;     break;
        case RefSource::MidiButton: chosen = g_refs.midiButton; break;
        case RefSource::ExpPedal:   chosen = g_refs.expPedal;   break;
        case RefSource::Footswitch: chosen = g_refs.footswitch; break;
    }

    portEXIT_CRITICAL(&refMux);
    out = chosen;
}
