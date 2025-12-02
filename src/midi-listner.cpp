// midi.cpp
#include "midi-listner.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "tusb.h"

#include "input_event.h"

static constexpr uint8_t MIDI_TARGET_CHANNEL = 1;
static constexpr uint8_t MIDI_TARGET_CC = 7;   // example: CC7 (volume)

static Adafruit_USBD_MIDI usb_midi;
static MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

static QueueHandle_t s_inputQueue = nullptr;

static void handleIncomingMIDI()
{
    while (MIDI.read()) {
        midi::MidiType type = MIDI.getType();

        if (type >= midi::SystemExclusive) {
            continue;
        }

        uint8_t ch = MIDI.getChannel();  // 1..16
        uint8_t d1 = MIDI.getData1();
        uint8_t d2 = MIDI.getData2();

        if (type != midi::ControlChange) {
            continue;
        }

        if (ch != MIDI_TARGET_CHANNEL) {
            continue;
        }

        if (MIDI_TARGET_CC != 0xFF && d1 != MIDI_TARGET_CC) {
            continue;
        }

        if (s_inputQueue != nullptr) {
            InputEvent ev;
            ev.source              = InputSource::Midi;
            ev.data.midi.channel    = ch;
            ev.data.midi.controller = d1;   // CC number
            ev.data.midi.value      = d2;   // CC value 0..127
            
            xQueueSend(s_inputQueue, &ev, 0);
        }
    }
}

static void midiTask(void *pvParameters)
{
    (void)pvParameters;

    TinyUSBDevice.setManufacturerDescriptor("Leslie MIDI Control");
    TinyUSBDevice.setProductDescriptor("Nano ESP32 USB-MIDI");

    MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.turnThruOff();

    for (;;)
    {
        if (TinyUSBDevice.mounted() && tud_ready()) {
            handleIncomingMIDI();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void midiInit(QueueHandle_t inputQueue)
{
    s_inputQueue = inputQueue;

    xTaskCreate(
        midiTask,
        "MidiTask",
        4096,
        nullptr,
        3,      // priority: adjust as needed
        nullptr
    );
}
