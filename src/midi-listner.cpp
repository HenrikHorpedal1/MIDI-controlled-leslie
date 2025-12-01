// midi.cpp
#include "midi-listner.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "tusb.h"              // tud_ready()

#include "input_event.h"      // InputEvent, MidiCCEvent, InputSource

// ---------------- Config ----------------
// MIDI channel to listen on (1..16)
static constexpr uint8_t MIDI_TARGET_CHANNEL = 1;

// Optional: specific CC number to react to.
// If you want all CCs on that channel, set this to 0xFF.
static constexpr uint8_t MIDI_TARGET_CC = 7;   // example: CC7 (volume)

// ---------------- USB MIDI device ----------------

static Adafruit_USBD_MIDI usb_midi;
static MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// Shared input queue (provided by input handler)
static QueueHandle_t s_inputQueue = nullptr;

// ---------------- Internal helpers ----------------

static void handleIncomingMIDI()
{
    // Process all pending messages
    while (MIDI.read()) {
        midi::MidiType type = MIDI.getType();

        // Ignore system messages (Clock, ActiveSensing, SysEx, etc.)
        if (type >= midi::SystemExclusive) {
            continue;
        }

        uint8_t ch = MIDI.getChannel();  // 1..16
        uint8_t d1 = MIDI.getData1();
        uint8_t d2 = MIDI.getData2();

        // We only care about Control Change
        if (type != midi::ControlChange) {
            continue;
        }

        // Filter on channel
        if (ch != MIDI_TARGET_CHANNEL) {
            continue;
        }

        // Filter on CC number (if configured)
        if (MIDI_TARGET_CC != 0xFF && d1 != MIDI_TARGET_CC) {
            continue;
        }

        // Build an InputEvent for the input handler
        if (s_inputQueue != nullptr) {
            InputEvent ev;
            ev.source              = InputSource::Midi;
            ev.data.midi.channel    = ch;
            ev.data.midi.controller = d1;   // CC number
            ev.data.midi.value      = d2;   // CC value 0..127
            //Serial.print("Received CC, vale: ");
            //Serial.println(ev.data.midi.value);

            // Non-blocking send; drop if queue is full
            xQueueSend(s_inputQueue, &ev, 0);
        }
    }
}

static void midiTask(void *pvParameters)
{
    (void)pvParameters;

    // Identify the USB device (optional, but nice)
    TinyUSBDevice.setManufacturerDescriptor("Leslie MIDI Lab");
    TinyUSBDevice.setProductDescriptor("Nano ESP32 USB-MIDI");

    // MIDI setup: same pattern as your working sketch
    MIDI.begin(MIDI_CHANNEL_OMNI);  // listen to all, we filter by channel
    MIDI.turnThruOff();             // don't echo back to DAW

    for (;;)
    {
        // Only try to read when the USB device is enumerated and ready
        if (TinyUSBDevice.mounted() && tud_ready()) {
            handleIncomingMIDI();
        }

        // Small delay to avoid busy-looping
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ---------------- Public API ----------------

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
