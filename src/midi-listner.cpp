// midi.cpp
#include "midi-listner.h"

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "tusb.h"

#include "input_event.h"

static constexpr uint8_t MIDI_TARGET_CHANNEL = 1;
static constexpr uint8_t MIDI_TARGET_CC = 7; 
static constexpr uint8_t PAD_CHANNEL   = 10;  
static constexpr uint8_t PAD_NOTE_E1   = 40;
static constexpr uint8_t PAD_NOTE_F1   = 41;
static constexpr uint8_t PAD_NOTE_FS1  = 42;


static Adafruit_USBD_MIDI usb_midi;
static MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

static QueueHandle_t s_inputQueue = nullptr;

static inline bool isPadNote(uint8_t n) {
    return n == PAD_NOTE_E1 || n == PAD_NOTE_F1 || n == PAD_NOTE_FS1;
}

static MidiButtonEvent noteToButton(uint8_t note){
    if (note == PAD_NOTE_E1)  return MidiButtonEvent::BUTTON0;
    if (note == PAD_NOTE_F1)  return MidiButtonEvent::BUTTON1;
    if (note == PAD_NOTE_FS1) return MidiButtonEvent::BUTTON2;

    return MidiButtonEvent::BUTTON0;
}

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

        bool haveEvent = false;
        InputEvent ev{};

        if (type == midi::ControlChange &&
            ch == MIDI_TARGET_CHANNEL &&
            d1 == MIDI_TARGET_CC) {

            ev.source = InputSource::MidiCC;
            ev.data.midiCC.value = d2;   // CC value 0..127
            haveEvent = true;
            Serial.println("CC");
        }
        else if ((type == midi::NoteOn || type == midi::NoteOff) &&
                 ch == PAD_CHANNEL && isPadNote(d1)) {

            ev.source = InputSource::MidiButton;
            ev.data.midiButton = noteToButton(d1);
            haveEvent = true;
            Serial.println("Button");
        }

        if (haveEvent && s_inputQueue) {
            xQueueSend(s_inputQueue, &ev, 0);
        }    
    }
}

void midiListnerTask(void *pvParameters)
{

    s_inputQueue = static_cast<QueueHandle_t>(pvParameters);

    TinyUSBDevice.setManufacturerDescriptor("Leslie MIDI Control");
    TinyUSBDevice.setProductDescriptor("Nano ESP32 USB-MIDI");

    MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.turnThruOff();

    Serial.println("starting midi task");
    for (;;)
    {
        if (TinyUSBDevice.mounted() && tud_ready()) {
            handleIncomingMIDI();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
