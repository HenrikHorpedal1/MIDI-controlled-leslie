# Firmware

Arduino/ESP32-S3 firmware for the MIDI-controlled Leslie. This directory is also the
**Arduino library root** (`library.properties`, `src/`, `examples/`, `main/`).

Target board: **ESP32S3 Dev Module** (`esp32:esp32:esp32s3`).

## Layout

- `src/` — library sources (the actual implementation).
- `main/main.ino` — the main application sketch.
- `examples/` — standalone sketches for testing individual subsystems
  (footswitch, input-handler, jitter-logger, midi-listner, mode-selector,
  one-edge-logger, test-sync).
- `pin-overview.md` —GPIO mapping.

## Dependencies

The exact external library versions known to work are pinned in
[`library.properties`](library.properties) via `depends=`:

Install the ESP32 core and these libraries through the Arduino IDE Library Manager,
or with `arduino-cli lib install`.

## Running it

For the IDE to find the library headers, `firmware/` must be installed as an Arduino
library — symlink it into your sketchbook:

```bash
ln -s "$(pwd)" ~/Documents/Arduino/libraries/MIDI-controlled-leslie
```

### Arduino IDE

1. Open `main/main.ino`, or an example via **File → Examples → MIDI-controlled-leslie**.
2. **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module** and set the board options
   (see table below — they persist per board).
3. Select the port, click **Upload**, open the **Serial Monitor** at 115200 baud.

## Board options (ESP32S3 Dev Module)

| Tools menu setting | FQBN option |
| --- | --- |
| USB CDC On Boot: Enabled | `CDCOnBoot=cdc` |
| CPU Frequency: 240MHz | `CPUFreq=240` |
| Core Debug Level: None | `DebugLevel=none` |
| USB DFU On Boot: Disabled | `DFUOnBoot=default` |
| Erase All Flash Before Upload: Disabled | `EraseFlash=none` |
| Events Run On: Core 1 | `EventsCore=1` |
| Flash Mode: QIO 80MHz | `FlashMode=qio` |
| Flash Size: 16MB | `FlashSize=16M` |
| JTAG Adapter: Disabled | `JTAGAdapter=default` |
| Arduino Runs On: Core 1 | `LoopCore=1` |
| USB Firmware MSC On Boot: Disabled | `MSCOnBoot=default` |
| Partition Scheme: Default 4MB with spiffs | `PartitionScheme=default` |
| PSRAM: OPI PSRAM | `PSRAM=opi` |
| Upload Mode: USB-OTG CDC (TinyUSB) | `UploadMode=cdc` |
| Upload Speed: 921600 | `UploadSpeed=921600` |
| USB Mode: USB-OTG (TinyUSB) | `USBMode=default` |
| Zigbee Mode: Disabled | `ZigbeeMode=default` |

### arduino-cli

The board options are baked into the FQBN so builds are reproducible:

```bash
# run from the repo root (parent of firmware/)
FQBN="esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=default,DebugLevel=none,PSRAM=opi,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default,UploadSpeed=921600"

# compile (swap firmware/main/ for any firmware/examples/<name>/)
arduino-cli compile --fqbn "$FQBN" firmware/main/

# find the port, then upload and monitor
arduino-cli board list
arduino-cli upload  --fqbn "$FQBN" -p /dev/cu.usbmodem* firmware/main/
arduino-cli monitor -p /dev/cu.usbmodem* -c baudrate=115200
```

