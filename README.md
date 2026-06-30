# MIDI-controlled Leslie

Monorepo for a MIDI-controlled Leslie speaker build — firmware, hardware design, and the Python
tooling used to configure the motors and analyse measurements.

## Demos

**Tempo sync** — horn speed locked to MIDI clock

https://github.com/user-attachments/assets/804f9586-0638-46d0-b1d2-34bc3389345e

**Rock'n'roll**

https://github.com/user-attachments/assets/c1948277-36b1-4e50-a5de-1323c6d4f3f3

**Black Hole Sun**

https://github.com/user-attachments/assets/2e4643aa-8b3f-4453-b8d9-3a8be26efdcb

## Cloning

This repo uses a git submodule (upstream moteus, for its `utils/` scripts):

```bash
git clone --recurse-submodules <url>
# or, after a plain clone:
git submodule update --init
```

## Layout

| Path          | Contents |
|---------------|----------|
| `firmware/`   | Arduino/ESP32-S3 firmware. This is the Arduino **library root** (`library.properties`, `src/`, `examples/`, `main/`). |
| `cad/`        | Mechanical design (Fusion 360 cabinet, fixtures, laser-cut output). |
| `electronics/`| PCB / wiring (CAN-FD board gerbers, test schematics). |
| `docs/`       | Build documentation. |
| `software/`   | Python tooling (uv project): moteus configuration / data collection and signal analysis, plus the upstream moteus submodule. |

Several directories are scaffolded with stub READMEs and will be populated as content is
migrated in.

## Firmware

Built with the Arduino CLI; `firmware/` is the library root:

```bash
arduino-cli compile --fqbn arduino:esp32:nano_nora firmware/examples/<example>/
arduino-cli upload  --fqbn arduino:esp32:nano_nora --port /dev/cu.usbmodem* firmware/main/
```

Target board: Arduino Nano ESP32 (ESP32-S3). Motor control uses the mjbots moteus driver over
CAN-FD (MCP2518FD via SPI). See `firmware/pin-overview.md` for GPIO mapping.

## Software (Python)

Managed with [uv](https://docs.astral.sh/uv/). See `software/README.md`.

```bash
cd software && uv sync
uv run moteus_tool --help
```
