# Arduino Nano ESP32 — Pin Overview

GPIO mapping for use when programming as a generic ESP32-S3.

## Footswitch / Expression Pedal (shared TRS socket)

The TRS socket is shared between the footswitch matrix and the expression pedal.
The SPDT (see Mode Selector below) routes the socket to the correct circuit based on the rotary switch position.

### Footswitch matrix (dual switch + LED demux)

Pins are multiplexed — each pin acts as input or output depending on the operation (read switch A, read switch B, light LED A, light LED B1, light LED B2).

| Variable | Arduino Pin | ESP32-S3 GPIO |
|----------|-------------|---------------|
| PIN_A    | D5          | GPIO8         |
| PIN_B    | D6          | GPIO9         |
| PIN_C    | D4          | GPIO7         |

## CAN Breakout (SPI)

| Signal | Arduino Pin | ESP32-S3 GPIO | Notes                          |
|--------|-------------|---------------|--------------------------------|
| CS     | D10         | GPIO21        | Chip Select                    |
| MOSI   | D11         | GPIO38        | Controller Out / Peripheral In |
| MISO   | D12         | GPIO47        | Controller In / Peripheral Out |
| SCK    | D13         | GPIO48        | SPI Clock                      |
| INT    | D9          | GPIO18        | Interrupt                      |

## IR Sensors

| Sensor | Arduino Pin | ESP32-S3 GPIO |
|--------|-------------|---------------|
| IR_A   | D7          | GPIO10        |
| IR_B   | D8          | GPIO17        |
| IR_C   | D9          | GPIO18        |

## Mode Selector (Rotary Switch + SPDT)

| Signal           | Arduino Pin | ESP32-S3 GPIO | Notes                                     |
|------------------|-------------|---------------|-------------------------------------------|
| Rotary ADC input | A7          | GPIO14        | Resistance ladder, 4 positions            |
| SPDT control     | D2          | GPIO5         | HIGH = Footswitch, LOW = Exp Pedal / MIDI |
