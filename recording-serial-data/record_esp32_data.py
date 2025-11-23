#!/usr/bin/env python3
"""
record_esp32_data.py
--------------------
Automatically detects an ESP32 serial port on macOS, records streaming CSV data,
and saves it to a timestamped .csv file for later analysis in Python.
"""

import csv
import time
from datetime import datetime
from pathlib import Path
import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------
BAUD = 500000      # Recommended for ESP32 on macOS
TIMEOUT = 1        # seconds to wait for serial data
SHOW_EVERY = 50    # how often to print a sample line to terminal
# ---------------------------------------------------------------------


def find_esp32_port() -> str:
    """
    Try to automatically find the ESP32's serial port.
    Looks for common USB-UART chips (CP210x, CH340, FTDI, etc.)
    """
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if ("usbserial" in p.device.lower()
            or "usbmodem" in p.device.lower()
            or "SLAB_USBtoUART" in p.device
            or "wchusbserial" in p.device.lower()
            or "esp32" in p.description.lower()):
            return p.device
    raise RuntimeError("Could not find ESP32 serial port. "
                       "Plug it in and check with: ls /dev/tty.*")


def main():
    port = find_esp32_port()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(f"esp32_log_{timestamp}.csv")
    print(f"📡 Found ESP32 on {port}")
    print(f"💾 Logging to {out_path.resolve()}")
    print("Press Ctrl-C to stop.\n")

    with serial.Serial(port, BAUD, timeout=TIMEOUT) as ser, out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        line_count = 0

        # Wait for a valid header (first CSV line)
        print("Waiting for CSV header from ESP32...")
        header = ""
        while True:
            raw = ser.readline().decode(errors="replace").strip()
            if "," in raw:
                header = raw
                break
        cols = [h.strip() for h in header.split(",")]
        writer.writerow(cols)
        print(f"✅ Header detected: {cols}")

        try:
            while True:
                raw = ser.readline().decode(errors="replace").strip()
                if not raw:
                    continue
                parts = [p.strip() for p in raw.split(",")]
                if len(parts) != len(cols):
                    continue  # skip malformed line
                writer.writerow(parts)
                line_count += 1
                if line_count % SHOW_EVERY == 0:
                    print(f"[{line_count:>6}] {raw}")
        except KeyboardInterrupt:
            print(f"\n🛑 Logging stopped by user. Saved {line_count} samples to {out_path.name}")


if __name__ == "__main__":
    main()
