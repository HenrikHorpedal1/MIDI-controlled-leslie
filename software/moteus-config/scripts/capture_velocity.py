#!/usr/bin/python3 -B

import asyncio
import csv
import moteus
import time

TARGET = 2
VELOCITY = 2.5
DURATION = 5.0
OUTPUT = 'capture.csv'

async def main():
    m = moteus.Controller(id=TARGET)
    s = moteus.Stream(m)

    await s.write_message(b"tel stop")
    await s.flush_read()
    await s.command(b"d stop")

    qr = moteus.QueryResolution()
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.q_current = moteus.F32

    await s.command(f"d pos nan {VELOCITY} nan a4".encode())
    await asyncio.sleep(1.0)

    print(f"Capturing {DURATION}s at {VELOCITY} rev/s...")
    rows = []
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t > DURATION:
            break
        state = await m.query(query_override=qr)
        rows.append({
            'time': t,
            'position': state.values[moteus.Register.POSITION],
            'velocity': state.values[moteus.Register.VELOCITY],
            'q_A': state.values[moteus.Register.Q_CURRENT],
        })

    await s.command(b"d stop")

    with open(OUTPUT, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time', 'position', 'velocity', 'q_A'])
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved {len(rows)} samples to {OUTPUT}")

if __name__ == '__main__':
    asyncio.run(main())
