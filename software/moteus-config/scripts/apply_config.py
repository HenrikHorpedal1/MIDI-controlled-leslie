#!/usr/bin/env python3
"""Set one or more moteus config values and write them to persistent storage.

A small helper for the bring-up procedure: apply `conf set` lines (zeroing the
cogging table, switching the output ratio, writing computed PID gains, ...)
without opening tview.  Values are written with `conf write` and then read back
for confirmation.  Does not move the motor.

Examples (run from software/):
    # zero the cogging compensation before encoder compensation
    uv run python moteus-config/scripts/apply_config.py -t 1 motor.cogging_dq_scale=0

    # apply computed position-loop gains
    uv run python moteus-config/scripts/apply_config.py -t 1 \\
        servo.pid_position.kp=91.2 servo.pid_position.ki=2007 servo.pid_position.kd=1.33
"""

import argparse
import asyncio

import moteus


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", "-t", type=int, default=1,
                        help="moteus CAN id (1=drum, 2=horn)")
    parser.add_argument("settings", nargs="+", metavar="KEY=VALUE",
                        help="one or more config assignments, e.g. motor.cogging_dq_scale=0")
    parser.add_argument("--no-write", action="store_true",
                        help="set the values but do not 'conf write' (lost on reboot)")
    args = parser.parse_args()

    pairs = []
    for tok in args.settings:
        if "=" not in tok:
            parser.error(f"expected KEY=VALUE, got '{tok}'")
        k, v = tok.split("=", 1)
        pairs.append((k.strip(), v.strip()))

    controller = moteus.Controller(id=args.target)
    stream = moteus.Stream(controller)

    await stream.write_message(b"tel stop")
    await stream.flush_read()

    for k, v in pairs:
        await stream.command(f"conf set {k} {v}".encode())
        print(f"set  {k} = {v}")

    if not args.no_write:
        await stream.command(b"conf write")
        print("conf write")

    print("--- readback ---")
    for k, _ in pairs:
        # conf get replies with just the value and no "OK" terminator, so we
        # must allow_any_response=True or Stream.command waits forever.
        r = await stream.command(f"conf get {k}".encode(), allow_any_response=True)
        text = r.decode().strip() if isinstance(r, (bytes, bytearray)) else str(r).strip()
        print(f"{k} = {text}")


if __name__ == "__main__":
    asyncio.run(main())
