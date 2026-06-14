#!/usr/bin/env python3
"""Zero the MA600 (sources[1]) reading at the current physical position.

The moteus diagnostic ``.raw`` field is NOT the value that feeds the source
formula, so ``offset = -raw`` does not work. Instead this shifts the EXISTING
offset by exactly the amount needed to drive the reported position to 0.

moteus reports the source's filtered position (register ENCODER_1_POSITION,
0x052) as::

    pos = (raw + offset) * sign / cpr      (mod 1)

To move the current reading ``p`` to 0, change the offset by ``-p*cpr*sign``::

    new_offset = old_offset - round(p * cpr * sign)

This is independent of what ``.raw`` means and of any rotor_to_output_ratio.
Touches only sources[1].offset — not the onboard encoder or output counter.
Changes are RAM-only until saved (pass --save or run 'conf write').
"""

import asyncio
import argparse

import moteus
import moteus.multiplex as mp_res


async def read_stable_position(controller, samples=6, delay=0.05):
    """Sample ENCODER_1_POSITION repeatedly and return (mean, spread).

    A free shaft drifts; zeroing while it moves saves an offset for a position
    the shaft has already left. The caller uses ``spread`` to warn the user.
    """
    readings = []
    for _ in range(samples):
        result = await controller.query()
        assert result is not None, "No response from controller"
        readings.append(result.values[moteus.Register.ENCODER_1_POSITION])
        await asyncio.sleep(delay)
    spread = max(readings) - min(readings)
    return readings[-1], spread


async def conf_get_int(stream, name):
    # "conf get" can be preceded by a stale "OK" ack in the read buffer;
    # retry, skipping any non-numeric line.
    for _ in range(5):
        result = await stream.command(
            f"conf get {name}".encode(), allow_any_response=True
        )
        text = result.decode().strip()
        try:
            return int(float(text))
        except ValueError:
            continue
    raise RuntimeError(f"Could not read {name} (last response: {text!r})")


async def main():
    parser = argparse.ArgumentParser(
        description="Zero the MA600 (sources[1]) reading at the current position."
    )
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1, help="CAN device ID (default: 1)")
    parser.add_argument("--save", action="store_true", help="Write config to flash after zeroing")
    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    qr = moteus.QueryResolution()
    qr._extra = {moteus.Register.ENCODER_1_POSITION: mp_res.F32}
    controller = moteus.Controller(id=args.target, transport=transport, query_resolution=qr)
    stream = moteus.Stream(controller)

    print(f"Connecting to moteus controller id={args.target}...")
    await stream.flush_read()

    old_offset = await conf_get_int(stream, "motor_position.sources.1.offset")
    cpr = await conf_get_int(stream, "motor_position.sources.1.cpr")
    sign = await conf_get_int(stream, "motor_position.sources.1.sign")
    print(f"  Current offset={old_offset}  cpr={cpr}  sign={sign}")

    print("Hold the shaft FIRMLY on your true zero mark and keep holding it.")
    print("Do NOT release it -- the MA600 is absolute, so torque does not bias")
    print("the reading, but a free shaft settles ~degrees away and that wrong")
    print("rest position is what gets locked in as zero.")
    input("Press Enter while holding at the zero mark... ")

    DRIFT_TOL = 0.0005  # ~33 counts at cpr=65536; held this still is fine
    pos_before, spread_before = await read_stable_position(controller)
    if spread_before > DRIFT_TOL:
        print(
            f"  WARNING: shaft is still moving (spread={spread_before:.6f} rev "
            f"over the sampling window). Hold it steady at the zero point.\n"
            f"           The offset below will be wrong by however far it drifts."
        )

    # Shift the existing offset so the reported position lands on 0.
    new_offset = old_offset - round(pos_before * cpr * sign)
    await stream.command(f"conf set motor_position.sources.1.offset {new_offset}".encode())

    await asyncio.sleep(0.2)

    pos_after, _ = await read_stable_position(controller)
    # Where the math says we should land (drift-free), wrapped into [0, 1).
    predicted = (pos_before + (new_offset - old_offset) * sign / cpr) % 1.0

    print(f"  MA600 position before     = {pos_before:.6f}")
    print(f"  Offset {old_offset} -> {new_offset}")
    print(f"  MA600 position after      = {pos_after:.6f}  (should be ~0)")

    # Distance from 0 on the circle (so 0.9999 counts as ~0, not ~1).
    resid = min(pos_after % 1.0, 1.0 - (pos_after % 1.0))
    if resid > 0.01:
        drift = min((pos_after - predicted) % 1.0, 1.0 - (pos_after - predicted) % 1.0)
        if drift > 0.01:
            print(
                f"  WARNING: shaft drifted by {drift:.6f} rev between read and verify "
                f"(predicted {predicted:.6f}). The offset is only as good as how "
                f"still the shaft was held — re-run while holding it steady."
            )
        else:
            print("  WARNING: MA600 position is not near 0. Check sign or cpr.")

    if args.save:
        await stream.command(b"conf write")
        print("  Config saved to flash.")
    else:
        print("  Config is RAM-only. Run with --save or 'conf write' to persist.")

    await controller.set_stop()


if __name__ == "__main__":
    asyncio.run(main())
