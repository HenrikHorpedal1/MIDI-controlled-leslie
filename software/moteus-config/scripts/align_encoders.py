#!/usr/bin/env python3
"""Align MA600 zero position and empirically determine belt pulley ratio for moteus c1.

Assumes:
  - Onboard encoder (sources[0]) is used for commutation and output position control
  - MA600 on aux2 SPI is configured as sources[1] (reference=output) but is NOT
    the output source — it is read externally to call set_output_exact at startup

Steps:
  1. Verify MA600 configuration
  2. Align zeros: at physical zero, set sources[1].offset so MA600 reads 0 AND
     run d cfg-set-output 0 so the onboard encoder output also reads 0
  3. Measure rotor_to_output_ratio (move N rotor revolutions, track MA600)
  4. Optionally save config to flash
"""

import argparse
import asyncio

import moteus
import moteus.multiplex as mp_res


MA600_CPR = 65536


class EncoderTracker:
    """Tracks cumulative revolutions of a 0-1 fractional encoder, detecting wraps."""

    def __init__(self, init_frac):
        self._frac = init_frac
        self._full_revs = 0
        self._start = init_frac

    def update(self, new_frac):
        d = new_frac - self._frac
        if d > 0.5:
            self._full_revs -= 1
        elif d < -0.5:
            self._full_revs += 1
        self._frac = new_frac

    @property
    def delta(self):
        return self._full_revs + self._frac - self._start


async def conf_get_float(stream, name):
    result = await stream.command(f"conf get {name}".encode(), allow_any_response=True)
    return float(result.decode().strip())


async def conf_get_int(stream, name):
    result = await stream.command(f"conf get {name}".encode(), allow_any_response=True)
    return int(float(result.decode().strip()))


async def conf_set(stream, name, value):
    await stream.command(f"conf set {name} {value}".encode())


async def conf_get_safe(stream, name, cast=float, default=None):
    try:
        result = await asyncio.wait_for(
            stream.command(f"conf get {name}".encode(), allow_any_response=True),
            timeout=2.0
        )
        return cast(float(result.decode().strip()))
    except Exception:
        return default


async def step_check_config(stream):
    print("\n=== Step 1: Checking configuration ===")

    ratio = await conf_get_safe(stream, "motor_position.rotor_to_output_ratio")
    out_source = await conf_get_safe(stream, "motor_position.output.source", cast=int)

    if ratio is None:
        print("  WARNING: Could not read rotor_to_output_ratio (timeout)")
    else:
        print(f"  rotor_to_output_ratio    = {ratio}")

    if out_source is None:
        print("  WARNING: Could not read output.source (timeout)")
    else:
        print(f"  output.source            = {out_source}  (0=onboard)")

    for key in ["motor_position.sources.1.type",
                "motor_position.sources.1.aux_number",
                "motor_position.sources.1.sign",
                "motor_position.sources.1.offset"]:
        val = await conf_get_safe(stream, key)
        label = key.split(".")[-1]
        if val is not None:
            print(f"  sources[1].{label:<16} = {val}")


async def step_align_zeros(stream, controller):
    """Set MA600 offset and onboard output zero at the same physical position."""
    print("\n=== Step 2: Align zeros ===")
    print("Manually position the mechanism at your desired zero point.")
    print("Release the shaft so it is free (no motor torque).")
    input("Press Enter when at zero position... ")

    result_before = await controller.query()
    frac_before = result_before.values[moteus.Register.ENCODER_1_POSITION]

    await stream.flush_read()
    mp = await stream.read_data("motor_position")
    raw = mp.sources[1].raw

    # (raw + offset) * sign / CPR = filtered_value; want 0 → offset = -raw
    new_offset = -int(raw)
    await conf_set(stream, "motor_position.sources.1.offset", new_offset)

    # Also zero the onboard output position counter
    await stream.command(b"d cfg-set-output 0")
    await asyncio.sleep(0.2)

    result_after = await controller.query()
    frac_after = result_after.values[moteus.Register.ENCODER_1_POSITION]
    out_pos = result_after.values[moteus.Register.POSITION]

    print(f"  MA600 raw at zero         = {raw}")
    print(f"  New sources[1].offset     = {new_offset}")
    print(f"  MA600 position before     = {frac_before:.6f}")
    print(f"  MA600 position after      = {frac_after:.6f}  (should be ~0)")
    print(f"  Output position after     = {out_pos:.6f}  (should be ~0)")

    if abs(frac_after) > 0.01 and abs(frac_after - 1.0) > 0.01:
        print("  WARNING: MA600 position is not near 0. Check sign or CPR.")


async def step_measure_ratio(controller, stream, n_rotor_revs, velocity, known_ratio=None):
    print(f"\n=== Step 3: Measuring pulley ratio ({n_rotor_revs} rotor revolutions) ===")
    print(f"  Velocity limit: {velocity} rev/s")
    print("  Ensure the mechanism has free range of motion.")
    input("Press Enter to start motion... ")

    if known_ratio is not None:
        old_ratio = known_ratio
        print(f"  Using provided ratio={old_ratio}")
    else:
        old_ratio = await conf_get_safe(stream, "motor_position.rotor_to_output_ratio")
        if old_ratio is None:
            print("ERROR: Could not read rotor_to_output_ratio — pass --ratio explicitly.")
            return None

    # Command in output-space using the current ratio so PID gains stay correct.
    # The controller moves n_rotor_revs rotor turns regardless of what old_ratio is;
    # we just express the target in whatever units the output register uses.
    target_output = n_rotor_revs * old_ratio
    target_velocity = velocity * old_ratio  # output rev/s

    print(f"  Using existing ratio={old_ratio} — target output pos: {target_output:.4f} rev")

    try:
        await controller.set_output_exact(position=0.0)
        await asyncio.sleep(0.4)

        start_result = await controller.query()
        ma600_start = start_result.values[moteus.Register.ENCODER_1_POSITION]
        ma600_tracker = EncoderTracker(ma600_start)
        print(f"  MA600 start: {ma600_start:.6f}")
        print("  Moving...")

        timeout = n_rotor_revs / velocity + 10.0
        deadline = asyncio.get_event_loop().time() + timeout
        settled = 0

        while True:
            result = await controller.set_position(
                position=target_output,
                velocity_limit=target_velocity,
                accel_limit=target_velocity * 0.5,
                query=True,
            )
            ma600_tracker.update(result.values[moteus.Register.ENCODER_1_POSITION])

            pos = result.values[moteus.Register.POSITION]
            vel = result.values[moteus.Register.VELOCITY]

            if abs(pos - target_output) < 0.02 * old_ratio and abs(vel) < 0.02 * old_ratio:
                settled += 1
                if settled >= 3:
                    break
            else:
                settled = 0

            if asyncio.get_event_loop().time() > deadline:
                print("  WARNING: Timed out waiting for motion to complete.")
                break

            await asyncio.sleep(0.05)

        final_result = await controller.query()
        ma600_tracker.update(final_result.values[moteus.Register.ENCODER_1_POSITION])

        # ratio = output revolutions (MA600) per rotor revolution
        ratio = ma600_tracker.delta / n_rotor_revs

    finally:
        await controller.set_stop()

    print(f"\n  MA600 delta:              {ma600_tracker.delta:.6f} output revolutions")
    print(f"  Rotor moved:              {n_rotor_revs} revolutions")
    print(f"  Computed ratio:           {ratio:.6f}")

    if ratio < 0:
        print("  NOTE: negative ratio — belt reverses direction.")
        print("  Set motor_position.sources.1.sign = -1, then re-run.")

    return ratio


async def step_apply_ratio(stream, ratio):
    print(f"\n  Suggested: motor_position.rotor_to_output_ratio = {abs(ratio):.6f}")
    answer = input("Apply computed ratio now? [y/N]: ").strip().lower()
    if answer == "y":
        await conf_set(stream, "motor_position.rotor_to_output_ratio", abs(ratio))
        print(f"  Set rotor_to_output_ratio = {abs(ratio):.6f}")
    else:
        print(f"  Skipped. Apply manually: conf set motor_position.rotor_to_output_ratio {abs(ratio):.6f}")


async def main():
    parser = argparse.ArgumentParser(
        description="Align MA600 zero and measure pulley ratio for moteus c1."
    )
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1, help="CAN device ID")
    parser.add_argument(
        "--move-rotations", type=float, default=5.0,
        help="Rotor revolutions to move during ratio measurement (default: 5)",
    )
    parser.add_argument(
        "--velocity", type=float, default=0.5,
        help="Velocity limit in rotor rev/s during ratio measurement (default: 0.5)",
    )
    parser.add_argument(
        "--ratio", type=float, default=None,
        help="Known output ratio (e.g. 0.25) — skips conf get if provided",
    )
    parser.add_argument("--skip-check", action="store_true", help="Skip configuration check step")
    parser.add_argument("--skip-zero", action="store_true", help="Skip zero alignment step")
    parser.add_argument("--skip-ratio", action="store_true", help="Skip ratio measurement step")

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    qr = moteus.QueryResolution()
    qr._extra = {moteus.Register.ENCODER_1_POSITION: mp_res.F32}
    controller = moteus.Controller(id=args.target, transport=transport, query_resolution=qr)
    stream = moteus.Stream(controller)

    print(f"Connecting to moteus controller id={args.target}...")

    if not args.skip_check:
        await step_check_config(stream)
    else:
        print("\n=== Step 1: Skipped (--skip-check) ===")

    if not args.skip_zero:
        await step_align_zeros(stream, controller)
    else:
        print("\n=== Step 2: Skipped (--skip-zero) ===")

    ratio = None
    if not args.skip_ratio:
        ratio = await step_measure_ratio(
            controller, stream, args.move_rotations, args.velocity,
            known_ratio=args.ratio,
        )
        if ratio is not None:
            await step_apply_ratio(stream, ratio)
    else:
        print("\n=== Step 3: Skipped (--skip-ratio) ===")

    print("\nConfiguration NOT saved to flash — changes are RAM-only.")
    await controller.set_stop()

    if ratio is not None:
        print("\nAt application startup, initialize absolute position with:")
        print("  result = await controller.query()")
        print("  await controller.set_output_exact(position=result.values[moteus.Register.ENCODER_1_POSITION])")


if __name__ == "__main__":
    asyncio.run(main())
