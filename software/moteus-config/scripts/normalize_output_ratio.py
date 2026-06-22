#!/usr/bin/env python3
"""Rescale motor_position.rotor_to_output_ratio and adjust PID gains proportionally.

When rotor_to_output_ratio changes from old_R to new_R, reported output
positions scale by new_R/old_R.  Everything in pid_position is output-referenced,
so a mechanical impedance reflects through the ratio as ratio^2: the position
gains scale by (old_R/new_R)^2, while the torque-domain integral limits scale
linearly by (old_R/new_R).  To keep the same physical response:

    new_kp = old_kp * (old_R / new_R)**2
    new_ki = old_ki * (old_R / new_R)**2
    new_kd = old_kd * (old_R / new_R)**2
    new_ilimit     = old_ilimit     * (old_R / new_R)
    new_iratelimit = old_iratelimit * (old_R / new_R)

servo.inertia_feedforward is torque per output acceleration; output acceleration
reflects through the ratio exactly like output position (a_out = R * a_rotor), so
it follows the same ratio^2 law as the position gains:

    new_inertia_feedforward = old_inertia_feedforward * (old_R / new_R)**2

Changes are written to RAM only — no 'conf write' is issued.

Usage (from software/):
    uv run python moteus-config/scripts/normalize_output_ratio.py --new-ratio 1.0
    uv run python moteus-config/scripts/normalize_output_ratio.py --new-ratio 0.5 -t 2
    uv run python moteus-config/scripts/normalize_output_ratio.py --new-ratio 1.0 --dry-run
"""

import asyncio
import argparse

import moteus  # type: ignore[import-untyped]


PID_GAIN_KEYS = [
    "servo.pid_position.kp",
    "servo.pid_position.ki",
    "servo.pid_position.kd",
    # torque per output acceleration -> same ratio^2 reflection as the gains
    "servo.inertia_feedforward",
]
PID_LIMIT_KEYS = [
    "servo.pid_position.ilimit",
    "servo.pid_position.iratelimit",
]
RATIO_KEY = "motor_position.rotor_to_output_ratio"


async def conf_get_float(stream, name: str) -> float:
    text = ""
    for _ in range(5):
        result = await stream.command(
            f"conf get {name}".encode(), allow_any_response=True
        )
        text = result.decode().strip()
        try:
            return float(text)
        except ValueError:
            continue
    raise RuntimeError(f"Could not read {name!r} (last response: {text!r})")


async def conf_set(stream, name: str, value: float) -> None:
    await stream.command(f"conf set {name} {value:.9g}".encode())


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Change rotor_to_output_ratio in RAM and rescale PID gains to match."
    )
    moteus.make_transport_args(parser)
    parser.add_argument(
        "--target", "-t", type=int, default=1, help="CAN device ID (default: 1)"
    )
    parser.add_argument(
        "--new-ratio", "-r", type=float, required=True,
        help="Target rotor_to_output_ratio (must be > 0 and <= 1.0)"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print what would be set without sending any conf set commands."
    )
    args = parser.parse_args()

    if not (0 < args.new_ratio <= 1.0):
        parser.error(f"--new-ratio must be in (0, 1.0], got {args.new_ratio}")

    transport = moteus.get_singleton_transport(args)
    controller = moteus.Controller(id=args.target, transport=transport)
    stream = moteus.Stream(controller)

    print(f"Connecting to moteus controller id={args.target}...")
    await stream.flush_read()

    old_R = await conf_get_float(stream, RATIO_KEY)
    new_R = args.new_ratio
    print(f"\n  {RATIO_KEY}: {old_R}  →  {new_R}")

    if abs(old_R - new_R) < 1e-9:
        print("  Ratio is already at the target value — nothing to do.")
        return

    print("\n  Current PID gains:")
    old_gains: dict[str, float] = {}
    for key in PID_GAIN_KEYS + PID_LIMIT_KEYS:
        val = await conf_get_float(stream, key)
        old_gains[key] = val
        print(f"    {key} = {val}")

    scale = old_R / new_R  # < 1 means gains shrink, > 1 means gains grow

    new_values: dict[str, float] = {}
    for key in PID_GAIN_KEYS:
        # Gains are an output-referenced impedance -> reflect as ratio^2.
        new_values[key] = old_gains[key] * scale * scale
    for key in PID_LIMIT_KEYS:
        # Torque-domain integral limits reflect linearly with the ratio.
        new_values[key] = old_gains[key] * scale
    new_values[RATIO_KEY] = new_R

    print(f"\n  Scale factor: {old_R:.6g} / {new_R:.6g} = {scale:.6g}")
    print(f"  gains × {scale*scale:.6g},  limits × {scale:.6g}")
    print("\n  New values:")
    for key, val in new_values.items():
        print(f"    {key}:  {old_gains.get(key, old_R):.9g}  →  {val:.9g}")

    if args.dry_run:
        print("\n  [dry-run] No changes sent.")
        return

    confirm = input("\nApply these changes to RAM? [y/N] ").strip().lower()
    if confirm != "y":
        print("Aborted.")
        return

    for key, val in new_values.items():
        await conf_set(stream, key, val)
        print(f"  set {key} = {val:.9g}")

    print("\nDone. Changes are in RAM only — power cycle or 'conf write' to persist/discard.")
    await controller.set_stop()


if __name__ == "__main__":
    asyncio.run(main())
