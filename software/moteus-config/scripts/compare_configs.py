#!/usr/bin/env python3
"""Dump two moteus configs (`conf enumerate`) and diff them.

Mirrors `moteus_tool --dump-config`, which is just
`stream.command("conf enumerate")` read until OK (see moteus_tool.py). We grab
both controllers, parse the "key value" lines, and report differences.

Numeric values are compared with a small relative tolerance so float print
noise (0.531000018 vs 0.531000019) is not flagged. Differences are split into
TUNING keys (a real difference here is a likely bug) and OTHER keys (per-motor
calibration — offsets, resistance, commutation tables — expected to differ).

Usage (from software/, uses the uv venv):
    uv run python moteus-config/scripts/compare_configs.py
    uv run python moteus-config/scripts/compare_configs.py --a 1 --b 2 \
        --a-name drum --b-name horn --save-dir /tmp
"""

import argparse
import asyncio
import os

import moteus


# Prefixes whose differences matter for tracking/tuning. Everything else is
# treated as per-motor calibration noise and shown separately.
TUNING_PREFIXES = (
    "servo.pid_position",
    "servo.pid_dq",
    "servo.max_current_A",
    "servo.max_position_slip",
    "servo.max_velocity",
    "servo.default_velocity_limit",
    "servo.default_accel_limit",
    "servo.feedforward",
    "servo.bemf_feedforward",
    "servo.flux_brake",
    "servo.voltage_mode_control",
    "servo.fixed_voltage_mode",
    "motor.poles",
    "motor.Kv",
    "motor_position.rotor_to_output_ratio",
    "motor_position.commutation_source",
    "motor_position.output",
    "motor_position.sources",  # type/sign/cpr/reference (offset filtered below)
)

# Even within a tuning prefix, these are per-motor calibration — keep as "other".
CALIB_SUBSTR = (
    ".offset",
    "compensation",
    "cogging",
    "resistance",
    "inductance",
)


async def dump_config(transport, target):
    controller = moteus.Controller(id=target, transport=transport)
    stream = moteus.Stream(controller)
    await stream.flush_read()
    text = (await stream.command(b"conf enumerate")).decode("latin1")
    cfg = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        key, _, value = line.partition(" ")
        cfg[key] = value.strip()
    return cfg


def values_differ(a, b):
    if a == b:
        return False
    try:
        fa, fb = float(a), float(b)
    except ValueError:
        return True
    scale = max(abs(fa), abs(fb), 1e-9)
    return abs(fa - fb) / scale > 1e-6


def is_tuning(key):
    if any(sub in key for sub in CALIB_SUBSTR):
        return False
    return key.startswith(TUNING_PREFIXES)


def report(cfg_a, cfg_b, name_a, name_b):
    keys = sorted(set(cfg_a) | set(cfg_b))
    tuning, other, only_a, only_b = [], [], [], []
    for k in keys:
        if k not in cfg_a:
            only_b.append(k)
            continue
        if k not in cfg_b:
            only_a.append(k)
            continue
        if values_differ(cfg_a[k], cfg_b[k]):
            (tuning if is_tuning(k) else other).append(k)

    def show(title, ks):
        print(f"\n=== {title} ({len(ks)}) ===")
        for k in ks:
            print(f"  {k}")
            print(f"      {name_a}: {cfg_a.get(k, '<missing>')}")
            print(f"      {name_b}: {cfg_b.get(k, '<missing>')}")

    print(f"Compared {len(cfg_a)} keys ({name_a}) vs {len(cfg_b)} keys ({name_b}).")
    show("TUNING DIFFERENCES (investigate — likely the cause)", tuning)
    show("OTHER DIFFERENCES (expected per-motor calibration)", other)
    if only_a:
        print(f"\n=== Keys only in {name_a} ({len(only_a)}) ===")
        print("  " + ", ".join(only_a))
    if only_b:
        print(f"\n=== Keys only in {name_b} ({len(only_b)}) ===")
        print("  " + ", ".join(only_b))
    if not tuning:
        print("\n>>> No tuning differences: the steady error is NOT a config "
              "difference (points to mechanical load / friction on the horn).")


async def main():
    parser = argparse.ArgumentParser(description="Diff two moteus configs.")
    moteus.make_transport_args(parser)
    parser.add_argument("--a", type=int, default=1, help="first CAN ID (default 1)")
    parser.add_argument("--b", type=int, default=2, help="second CAN ID (default 2)")
    parser.add_argument("--a-name", default="drum_id1")
    parser.add_argument("--b-name", default="horn_id2")
    parser.add_argument("--save-dir", default=None,
                        help="also write each full config to <save-dir>/<name>.cfg")
    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    print(f"Dumping config from id={args.a} ({args.a_name})...")
    cfg_a = await dump_config(transport, args.a)
    print(f"Dumping config from id={args.b} ({args.b_name})...")
    cfg_b = await dump_config(transport, args.b)

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        for name, cfg in ((args.a_name, cfg_a), (args.b_name, cfg_b)):
            path = os.path.join(args.save_dir, f"{name}.cfg")
            with open(path, "w") as f:
                for k in sorted(cfg):
                    f.write(f"{k} {cfg[k]}\n")
            print(f"  wrote {path}")

    report(cfg_a, cfg_b, args.a_name, args.b_name)


if __name__ == "__main__":
    asyncio.run(main())
