#!/usr/bin/env python3
"""Diff a LIVE moteus config against a backup .cfg file.

    uv run python moteus-config/scripts/compare_to_backup.py \\
        --target 1 --backup moteus-config/backups/drum-id1-20260616-134425.cfg
"""

import argparse
import asyncio

import moteus


def load_cfg_file(path):
    cfg = {}
    with open(path, encoding="latin1") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            cfg[key] = value.strip()
    return cfg


async def dump_live(transport, target):
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


def differ(a, b):
    if a == b:
        return False
    try:
        fa, fb = float(a), float(b)
    except ValueError:
        return True
    scale = max(abs(fa), abs(fb), 1e-9)
    return abs(fa - fb) / scale > 1e-6


async def main():
    p = argparse.ArgumentParser(description=__doc__)
    moteus.make_transport_args(p)
    p.add_argument("--target", "-t", type=int, default=1)
    p.add_argument("--backup", required=True, help="path to backup .cfg")
    p.add_argument("--all", action="store_true",
                   help="show every difference (default hides calibration noise)")
    args = p.parse_args()

    transport = moteus.get_singleton_transport(args)
    live = await dump_live(transport, args.target)
    backup = load_cfg_file(args.backup)

    CALIB = (".offset", "compensation", "cogging", "resistance", "inductance",
             "cal_", "encoder_offset")

    keys = sorted(set(live) | set(backup))
    diffs, calib, only_live, only_backup = [], [], [], []
    for k in keys:
        if k not in live:
            only_backup.append(k); continue
        if k not in backup:
            only_live.append(k); continue
        if differ(live[k], backup[k]):
            (calib if any(s in k for s in CALIB) else diffs).append(k)

    print(f"LIVE id={args.target}: {len(live)} keys   BACKUP: {len(backup)} keys "
          f"({args.backup})\n")

    print(f"=== DIFFERENCES (non-calibration) ({len(diffs)}) ===")
    for k in diffs:
        print(f"  {k}")
        print(f"      live  : {live[k]}")
        print(f"      backup: {backup[k]}")

    if args.all and calib:
        print(f"\n=== CALIBRATION DIFFERENCES ({len(calib)}) ===")
        for k in calib:
            print(f"  {k}: live={live[k]}  backup={backup[k]}")
    elif calib:
        print(f"\n({len(calib)} calibration-key differences hidden; --all to show)")

    if only_live:
        print(f"\n=== only in LIVE ({len(only_live)}) ===\n  " + ", ".join(only_live))
    if only_backup:
        print(f"\n=== only in BACKUP ({len(only_backup)}) ===\n  " + ", ".join(only_backup))


if __name__ == "__main__":
    asyncio.run(main())
