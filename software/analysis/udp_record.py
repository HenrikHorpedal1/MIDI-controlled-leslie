#!/usr/bin/env python3
"""Record the ESP32 UDP telemetry stream straight to a PlotJuggler-style CSV.

The firmware sends batched JSON packets to UDP port 9870, e.g.
    {"ts":244.859,"horn/posctrl/pos/traj":1.23,"horn/posctrl/pos/meas":1.21}
Several packets may share the same "ts" (one control tick batched across
packets), so rows are grouped by ts and the columns are the union of all keys
seen. Output matches PlotJuggler's export: first column "__time", one column per
signal, empty cells where a signal had no sample — so plot_position_controller.py
reads it unchanged.

Usage:
    cd software
    # record until Ctrl-C:
    uv run python analysis/udp_record.py analysis/data/controller_test/plotjuggler/park.csv
    # record a fixed window:
    uv run python analysis/udp_record.py park.csv --duration 8
    # only keep certain signals (substring match), and watch live rpm:
    uv run python analysis/udp_record.py park.csv --filter posctrl --watch drum/speed/rpm

Tip: the firmware must be told this machine's IP (Telemetry.begin("<ip>", 9870)).
Find yours with `ipconfig getifaddr en0`.
"""

import argparse
import csv
import json
import signal
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", help="output CSV path")
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--bind", default="0.0.0.0", help="local interface to listen on")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after N seconds (default: run until Ctrl-C)")
    ap.add_argument("--filter", default=None,
                    help="keep only keys containing this substring (besides ts)")
    ap.add_argument("--watch", default=None,
                    help="print this key's latest value live so you can see motion")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)

    rows = {}     # ts -> {key: value}
    keys = []     # ordered union of keys seen (preserves first-seen order)
    keyset = set()
    stop = {"flag": False}

    def on_sigint(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_sigint)

    print(f"listening on {args.bind}:{args.port} … (Ctrl-C to stop and save)",
          file=sys.stderr)
    t0 = time.time()
    n_pkts = 0
    last_report = 0.0

    while not stop["flag"]:
        if args.duration is not None and time.time() - t0 >= args.duration:
            break
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        try:
            obj = json.loads(data.decode("utf-8", "replace"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue
        ts = obj.get("ts")
        if ts is None:
            continue
        n_pkts += 1
        row = rows.setdefault(ts, {})
        for k, v in obj.items():
            if k == "ts":
                continue
            if args.filter and args.filter not in k:
                continue
            if k not in keyset:
                keyset.add(k)
                keys.append(k)
            row[k] = v

        now = time.time()
        if now - last_report >= 0.5:
            last_report = now
            watch = ""
            if args.watch and args.watch in obj:
                watch = f"  {args.watch}={obj[args.watch]:.3f}"
            print(f"\r{len(rows)} samples, {n_pkts} packets{watch}   ",
                  end="", file=sys.stderr, flush=True)

    sock.close()
    print(file=sys.stderr)

    if not rows:
        print("no packets received — check the firmware host IP / Wi-Fi", file=sys.stderr)
        sys.exit(1)

    header = ["__time"] + keys
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for ts in sorted(rows):
            row = rows[ts]
            w.writerow([ts] + [row.get(k, "") for k in keys])

    span = max(rows) - min(rows)
    print(f"wrote {args.out}: {len(rows)} rows, {len(keys)} signals, "
          f"{span:.1f}s span", file=sys.stderr)


if __name__ == "__main__":
    main()
