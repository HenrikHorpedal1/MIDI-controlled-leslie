#!/usr/bin/env python3
"""
compare_poll.py - stacked residual plots of two jitter captures (e.g. 120 BPM
with 1 ms polling vs. busy-polling), trimmed to the SAME number of ticks and
sharing one y-axis so the two panels are directly comparable.

Usage:
    python compare_poll.py 120bpm.txt 120bpm-busypoll.txt
    python compare_poll.py 120bpm.txt 120bpm-busypoll.txt \
        --labels "1 ms poll" "busy-poll" --out jitter-poll-compare.png

Requires: numpy, matplotlib
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt


def load(path):
    k, t = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split(",")
            if len(p) < 2:
                continue
            try:
                k.append(int(p[0]))
                t.append(float(p[1]))
            except ValueError:
                continue
    return np.asarray(k, float), np.asarray(t, float)


def residuals(k, t):
    b, a = np.polyfit(k, t, 1)
    return t - (a + b * k)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs=2, help="two capture files")
    ap.add_argument("--labels", nargs=2, default=None)
    ap.add_argument("--ticks", type=int, default=None, help="show only the first N ticks")
    ap.add_argument("--out", default="jitter-poll-compare.png")
    ap.add_argument("--csv", default=None, help="also write residuals to this CSV (tick,poll,busy)")
    args = ap.parse_args()

    data = [load(f) for f in args.files]
    n_fit = min(len(k) for k, _ in data)       # equal window; fit the period over all of it
    labels = args.labels or args.files

    # Fit the constant-tempo grid over the whole window (so the startup transient
    # does not bias the slope), then zoom in for display.
    resids_full = [residuals(k[:n_fit], t[:n_fit]) for k, t in data]
    sigmas = [r.std(ddof=2) for r in resids_full]

    n = min(n_fit, args.ticks) if args.ticks else n_fit   # how many ticks to show
    resids = [r[:n] for r in resids_full]
    ymax = max(np.abs(r).max() for r in resids) * 1.05

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("tick,poll,busy\n")
            for i in range(n):
                f.write(f"{i},{resids[0][i]:.1f},{resids[1][i]:.1f}\n")
        print(f"wrote {args.csv}  (set the pgfplots y-range to about +/- {ymax:.0f})")

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax, r, lab, sig in zip(axes, resids, labels, sigmas):
        ax.plot(np.arange(n), r, lw=0.6)
        ax.axhline(0.0, color="k", lw=0.5)
        ax.set_ylim(-ymax, ymax)               # shared scale
        ax.set_ylabel("residual [us]")
        ax.set_title(f"{lab}   (sigma = {sig:.0f} us)")
    axes[-1].set_xlabel("tick index")

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"saved {args.out}  ({n} ticks each)")
    plt.show()


if __name__ == "__main__":
    main()
