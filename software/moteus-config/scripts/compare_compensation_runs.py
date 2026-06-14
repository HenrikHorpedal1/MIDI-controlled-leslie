#!/usr/bin/env python3
"""Compare two --write-unbiased dumps from compensate_encoder_velocity.py.

Each input file has two whitespace-separated columns:
    <encoder position [0,1)>   <velocity deviation (fraction of mean)>

The point is to tell whether the sharp dips in the velocity-deviation curve are
*deterministic* (repeatable angle error -> correctable by a finer table) or
*low-SNR noise* (field-cancellation dead zone -> only fixable by shielding):

  - same shape both runs (high correlation, dips at same angle/depth)  => deterministic
  - dips wander / change depth (low correlation in the dip region)      => noise

Usage:
    uv run software/moteus-config/scripts/compare_compensation_runs.py \
        run1.txt run2.txt [--plot]
"""

import argparse
import numpy as np


def load(path):
    data = np.loadtxt(path)
    x = data[:, 0]
    y = data[:, 1]
    order = np.argsort(x)
    return x[order], y[order]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run1")
    ap.add_argument("run2")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--dip-threshold", type=float, default=-0.20,
                    help="deviation below this (fraction) counts as a dip")
    args = ap.parse_args()

    x1, y1 = load(args.run1)
    x2, y2 = load(args.run2)

    # Resample run2 onto run1's angular grid so we compare like-for-like
    # (periodic, since position wraps at 1.0).
    y2i = np.interp(x1, x2, y2, period=1.0)

    diff = y1 - y2i
    overall_corr = np.corrcoef(y1, y2i)[0, 1]
    overall_rms = np.sqrt(np.mean(diff ** 2))

    # Dip region = where EITHER run drops well below zero.
    dip_mask = (y1 < args.dip_threshold) | (y2i < args.dip_threshold)
    print(f"samples: {len(x1)}   grid spacing: {1.0/len(x1)*100:.3f}% / rev")
    print(f"overall  correlation: {overall_corr:.3f}")
    print(f"overall  RMS difference: {overall_rms*100:.2f}%")

    if dip_mask.any():
        dx = x1[dip_mask]
        dcorr = np.corrcoef(y1[dip_mask], y2i[dip_mask])[0, 1]
        drms = np.sqrt(np.mean(diff[dip_mask] ** 2))
        print(f"\ndip region ({dip_mask.sum()} pts, "
              f"angles {dx.min():.3f}..{dx.max():.3f}):")
        print(f"  run1 min: {y1[dip_mask].min()*100:.1f}%   "
              f"run2 min: {y2i[dip_mask].min()*100:.1f}%")
        print(f"  dip-region correlation: {dcorr:.3f}")
        print(f"  dip-region RMS difference: {drms*100:.2f}%")
        verdict = ("DETERMINISTIC -> a finer table should help"
                   if dcorr > 0.8 and drms < 0.15
                   else "NOISY / dead-zone -> resolution will not fix it; shield instead")
        print(f"\n  verdict: {verdict}")
    else:
        print("\nno dips below threshold found in either run.")

    if args.plot:
        import matplotlib.pyplot as plt
        _, ax = plt.subplots()
        ax.plot(x1, y1, label="run1")
        ax.plot(x1, y2i, label="run2 (resampled)")
        ax.plot(x1, diff, label="difference", color="red", alpha=0.5)
        ax.set_xlabel("encoder position")
        ax.set_ylabel("velocity deviation")
        ax.legend()
        plt.show()


if __name__ == "__main__":
    main()
