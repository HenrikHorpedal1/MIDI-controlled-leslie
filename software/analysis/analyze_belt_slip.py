#!/usr/bin/env python3
"""Re-analyze a slip_measurements CSV with optional sign flip on ω_L."""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress


def main():
    parser = argparse.ArgumentParser(description="Analyze belt slip CSV.")
    parser.add_argument("csv", help="Input CSV file (slip_measurements_*.csv)")
    parser.add_argument("--i0", type=float, default=None,
                        help="Ideal transmission ratio (default: min observed)")
    parser.add_argument("--flip-encoder", action="store_true",
                        help="Negate ω_L (fix sign error during recording)")
    args = parser.parse_args()

    with open(args.csv, newline="") as f:
        rows = list(csv.DictReader(f))

    om = np.array([float(r["omega_m"]) for r in rows])
    oL = np.array([float(r["omega_L"]) for r in rows])
    q  = np.array([float(r["q_A"])    for r in rows])

    if args.flip_encoder:
        oL = -oL
        print("Note: ω_L sign flipped.")

    # Drop rows where load isn't moving
    mask = np.abs(oL) > 1e-4
    om, oL, q = om[mask], oL[mask], q[mask]

    ri = om / oL

    i0 = args.i0 if args.i0 is not None else float(np.min(ri))
    print(f"i0 = {i0:.5f}  ({'provided' if args.i0 else 'min observed'})")

    slip = 1.0 - i0 / ri

    slope_forced = float(np.dot(om, oL) / np.dot(om, om))
    residuals = oL - slope_forced * om
    r2_forced = 1.0 - np.sum(residuals**2) / np.sum((oL - np.mean(oL))**2)

    slope_free, intercept, r_val, _, _ = linregress(om, oL)
    slope_free = float(slope_free)
    intercept  = float(intercept)
    r2_free    = float(r_val ** 2)

    print(f"\n{'Cmd':>6} {'ω_m':>10} {'ω_L':>10} {'i=ω_m/ω_L':>12} {'q_A':>8}")
    print("-" * 52)
    for vm, vl, ri_, q_ in zip(om, oL, ri, q):
        print(f"{vm:6.2f} {vm:10.4f} {vl:10.4f} {ri_:12.5f} {q_:8.4f}")

    print(f"\n--- Linearity (ω_L vs ω_m) ---")
    print(f"  Forced-through-origin slope  = {slope_forced:.5f}   R² = {r2_forced:.6f}")
    print(f"  Free linear fit slope        = {slope_free:.5f},  intercept = {intercept:.5f},  R² = {r2_free:.6f}")
    print(f"  Implied i from forced slope  = {1/slope_forced:.5f}")

    print(f"\n--- Slip s = 1 - i0/i ---")
    print(f"  mean  = {np.mean(slip)*100:.3f}%")
    print(f"  std   = {np.std(slip)*100:.3f}%")
    print(f"  range = [{np.min(slip)*100:.3f}%, {np.max(slip)*100:.3f}%]")

    sl_om = sl_q = r2_som = r2_sq = 0.0
    if len(om) >= 3:
        sl_om, _, r_om, _, _ = linregress(om, slip)
        sl_om  = float(sl_om)
        r2_som = float(r_om ** 2)
        sl_q, _, r_q, _, _ = linregress(q, slip)
        sl_q   = float(sl_q)
        r2_sq  = float(r_q ** 2)
        print(f"\n  s vs ω_m  :  slope={sl_om*100:.4f}%/(rev/s),  R²={r2_som:.4f}")
        print(f"  s vs q_A  :  slope={sl_q*100:.4f}%/A,          R²={r2_sq:.4f}")

    _, axes = plt.subplots(1, 3, figsize=(14, 4))

    ax = axes[0]
    ax.scatter(om, oL, zorder=3, label="measured")
    x = np.array([0, om.max() * 1.05])
    ax.plot(x, slope_forced * x, "--", label=f"forced-origin  R²={r2_forced:.4f}")
    ax.plot(x, slope_free * x + intercept, ":", label=f"free fit  R²={r2_free:.4f}")
    ax.set_xlabel("Motor ω_m  [rev/s]")
    ax.set_ylabel("Load ω_L  [rev/s]")
    ax.set_title("Linearity check")
    ax.legend(fontsize=8)
    ax.grid(True)

    ax = axes[1]
    ax.scatter(om, slip * 100, zorder=3)
    ax.axhline(np.mean(slip) * 100, ls="--", c="r",
               label=f"mean s = {np.mean(slip)*100:.3f}%")
    if len(om) >= 3 and r2_som > 0:
        ax.plot(om, (sl_om * om + (np.mean(slip) - sl_om * np.mean(om))) * 100,
                ":", c="grey", label=f"linear fit  R²={r2_som:.3f}")
    ax.set_xlabel("Motor ω_m  [rev/s]")
    ax.set_ylabel("Slip  s  [%]")
    ax.set_title("Slip vs speed")
    ax.legend(fontsize=8)
    ax.grid(True)

    ax = axes[2]
    ax.scatter(q, slip * 100, zorder=3)
    ax.set_xlabel("q-current  [A]  (∝ torque)")
    ax.set_ylabel("Slip  s  [%]")
    ax.set_title("Slip vs torque")
    ax.grid(True)

    plt.tight_layout()
    out = Path(args.csv).stem + "_reanalyzed.png"
    plt.savefig(out, dpi=150)
    print(f"\nPlot saved → {out}")
    plt.show()


if __name__ == "__main__":
    main()
