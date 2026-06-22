#!/usr/bin/env python3
"""Re-fit a friction-map CSV with a Coulomb+viscous model over a low-speed window.

The drum's aerodynamic drag makes the torque grow super-linearly at high speed, so
a Coulomb+viscous (linear) model is only valid at low-to-moderate speed.  This tool
fits T_f = T_C*sign(w) + b*w to the points with |w| <= --fit-max-vel, plots ALL
points with the fit extended across the full range, and reports how far the data
departs from the linear model at top speed (the air-drag contribution).

No moteus connection -- reads the CSV written by identify_friction.py.

    uv run python analysis/analyze_friction.py <friction_map.csv> --fit-max-vel 7
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", help="friction_map.csv from identify_friction.py")
    p.add_argument("--fit-min-vel", type=float, default=1.0,
                   help="ignore |omega| < this in the fit (rev/s); excludes the "
                        "low-speed points the velocity loop can't hold; default 1")
    p.add_argument("--fit-max-vel", type=float, default=12.0,
                   help="ignore |omega| > this in the fit (rev/s); excludes the "
                        "air-drag region; default 12")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    rows = list(csv.DictReader(open(args.csv, newline="")))
    w = np.array([float(r["omega"]) for r in rows])
    T = np.array([float(r["torque"]) for r in rows])

    aw = np.abs(w)
    mask = (aw >= args.fit_min_vel) & (aw <= args.fit_max_vel)
    if mask.sum() < 2:
        p.error("fewer than 2 points in the fit window")

    # Fit T = T_C*sign(w) + b*w over the low-speed window.
    A = np.column_stack([np.sign(w[mask]), w[mask]])
    (T_C, b), *_ = np.linalg.lstsq(A, T[mask], rcond=None)
    T_C, b = float(T_C), float(b)

    # Air-drag departure: measured minus linear model at the top speed.
    w_top = np.abs(w).max()
    i_top = int(np.argmax(np.abs(w)))
    T_lin_top = T_C * np.sign(w[i_top]) + b * w[i_top]
    departure = T[i_top] - T_lin_top

    print(f"Coulomb+viscous fit over |w| <= {args.fit_max_vel} rev/s "
          f"({mask.sum()} of {len(w)} points):")
    print(f"  T_C = {T_C:.5f} Nm")
    print(f"  b   = {b:.6f} Nm/(rev/s)")
    print(f"\n  at top speed |w|={w_top:.1f} rev/s:")
    print(f"    measured torque   = {T[i_top]:.5f} Nm")
    print(f"    linear model      = {T_lin_top:.5f} Nm")
    print(f"    air-drag departure= {departure:+.5f} Nm "
          f"({100*departure/T_lin_top:+.0f}% over linear)")

    ws = np.linspace(w.min(), w.max(), 400)
    fit = T_C * np.sign(ws) + b * ws
    low = aw < args.fit_min_vel
    high = aw > args.fit_max_vel
    plt.figure(figsize=(7, 5))
    plt.scatter(w[mask], T[mask], c="tab:blue", zorder=3, label="fit window")
    plt.scatter(w[low], T[low], c="tab:orange", zorder=3,
                label="low speed (loop can't hold)")
    plt.scatter(w[high], T[high], c="tab:red", zorder=3,
                label="high speed (air drag)")
    plt.plot(ws, fit, "k--", label=f"C+V fit (T_C={T_C:.4f}, b={b:.5f})")
    plt.axvline(args.fit_max_vel, color="grey", ls=":", lw=0.8)
    plt.axvline(-args.fit_max_vel, color="grey", ls=":", lw=0.8)
    plt.axhline(0, c="k", lw=0.4)
    plt.axvline(0, c="k", lw=0.4)
    plt.xlabel("speed  [rev/s]")
    plt.ylabel("steady-state torque  [Nm]")
    plt.title("Friction: Coulomb+viscous fit and air-drag departure")
    plt.legend(fontsize=8)
    plt.grid(True)
    plt.tight_layout()
    out = Path(args.csv).with_name("friction_cv_fit.png")
    plt.savefig(out, dpi=150)
    print(f"\nPlot saved -> {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
