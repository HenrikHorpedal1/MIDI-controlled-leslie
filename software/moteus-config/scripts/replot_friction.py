#!/usr/bin/env python3
"""Replot an existing friction_map.csv without re-running hardware.

    uv run python moteus-config/scripts/replot_friction.py \
        analysis/data/friction/drum_20260618_183412
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np


def friction_model(w, c, b, a):
    return np.sign(w) * (c + a * w ** 2) + b * w


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dir", help="path to the artifact folder")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    run_dir = Path(args.run_dir)
    csv_path = run_dir / "friction_map.csv"
    if not csv_path.exists():
        sys.exit(f"not found: {csv_path}")

    with open(csv_path) as f:
        rows = list(csv.DictReader(f))

    w = np.array([float(r["omega"]) for r in rows])
    T = np.array([float(r["torque"]) for r in rows])
    tq_std = np.array([float(r["torque_std"]) for r in rows])
    w_std = np.array([float(r["omega_std"]) for r in rows])

    # OLS fit: T = sign(w)*(c + a*w^2) + b*w
    A = np.column_stack([np.sign(w), w, np.sign(w) * w ** 2])
    coef, *_ = np.linalg.lstsq(A, T, rcond=None)
    c, b, a = float(coef[0]), float(coef[1]), float(coef[2])
    print(f"c={c:.5f} Nm  b={b:.6f} Nm/(rev/s)  a={a:.6f} Nm/(rev/s)^2")

    ws = np.linspace(w.min(), w.max(), 400)
    fig, ax = plt.subplots(figsize=(7, 5))
    for xi, yi, dx, dy in zip(w, T, w_std, tq_std):
        ax.add_patch(mpatches.Rectangle(
            (xi - dx, yi - dy), 2 * dx, 2 * dy,
            linewidth=0, facecolor="steelblue", alpha=0.25))
    ax.scatter(w, T, s=18, color="steelblue", zorder=3,
               label="measured  (box = ±1σ speed / torque)")
    fit_label = (f"fit:  $\\mathrm{{sgn}}(\\omega)\\,({c*1e3:.2f}"
                 f" + {a*1e6:.2f}\\times10^{{-6}}\\,\\omega^2)$\n"
                 f"$\\quad+ {b*1e3:.3f}\\times10^{{-3}}\\,\\omega$  [Nm]")
    ax.plot(ws, friction_model(ws, c, b, a), "r--", label=fit_label)
    ax.axhline(0, c="k", lw=0.5)
    ax.axvline(0, c="k", lw=0.5)
    ax.set_xlabel("speed  [rev/s]")
    ax.set_ylabel("holding torque  [Nm]")
    motor = run_dir.name.split("_")[0]
    ax.set_title(f"Friction map ({motor})")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()

    out = run_dir / "friction_map.svg"
    fig.savefig(str(out))
    print(f"saved: {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
