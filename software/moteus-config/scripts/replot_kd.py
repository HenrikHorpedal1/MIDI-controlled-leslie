#!/usr/bin/env python3
"""Replot a kd_sweep artifact folder without re-running hardware.

    uv run python moteus-config/scripts/replot_kd.py \
        ../../analysis/data/kd_sweep/drum_20260618_214038 --no-show
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("artifact_dir", help="path to the kd_sweep artifact folder")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    d = Path(args.artifact_dir)

    with open(d / "meta.json") as f:
        meta = json.load(f)

    with open(d / "kd_sweep.csv") as f:
        summary = list(csv.DictReader(f))

    rows = []
    for r in summary:
        kd = float(r["kd"])
        trace_file = d / f"trace_kd_{kd:.4f}.csv"
        t_arr, v_arr = [], []
        if trace_file.exists():
            with open(trace_file) as f:
                for row in csv.DictReader(f):
                    t_arr.append(float(row["t"]))
                    v_arr.append(float(row["v"]))
        rows.append(dict(
            kd=kd,
            hf_rms=float(r["hf_rms"]) if r["hf_rms"] != "nan" else float("nan"),
            stable=r["stable"] == "True",
            t_trace=t_arr,
            v_trace=v_arr,
        ))

    hf_baseline = meta.get("hf_baseline", None)
    hf_thresh   = meta.get("hf_threshold", None)
    kd_max      = meta.get("kd_max_stable", float("nan"))
    target_vel  = meta.get("target_vel_rev_s", None)
    motor       = meta.get("motor", "drum")
    target_id   = meta.get("target", 1)
    hf_cut      = meta.get("hf_cut_hz", 30)

    n = len(rows)
    cmap = plt.cm.RdYlGn_r(np.linspace(0.0, 0.85, max(n, 1)))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    for r, color in zip(reversed(rows), reversed(cmap.tolist())):
        if not r["t_trace"]:
            continue
        t_tr = np.array(r["t_trace"])
        v_tr = np.array(r["v_trace"])
        lw = 1.0 if r["stable"] else 1.5
        ls = "-" if r["stable"] else "--"
        ax1.plot(t_tr, v_tr, color=color, lw=lw, ls=ls,
                 label=f"kd={r['kd']:.3f}" + ("" if r["stable"] else " ✗"))

    if target_vel:
        ax1.axhline(target_vel, c="k", ls=":", lw=0.8, label="target vel")
    ax1.set_ylabel("velocity  [rev/s]")
    ax1.set_xlabel("time  [s]")
    ax1.set_title(f"k_d sweep — velocity ramps ({motor}, id={target_id})")
    ax1.legend(fontsize=7, ncol=2)
    ax1.grid(True)

    kk = np.array([r["kd"] for r in rows])
    hh = np.array([r["hf_rms"] for r in rows])
    for r, color in zip(rows, cmap):
        ax2.scatter(r["kd"], r["hf_rms"], color=color, zorder=3, s=50)
    ax2.plot(kk, hh, "-", color="grey", lw=0.8, zorder=2)
    if hf_thresh:
        ax2.axhline(hf_thresh, c="grey", ls="--",
                    label=f"threshold ({hf_thresh:.3f})")
    if math.isfinite(float(kd_max)):
        ax2.axvline(float(kd_max), c="green", ls=":",
                    label=f"ceiling kd={float(kd_max):.3f}")
    ax2.set_ylabel(f"hf_rms  >{hf_cut:g} Hz  [rev/s]")
    ax2.set_xlabel("k_d  (motor frame)")
    ax2.legend(fontsize=8)
    ax2.grid(True)

    fig.tight_layout()
    out = d / "kd_sweep.svg"
    fig.savefig(str(out))
    print(f"Saved: {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
