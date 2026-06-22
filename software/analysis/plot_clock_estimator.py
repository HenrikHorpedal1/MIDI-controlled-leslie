#!/usr/bin/env python3
"""Plot the MIDI clock-estimator figure for the thesis Test chapter.

Companion to ``plot_slip_estimator.py`` — reads the same PlotJuggler CSV export
of the ESP32 UDP telemetry and produces the figure for the Clock Estimator
subsection. The estimator is the two-state (alpha-beta) constant-velocity
tracker of ``firmware/src/clock_sync.cpp``: it tracks the tick arrival time
(position) and the tick period (velocity = tempo), rejecting the per-tick USB
timing jitter on the incoming 24 PPQN MIDI clock.

Notation follows design.tex, "Clock Estimator" (eq:clk-update). The filter
states are the predicted tick time t_hat and the tick period T_hat; the
innovation is e_n = t_n - t_hat_n, the per-tick prediction error.

Figure:

    clock     top: instantaneous BPM from the raw per-tick period vs the filtered
              tempo estimate (jitter rejection); bottom: the innovation e_n [ms]

Telemetry keys (see firmware/src/clock_sync.cpp):
    clock/raw_period_us  raw inter-tick period (noisy measurement) [us]
    clock/period_us      filtered period T_hat [us]
    clock/bpm            smoothed tempo = 60e6 / (24 * T_hat) [bpm]
    clock/pred_err_us    prediction residual e_n = t_n - t_hat_n [us]
    clock/locked         1 while the PLL is locked

Usage:
    cd software
    uv run python analysis/plot_clock_estimator.py LOG.csv
    uv run python analysis/plot_clock_estimator.py LOG.csv \
        --out ../Master-thesis/fig/implementation/test
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

TIME_COL = "__time"
TICKS_PER_QUARTER = 24  # MIDI clock resolution


def load(csv_path):
    import pandas as pd
    df = pd.read_csv(csv_path)
    df = df.rename(columns=lambda c: c.lstrip("/") if c != TIME_COL else c)
    if TIME_COL not in df.columns:
        for c in ("time", "Time", df.columns[0]):
            if c in df.columns:
                df = df.rename(columns={c: TIME_COL})
                break
    df[TIME_COL] -= df[TIME_COL].iloc[0]
    return df


def series(df, key):
    """Return (t, y) for a key, dropping NaNs; (None, None) if absent."""
    if key not in df.columns:
        return None, None
    y = df[key].to_numpy()
    t = df[TIME_COL].to_numpy()
    m = ~np.isnan(y)
    if not m.any():
        return None, None
    return t[m], y[m]


def _period_to_bpm(period_us):
    return 60e6 / (TICKS_PER_QUARTER * period_us)


def fig_clock(df):
    # Only plot while the PLL is locked, so the initial acquisition transient
    # (and the stop tail) does not dominate the tempo axis.
    tl, lk = series(df, "clock/locked")
    if tl is not None:
        on = np.where(lk > 0.5)[0]
        if len(on):
            t0, t1 = tl[on[0]], tl[on[-1]]
            df = df[(df[TIME_COL] >= t0) & (df[TIME_COL] <= t1)].copy()
            df[TIME_COL] -= t0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    # -- tempo: raw per-tick vs filtered ---------------------------------------
    tr, raw = series(df, "clock/raw_period_us")
    if tr is not None:
        ax1.plot(tr, _period_to_bpm(raw), color="0.6", lw=0.7, zorder=1,
                 label="Raw per-tick tempo")
    tb, bpm = series(df, "clock/bpm")
    if tb is not None:
        ax1.plot(tb, bpm, color="C0", lw=1.6, zorder=2,
                 label=r"Filtered tempo (from $\hat T$)")
        med = float(np.median(bpm))
        ax1.axhline(med, color="k", lw=0.8, ls="--",
                    label=f"median {med:.1f} bpm")
    ax1.set_ylabel("Tempo [bpm]")
    ax1.legend(loc="best", frameon=False)

    # -- innovation ------------------------------------------------------------
    te, err = series(df, "clock/pred_err_us")
    if te is not None:
        err_ms = err / 1e3
        ax2.plot(te, err_ms, color="C3", lw=0.9,
                 label=r"Innovation $e_n = t_n - \hat t_n$")
        mean = float(np.mean(err_ms))
        sd = float(np.std(err_ms))
        ax2.axhline(mean, color="k", lw=0.8, ls="--",
                    label=f"mean {mean:.3f} ms")
        ax2.set_title(rf"timing jitter $\sigma \approx {sd:.2f}$ ms", fontsize=9)
    ax2.axhline(0, color="k", lw=0.5)
    ax2.set_ylabel("Innovation [ms]")
    ax2.set_xlabel("Time [s]")
    ax2.legend(loc="best", frameon=False)
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="PlotJuggler CSV export")
    ap.add_argument("--out", default=None, help="dir to save SVG (default: alongside CSV)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    df = load(args.csv)
    out_dir = Path(args.out) if args.out else Path(args.csv).parent
    out_dir.mkdir(parents=True, exist_ok=True)

    fig = fig_clock(df)
    p = out_dir / "clockest_clock.svg"
    fig.savefig(p)
    print(f"wrote {p}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
