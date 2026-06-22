#!/usr/bin/env python3
"""Plot the slip / load-estimator figures for the thesis Test chapter.

Companion to ``plot_position_controller.py`` — reads the same PlotJuggler CSV
exports of the ESP32 UDP telemetry stream and produces the figures for the
State Estimator subsection. The estimator is the two-state (alpha-beta)
constant-velocity tracker in ``firmware/src/state_estimator.h`` that fuses the
clean motor encoder with the noisy, wrapping MA600 load encoder.

Figures:

    slip      residual creep rate (theta_s_dot) and innovation e on top, with
              motor velocity underneath for reference — creep steps with speed
    ratioerr  the fractional ratio error Delta_i/i_hat [%] recovered from the
              creep rate, i.e. the linearisation residual of the held ratio

Notation follows design.tex, "Position Feed-Forward Based on Slip Estimation".
The alpha-beta filter tracks the slip angle theta_s = theta_m/i_hat - theta_L
[load rev] and the residual creep rate theta_s_dot [load rev/s]. At a fixed
speed v the creep is theta_s_dot = (Delta_i/i_hat) v, so dividing it by the load
speed gives the fractional ratio error Delta_i/i_hat (~1.5%). This is the held-
ratio linearisation residual, NOT the friction slip ratio s = (v_m - v_L)/v_m.

Telemetry keys (see firmware/src/controller.cpp), per motor {horn,drum}:
    slip/vel       est.slipVel    residual creep rate theta_s_dot [load rev/s]
    slip/err       est.innov      innovation e (eq:slip-innov) [load rev]
    vel_est/motor  v.velocity / i_hat  motor velocity in the load frame [rev/s]

Usage:
    cd software
    uv run python analysis/plot_slip_estimator.py LOG.csv --motor horn
    uv run python analysis/plot_slip_estimator.py LOG.csv --motor drum \
        --out ../Master-thesis/fig/implementation/test
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

TIME_COL = "__time"


def load(csv_path):
    import pandas as pd
    df = pd.read_csv(csv_path)
    # PlotJuggler prefixes every series with a leading "/"; strip it so keys
    # match the telemetry paths ("horn/slip/..." not "/horn/slip/...").
    df = df.rename(columns=lambda c: c.lstrip("/") if c != TIME_COL else c)
    if TIME_COL not in df.columns:
        for c in ("time", "Time", df.columns[0]):
            if c in df.columns:
                df = df.rename(columns={c: TIME_COL})
                break
    df[TIME_COL] -= df[TIME_COL].iloc[0]  # zero the time base
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


def _plot(ax, df, key, **kw):
    t, y = series(df, key)
    if t is not None:
        ax.plot(t, y, **kw)
        return True
    return False


def fig_slip(df, motor):
    # Notation follows design.tex, "Position Feed-Forward Based on Slip Estimation".
    # The alpha-beta filter tracks the slip angle theta_s = theta_m/i_hat - theta_L
    # (load rev) and its rate, the residual creep rate d(theta_s)/dt (load rev/s).
    # These are the estimator states; the friction slip ratio s of the theory is a
    # different (dimensionless) quantity, recovered separately in fig_slipratio.
    #
    # Top: the residual creep rate (theta_s_dot, used for velocity feedforward) and
    # the innovation e (eq:slip-innov) driving the corrections — a settled
    # innovation around 0 means the tracker is locked.
    # Bottom: motor velocity for reference — the creep rate scales with speed
    # (theta_s_dot = (Delta_i/i_hat) v), so its steps line up with the speed below.
    p = f"{motor}/slip"
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    _plot(ax1, df, f"{p}/err", color="0.6", lw=0.8, zorder=1,
          label=r"Innovation $e = \theta_{s,\mathrm{raw}} - \hat\theta_s$")
    _plot(ax1, df, f"{p}/vel", color="C3", lw=1.4, zorder=2,
          label=r"Residual creep rate $\hat{\dot\theta}_s$")
    te, err = series(df, f"{p}/err")
    if te is not None:
        emean = float(np.mean(err))
        ax1.axhline(emean, color="k", lw=0.8, ls="--",
                    label=f"innovation mean {emean:.2e} rev")
    ax1.axhline(0, color="k", lw=0.5)
    ax1.set_ylabel("Creep rate [load rev/s]")
    ax1.legend(loc="best", frameon=False)

    _plot(ax2, df, f"{motor}/vel_est/motor", color="C0", lw=1.4,
          label=r"Motor velocity $\dot\theta_\mathrm{m}/\hat\imath$")
    ax2.axhline(0, color="k", lw=0.5)
    ax2.set_ylabel("Load velocity [rev/s]")
    ax2.set_xlabel("Time [s]")
    ax2.legend(loc="best", frameon=False)
    fig.tight_layout()
    return fig


def fig_ratioerr(df, motor, vmin=1.0, amax=0.3):
    # Recover the fractional ratio error Delta_i/i_hat from the estimator. At a
    # fixed speed v the residual creep rate is theta_s_dot = (Delta_i/i_hat) v
    # (design.tex eq:slip-angle), so
    #   Delta_i/i_hat = theta_s_dot / v = slip/vel / (theta_m_dot/i_hat).
    # This is the linearisation residual of the held ratio i_hat = i_0, NOT the
    # friction slip ratio s. The derivation assumes a *fixed operating point*, so
    # for a clean steady-state figure we keep only samples that are both spinning
    # (|v| > vmin [rev/s]) and not accelerating (|dv/dt| < amax [rev/s^2]); the
    # speed steps in between are transients where the ratio is not meaningful.
    t = df[TIME_COL].to_numpy()
    sv = df.get(f"{motor}/slip/vel")
    vm = df.get(f"{motor}/vel_est/motor")
    fig, ax = plt.subplots(figsize=(9, 4.5))
    usetex = plt.rcParams["text.usetex"]
    if sv is not None and vm is not None:
        sv = sv.to_numpy()
        vm = vm.to_numpy()
        finite = ~np.isnan(sv) & ~np.isnan(vm)
        accel = np.full_like(vm, np.nan)
        accel[finite] = np.gradient(vm[finite], t[finite])
        m = finite & (np.abs(vm) > vmin) & (np.abs(accel) < amax)
        r = 100.0 * sv[m] / vm[m]
        ax.plot(t[m], r, ".", color="C3", ms=2.5,
                label=r"Estimated ratio error $\Delta\hat\imath/\hat\imath$")
        mean = float(np.median(r))
        ax.axhline(mean, color="k", lw=0.8, ls="--",
                   label=f"median {mean:.2f} %")
    ax.set_ylabel(r"$\Delta i/\hat\imath$ [\%]" if usetex else "Ratio error [%]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="best", frameon=False)
    fig.tight_layout()
    return fig


FIGS = {"slip": fig_slip, "ratioerr": fig_ratioerr}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="PlotJuggler CSV export")
    ap.add_argument("--motor", default="horn", choices=("horn", "drum"))
    ap.add_argument("--fig", default="slip", choices=tuple(FIGS) + ("all",))
    ap.add_argument("--out", default=None, help="dir to save SVG (default: alongside CSV)")
    ap.add_argument("--vmin", type=float, default=1.0,
                    help="ratioerr: min |speed| [rev/s] to include (default 1.0)")
    ap.add_argument("--amax", type=float, default=0.3,
                    help="ratioerr: max |accel| [rev/s^2] to include (default 0.3)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    df = load(args.csv)
    out_dir = Path(args.out) if args.out else Path(args.csv).parent
    out_dir.mkdir(parents=True, exist_ok=True)

    names = list(FIGS) if args.fig == "all" else [args.fig]
    for name in names:
        if name == "ratioerr":
            fig = fig_ratioerr(df, args.motor, vmin=args.vmin, amax=args.amax)
        else:
            fig = FIGS[name](df, args.motor)
        p = out_dir / f"slipest_{name}_{args.motor}.svg"
        fig.savefig(p)
        print(f"wrote {p}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
