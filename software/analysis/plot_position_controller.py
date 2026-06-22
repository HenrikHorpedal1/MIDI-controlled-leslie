#!/usr/bin/env python3
"""Plot the position-controller figures for the thesis Test chapter.

Reads a PlotJuggler CSV export (File -> Export -> Data to CSV) of the ESP32 UDP
telemetry stream and produces the three figures described in the Position
Controller subsection:

    park       position step-and-hold, showing overshoot
    velocity   continuous / chorale / tremolo-stop velocity responses
    beatsync   beat-sync error and the transient when subdivision changes

For each figure we overlay the controller's three "states":
    traj  -> moteus internal planner reference (…/posctrl/{pos,vel}/traj)
    meas  -> measurement                       (…/posctrl/{pos,vel,torque}/meas)
    cmd   -> what moteus receives              (…/posctrl/torque/cmd, …/beat/cmd_*)

PlotJuggler CSV layout: first column "__time" (seconds), remaining columns are
named by the telemetry key, e.g. "horn/posctrl/vel/traj". Missing series are
skipped gracefully so partial recordings still plot.

Usage:
    cd software
    uv run python analysis/plot_position_controller.py LOG.csv --motor horn --fig park
    uv run python analysis/plot_position_controller.py LOG.csv --motor drum --fig velocity
    uv run python analysis/plot_position_controller.py LOG.csv --fig beatsync \
        --out ../Master-thesis/fig/implementation/test
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

TIME_COL = "__time"


def load(csv_path):
    df = pd.read_csv(csv_path)
    # PlotJuggler prefixes every series with a leading "/"; strip it so keys
    # match the telemetry paths ("horn/posctrl/..." not "/horn/posctrl/...").
    df = df.rename(columns=lambda c: c.lstrip("/") if c != TIME_COL else c)
    if TIME_COL not in df.columns:
        # Some PlotJuggler versions name it "time" or put an index column.
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


def fig_park(df, motor):
    p = f"{motor}/posctrl"
    fig, ax = plt.subplots(figsize=(9, 5))
    _plot(ax, df, f"{p}/pos/cmd",  color="C1", lw=1.2, ls="--",
          label=r"Commanded setpoint $\theta_\mathrm{cmd}$")
    _plot(ax, df, f"{p}/pos/traj", color="C0", lw=1.2,
          label=r"Planner reference $\theta_\mathrm{ref}$")
    _plot(ax, df, f"{p}/pos/meas", color="C3", lw=1.0,
          label=r"Measured position $\theta$")
    ax.set_ylabel("Position [rev]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="best", frameon=False)
    fig.tight_layout()
    return fig


def fig_velocity(df, motor):
    # All signals are in the motor frame, so they overlay directly. Measured
    # traces are noisy, so they are drawn underneath (low zorder) and the
    # references on top so the planner trajectories stay readable.
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    _plot(ax1, df, f"{motor}/posctrl/vel/meas", color="0.6", lw=0.8, zorder=1,
          label=r"Measured velocity $\dot\theta$")
    _plot(ax1, df, f"{motor}/beat/cmd_vel_revs", color="C1", lw=1.3, ls=":", zorder=2,
          label=r"Commanded velocity $\dot\theta_\mathrm{cmd}$")
    _plot(ax1, df, f"{motor}/posctrl/vel/traj", color="C0", lw=1.3, ls="--", zorder=3,
          label=r"Planner reference $\dot\theta_\mathrm{ref}$")
    ax1.set_ylabel("Velocity [rev/s]")
    ax1.legend(loc="best", frameon=False)

    _plot(ax2, df, f"{motor}/posctrl/torque/meas", color="0.6", lw=0.8, zorder=1,
          label=r"Measured torque $\tau$")
    _plot(ax2, df, f"{motor}/posctrl/torque/cmd", color="C0", lw=1.3, ls="--", zorder=2,
          label=r"Torque reference $\tau_\mathrm{ref}$")
    _plot(ax2, df, f"{motor}/posctrl/torque/ff", color="C1", lw=1.3, ls=":", zorder=3,
          label=r"Feedforward torque $\tau_\mathrm{ff}$")
    ax2.set_ylabel("Torque [Nm]")
    ax2.set_xlabel("Time [s]")
    ax2.legend(loc="best", frameon=False)
    fig.tight_layout()
    return fig


def fig_beatsync(df, motor):
    # Start the figure where the first 1/2 subdivision begins, dropping the
    # initial spin-up / lock-acquisition transient; re-zero the time axis.
    tsub, sub = series(df, "beat/subdivision")
    if tsub is not None:
        half = np.where(np.round(sub).astype(int) == 1)[0]
        if len(half):
            t0 = tsub[half[0]]
            df = df[df[TIME_COL] >= t0].copy()
            df[TIME_COL] = df[TIME_COL] - t0

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    # -- phase error ------------------------------------------------------------
    # Within each held-subdivision segment, unwrap so the re-lock transient reads
    # as a single spike (not a +/-180 sawtooth), then re-zero to the settled tail
    # so the spike returns to a 0 baseline instead of staying offset by whole turns.
    te, ed = series(df, f"{motor}/beat/err_deg")
    ts0, sub0 = series(df, "beat/subdivision")
    if te is not None:
        ed = ed.copy()
        if ts0 is not None:
            bnds = [te[0]] + [ts0[i + 1] for i in np.where(np.diff(sub0) != 0)[0]] + [te[-1]]
        else:
            bnds = [te[0], te[-1]]
        for a, b in zip(bnds[:-1], bnds[1:]):
            seg = (te >= a) & (te <= b)
            if seg.sum() > 1:
                u = np.unwrap(ed[seg], period=360.0)
                tail = te[seg] >= a + 0.5 * (b - a)   # settled half of the segment
                u -= np.median(u[tail]) if tail.any() else np.median(u)
                ed[seg] = u
        ax1.plot(te, ed, color="C3", lw=1.0, label=r"Phase error $e_\phi$")
    ax1.axhline(0, color="k", lw=0.5)
    ax1.set_ylabel("Phase error [deg]")
    ax1.legend(loc="best", frameon=False)

    # -- velocity transient -----------------------------------------------------
    _plot(ax2, df, f"{motor}/posctrl/vel/meas", color="0.6", lw=0.8, zorder=1,
          label=r"Measured velocity $\dot\theta$")
    _plot(ax2, df, f"{motor}/beat/cmd_vel_revs", color="C1", lw=1.3, ls=":", zorder=2,
          label=r"Commanded velocity $\dot\theta_\mathrm{cmd}$")
    _plot(ax2, df, f"{motor}/posctrl/vel/traj", color="C0", lw=1.3, ls="--", zorder=3,
          label=r"Planner reference $\dot\theta_\mathrm{ref}$")
    ax2.set_ylabel("Velocity [rev/s]")
    ax2.legend(loc="best", frameon=False)

    # -- active subdivision, with change markers across all panels --------------
    t, sub = series(df, "beat/subdivision")
    if t is not None:
        ax3.step(t, sub, where="post", color="C0", lw=1.2)
        ch = np.where(np.diff(sub) != 0)[0]
        for ax in (ax1, ax2, ax3):
            for i in ch:
                ax.axvline(t[i + 1], color="0.7", ls="--", lw=0.8)
        # label the y-axis with the perceived musical subdivisions present
        present = sorted(set(int(round(v)) for v in sub))
        ax3.set_yticks(present)
        ax3.set_yticklabels([SUBDIV_LABELS.get(v, str(v)) for v in present])
    ax3.set_ylabel("Subdivision")
    ax3.set_xlabel("Time [s]")
    fig.tight_layout()
    return fig


# Subdivision enum index -> perceived musical name (matches Subdivision in
# firmware/src/beat_sync.h). Used to label the beat-sync subdivision axis.
SUBDIV_LABELS = {
    0: "Rest", 1: "1/2", 2: "1/4", 3: "1/8.", 4: "1/4T", 5: "1/8",
    6: "1/16.", 7: "1/8T", 8: "1/16", 9: "1/32.", 10: "1/16T",
    11: "1/32", 12: "1/32T",
}


def _circ_stats(deg):
    """Circular mean and std [deg] of an array of angles in degrees."""
    ang = np.deg2rad(deg)
    c, s = np.cos(ang).mean(), np.sin(ang).mean()
    mean = np.rad2deg(np.arctan2(s, c))
    R = min(1.0, float(np.hypot(c, s)))
    std = abs(np.rad2deg(np.sqrt(max(0.0, -2.0 * np.log(R))))) if R > 0 else float("nan")
    return mean, std


def beatsync_offset_table(df):
    """Steady-state phase offset +/- circular std per subdivision, both rotors.

    For each held subdivision segment the settled tail (last 50%) is taken, and
    all such tails for a given subdivision are pooled before the circular stats.
    Returns a LaTeX tabular (string) with one row per subdivision present.
    """
    ts, sub = series(df, "beat/subdivision")
    if ts is None:
        return ""
    ch = np.where(np.diff(sub) != 0)[0]
    bounds = [ts[0]] + [ts[i + 1] for i in ch] + [ts[-1]]

    # collect settled-tail samples per subdivision index, per motor
    pooled = {m: {} for m in ("drum", "horn")}
    for m in ("drum", "horn"):
        te, ed = series(df, f"{m}/beat/err_deg")
        if te is None:
            continue
        for a, b in zip(bounds[:-1], bounds[1:]):
            tail = a + 0.5 * (b - a)
            seg = (te >= tail) & (te <= b)
            if seg.sum() < 3:
                continue
            sm = (ts >= a) & (ts < b)
            if not sm.any():
                continue
            sd = int(round(np.median(sub[sm])))
            pooled[m].setdefault(sd, []).append(ed[seg])

    subs = sorted(set(pooled["drum"]) | set(pooled["horn"]))
    rows = []
    for sd in subs:
        cells = [SUBDIV_LABELS.get(sd, str(sd))]
        for m in ("drum", "horn"):
            if sd in pooled[m]:
                mean, std = _circ_stats(np.concatenate(pooled[m][sd]))
                cells.append(rf"${mean:+.1f} \pm {std:.1f}$")
            else:
                cells.append("--")
        rows.append(" & ".join(cells) + r" \\")

    body = "\n".join(rows)
    return (
        "\\begin{tabular}{lrr}\n"
        "\t\\toprule\n"
        "\tSubdivision & Drum [deg] & Horn [deg] \\\\\n"
        "\t\\midrule\n\t" + body.replace("\n", "\n\t") + "\n"
        "\t\\bottomrule\n"
        "\\end{tabular}\n"
    )


FIGS = {"park": fig_park, "velocity": fig_velocity, "beatsync": fig_beatsync}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="PlotJuggler CSV export")
    ap.add_argument("--motor", default="horn", choices=("horn", "drum"))
    ap.add_argument("--fig", default="park", choices=tuple(FIGS) + ("all",))
    ap.add_argument("--out", default=None, help="dir to save SVG (default: alongside CSV)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    df = load(args.csv)
    out_dir = Path(args.out) if args.out else Path(args.csv).parent
    out_dir.mkdir(parents=True, exist_ok=True)

    names = list(FIGS) if args.fig == "all" else [args.fig]
    for name in names:
        fig = FIGS[name](df, args.motor)
        p = out_dir / f"posctrl_{name}_{args.motor}.svg"
        fig.savefig(p)
        print(f"wrote {p}")
        # beat-sync also emits the steady-state offset table (motor-independent)
        if name == "beatsync":
            tbl = beatsync_offset_table(df)
            if tbl:
                tp = out_dir / "posctrl_beatsync_offsets.tex"
                tp.write_text(tbl)
                print(f"wrote {tp}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
