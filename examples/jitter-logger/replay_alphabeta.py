#!/usr/bin/env python3
"""
replay_alphabeta.py - offline replay/tuning of the MIDI-clock alpha-beta filter.

The on-target filter (src/clock_sync.cpp, processTick) is pure arithmetic on the
tick arrival timestamps. The jitter-logger captures exactly those timestamps, so
we can replay the *identical* update here, with no re-flashing, to:

  * measure the one-step-ahead prediction residual (the filter-quality metric),
  * sweep the gains (g=alpha, h=beta) and pick the pair with the lowest
    steady-state residual RMS.

This mirrors the C++ logic exactly:
    xPred = nextPred                       # a-priori prediction for this tick
    err   = t - xPred                      # residual (logged BEFORE clamping)
    err   = clamp(err, +/- 0.5*period)     # outlier rejection
    period = clamp(period + h*err)         # velocity (period) update
    xEst   = xPred + g*err                 # position update
    nextPred = xEst + period               # predict next tick

Usage:
    python replay_alphabeta.py                      # auto-load *bpm*.txt here
    python replay_alphabeta.py 120bpm.txt 200bpm.txt
    python replay_alphabeta.py --sweep              # grid-search g,h
    python replay_alphabeta.py --plot               # residual-vs-tick plots
    python replay_alphabeta.py -g 0.16 -b 0.014     # try specific gains

The residual reported is the raw (pre-clamp) prediction error t - xPred, which is
what the firmware exposes via clockSyncGetLastErrUs(). The first --warmup ticks
are excluded from the steady-state RMS to drop the lock-in transient.
"""

import argparse
import glob
import math
import os
import re
import sys

# ---- firmware constants (keep in sync with src/clock_sync.cpp) ----
BPM_MIN = 30.0
BPM_MAX = 300.0
G_ALPHA = 0.16
H_BETA = 0.014
INIT_BPM = 120.0


def bpm_to_tick_period_us(bpm):
    # MIDI clock = 24 ticks / quarter note
    return 60.0e6 / (bpm * 24.0)


def tick_period_us_to_bpm(period_us):
    return 60.0e6 / (period_us * 24.0)


def clampd(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def load_capture(path):
    """Return list of tick arrival timestamps [us] from a jitter-logger CSV."""
    ts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                ts.append(int(parts[1]))  # k,timestamp_us,residual_us
            except ValueError:
                continue
    return ts


def replay(timestamps, g=G_ALPHA, h=H_BETA):
    """Run the alpha-beta filter over arrival timestamps.

    Returns (locked_k, residuals_us, bpm_est, bpm_raw) where residuals are the
    raw pre-clamp prediction errors t - xPred for every locked tick, locked_k is
    the tick index (into timestamps) each residual belongs to, bpm_est is the
    filtered (period-state-derived) BPM, and bpm_raw is the unfiltered
    instantaneous BPM from the single inter-arrival gap at that tick.
    """
    period_min = bpm_to_tick_period_us(BPM_MAX)
    period_max = bpm_to_tick_period_us(BPM_MIN)

    period = bpm_to_tick_period_us(INIT_BPM)
    next_pred = 0.0
    last_tick = 0
    locked = False

    locked_k = []
    residuals = []
    bpm_est = []
    bpm_raw = []

    for k, t in enumerate(timestamps):
        if not locked:
            if last_tick == 0:
                last_tick = t
                continue
            dt = t - last_tick
            last_tick = t
            if dt < 1000 or dt > 200000:  # sanity reject
                continue
            period = clampd(float(dt), period_min, period_max)
            next_pred = float(t) + period
            locked = True
            continue

        # Clock-loss / discontinuity: the firmware drops lock after a >500 ms
        # gap (resetLockKeepPeriod). A capture spanning a stop/restart shows up
        # as a negative or absurd inter-arrival gap; re-arm instead of feeding a
        # garbage residual into the filter.
        gap = t - last_tick
        if gap < 0 or gap > 500000:
            locked = False
            last_tick = t
            continue

        x_pred = next_pred
        err = float(t) - x_pred  # raw residual (logged before clamp)

        locked_k.append(k)
        residuals.append(err)
        bpm_est.append(tick_period_us_to_bpm(period))
        bpm_raw.append(tick_period_us_to_bpm(gap))  # unfiltered, from single dt

        err_clamp = 0.50 * period
        err = clampd(err, -err_clamp, err_clamp)
        period = clampd(period + h * err, period_min, period_max)
        x_est = x_pred + g * err
        last_tick = t
        next_pred = x_est + period

    return locked_k, residuals, bpm_est, bpm_raw


def rms(xs):
    if not xs:
        return float("nan")
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def stats(residuals, warmup):
    """Steady-state residual stats, skipping the first `warmup` locked ticks."""
    ss = residuals[warmup:] if warmup < len(residuals) else []
    return {
        "n": len(residuals),
        "n_ss": len(ss),
        "rms_full": rms(residuals),
        "rms_ss": rms(ss),
        "max_abs_ss": max((abs(x) for x in ss), default=float("nan")),
    }


def parse_bpm(path):
    m = re.search(r"(\d+)bpm", os.path.basename(path))
    return int(m.group(1)) if m else 1_000_000


def default_files():
    here = os.path.dirname(os.path.abspath(__file__))
    files = glob.glob(os.path.join(here, "*bpm*.txt"))
    return sorted(files, key=lambda p: (parse_bpm(p), "busypoll" in p))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="jitter-logger capture(s); default *bpm*.txt")
    ap.add_argument("-g", "--alpha", type=float, default=G_ALPHA, help="position gain g (alpha)")
    ap.add_argument("-b", "--beta", type=float, default=H_BETA, help="velocity gain h (beta)")
    ap.add_argument("--warmup", type=int, default=200,
                    help="locked ticks to skip for steady-state stats (default 200)")
    ap.add_argument("--sweep", action="store_true", help="grid-search g,h over all files")
    ap.add_argument("--plot", action="store_true", help="show residual-vs-tick plots")
    args = ap.parse_args()

    files = args.files or default_files()
    if not files:
        print("No capture files found (looked for *bpm*.txt).", file=sys.stderr)
        return 1

    captures = [(f, load_capture(f)) for f in files]
    captures = [(f, ts) for f, ts in captures if len(ts) >= 3]
    if not captures:
        print("No usable captures (need >= 3 ticks each).", file=sys.stderr)
        return 1

    if args.sweep:
        run_sweep(captures, args.warmup)
    else:
        run_single(captures, args.alpha, args.beta, args.warmup)

    if args.plot:
        do_plots(captures, args.alpha, args.beta)

    return 0


def run_single(captures, g, h, warmup):
    print(f"# alpha-beta replay   g(alpha)={g}  h(beta)={h}  warmup={warmup} ticks")
    print(f"# {'file':<24} {'ticks':>6} {'BPM_est':>8} "
          f"{'rms_full':>10} {'rms_ss':>10} {'maxabs_ss':>10}  (us)")
    for f, ts in captures:
        _, resid, bpm, _ = replay(ts, g, h)
        s = stats(resid, warmup)
        bpm_final = bpm[-1] if bpm else float("nan")
        print(f"  {os.path.basename(f):<24} {s['n']:>6d} {bpm_final:>8.3f} "
              f"{s['rms_full']:>10.1f} {s['rms_ss']:>10.1f} {s['max_abs_ss']:>10.1f}")
    print("# rms_ss = steady-state RMS of the one-step prediction residual "
          "(t - xPred), the filter-quality metric.")


def run_sweep(captures, warmup):
    g_grid = [0.05, 0.08, 0.10, 0.12, 0.16, 0.20, 0.25, 0.30, 0.40]
    h_grid = [0.002, 0.005, 0.008, 0.010, 0.014, 0.020, 0.030, 0.050]
    print(f"# gain sweep over {len(captures)} file(s); metric = mean steady-state "
          f"residual RMS (us), warmup={warmup}")
    results = []
    for g in g_grid:
        for h in h_grid:
            rmss = []
            for _, ts in captures:
                _, resid, _, _ = replay(ts, g, h)
                rmss.append(stats(resid, warmup)["rms_ss"])
            rmss = [r for r in rmss if not math.isnan(r)]
            if rmss:
                results.append((sum(rmss) / len(rmss), g, h))

    results.sort()
    print(f"# {'rank':>4} {'g(alpha)':>9} {'h(beta)':>9} {'mean_rms_ss':>12}")
    for i, (m, g, h) in enumerate(results[:15], 1):
        mark = "  <- current" if (abs(g - G_ALPHA) < 1e-9 and abs(h - H_BETA) < 1e-9) else ""
        print(f"  {i:>4} {g:>9.3f} {h:>9.3f} {m:>12.1f}{mark}")
    best = results[0]
    print(f"# best: g={best[1]} h={best[2]}  mean_rms_ss={best[0]:.1f} us")


def do_plots(captures, g, h):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plots.", file=sys.stderr)
        return
    n = len(captures)
    fig, axes = plt.subplots(n, 1, figsize=(10, 3.0 * n), squeeze=False)
    for ax, (f, ts) in zip(axes[:, 0], captures):
        k, _, bpm_est, bpm_raw = replay(ts, g, h)
        ax.plot(k, bpm_raw, lw=0.5, color="0.6", label="unfiltered (per-tick dt)")
        ax.plot(k, bpm_est, lw=1.2, color="C0", label="filtered (alpha-beta)")
        ax.set_title(f"{os.path.basename(f)}  (g={g}, h={h})")
        ax.set_ylabel("BPM")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xlabel("locked tick index")
    fig.tight_layout()
    out = os.path.join(os.path.dirname(captures[0][0]), "alphabeta-residuals.png")
    fig.savefig(out, dpi=120)
    print(f"# saved {out}")
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
