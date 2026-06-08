#!/usr/bin/env python3
"""
analyze_jitter.py - analyse MIDI clock jitter captures from jitter-logger.ino.

Loads one or more captured Serial logs (each a separate run). For each run it
shows the residual-vs-tick plot so you can CLICK the warmup cutoff (where the
startup transient ends). It reports both the full-window sigma_t and the
steady-state (post-trim) sigma_t, then prints a console summary table and
ready-to-paste LaTeX rows.

Usage:
    python analyze_jitter.py                 # auto-load *bpm*.txt in this folder
    python analyze_jitter.py 40bpm.txt ...   # or pass files explicitly

Files are sorted by BPM (parsed from the filename), with busy-poll variants
placed right after their tempo. Filenames like "120bpm.txt" and
"120bpm-busypoll.txt" are recognised.

Per-file interaction:
    - left-click once where steady state begins (the warmup cutoff);
      close the window (or press enter without clicking) to keep the whole run;
    - then press any key / click to advance to the next file.

sigma_t is the rms deviation of the tick timestamps from a best-fit
constant-tempo grid. The grid is re-fitted on the trimmed data so the warmup
transient does not bias the steady-state estimate.

Requires: numpy, matplotlib (interactive backend)
"""

import sys
import os
import re
import glob
import numpy as np
import matplotlib.pyplot as plt

PPQN = 24  # MIDI clock ticks per quarter note


def load(path):
    """Parse (k, timestamp_us) from a captured Serial log (ignores '#' lines)."""
    k, t = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                k.append(int(parts[0]))
                t.append(float(parts[1]))
            except ValueError:
                continue
    return np.asarray(k, float), np.asarray(t, float)


def fit_sigma(k, t):
    """Fit t = a + b*k; return period[us], bpm, residuals[us], sigma_t[us]."""
    b, a = np.polyfit(k, t, 1)
    resid = t - (a + b * k)
    sigma = resid.std(ddof=2)            # 2 fitted parameters
    bpm = 60.0e6 / (b * PPQN)
    return b, bpm, resid, sigma


def sort_key(path):
    name = os.path.basename(path).lower()
    m = re.match(r"(\d+)", name)
    bpm = int(m.group(1)) if m else 0
    busy = "busy" in name
    return (bpm, busy)


def main():
    # Parse args: optional "--skip-sec S" applies one uniform time-trim to every
    # file (non-interactive, reproducible); remaining args are input files.
    argv = sys.argv[1:]
    skip_sec = None
    files = []
    i = 0
    while i < len(argv):
        if argv[i] == "--skip-sec" and i + 1 < len(argv):
            skip_sec = float(argv[i + 1])
            i += 2
        else:
            files.append(argv[i])
            i += 1
    if not files:
        files = glob.glob("*bpm*.txt")
    files = sorted(files, key=sort_key)
    if not files:
        print("no input files (pass filenames or run in a folder with *bpm*.txt)")
        sys.exit(1)

    rows = []
    for path in files:
        label = os.path.splitext(os.path.basename(path))[0]
        busy = "busy" in label.lower()
        k, t = load(path)
        if len(k) < 5:
            print(f"# {label}: too few ticks, skipping")
            continue

        period, bpm, resid, sigma_full = fit_sigma(k, t)
        pp = resid.max() - resid.min()

        if skip_sec is not None:
            # ---- uniform time-trim: discard the first skip_sec seconds ----
            trim = max(0, min(int(round(skip_sec * 1e6 / period)), len(k) - 3))
            _, _, _, sigma_steady = fit_sigma(k[trim:], t[trim:])
        else:
            # ---- interactive warmup-cutoff selection ----
            # Left-click sets/adjusts the cutoff; any key or closing the window
            # advances to the next file. plt.show() per figure is robust to the
            # window being X-ed out (unlike ginput/waitforbuttonpress).
            state = {"trim": 0, "sigma_steady": sigma_full}
            fig, ax = plt.subplots(figsize=(11, 4))
            ax.plot(k, resid, lw=0.7)
            ax.axhline(0.0, color="k", lw=0.5)
            ax.set_xlabel("tick index")
            ax.set_ylabel("residual [us]")
            vline = ax.axvline(0, color="r", ls="--", visible=False)
            ax.set_title(f"{label}   (sigma_full = {sigma_full:.1f} us)\n"
                         f"left-click warmup cutoff  --  press a key / close window to continue")
            fig.tight_layout()

            def on_click(event):
                if event.inaxes != ax or event.xdata is None or event.button != 1:
                    return
                trim = max(0, min(int(round(event.xdata)), len(k) - 3))
                _, _, _, ss = fit_sigma(k[trim:], t[trim:])
                state["trim"] = trim
                state["sigma_steady"] = ss
                vline.set_xdata([trim, trim])
                vline.set_visible(True)
                ax.set_title(f"{label}   full = {sigma_full:.1f} us    "
                             f"steady = {ss:.1f} us   (trim = {trim})\n"
                             f"left-click to adjust  --  press a key / close window to continue")
                fig.canvas.draw_idle()

            def on_key(_event):
                plt.close(fig)

            fig.canvas.mpl_connect("button_press_event", on_click)
            fig.canvas.mpl_connect("key_press_event", on_key)
            plt.show()   # blocks until the window is closed (key handler or window X)

            trim = state["trim"]
            sigma_steady = state["sigma_steady"]

        bpm_label = f"{round(bpm)} (busy-poll)" if busy else f"{round(bpm)}"
        rows.append({
            "label": label,
            "bpm_label": bpm_label,
            "bpm": bpm,
            "period_ms": period / 1000.0,
            "ticks": len(k),
            "trim": trim,
            "sigma_full": sigma_full,
            "sigma_steady": sigma_steady,
            "pp": pp,
        })
        print(f"{label}: bpm={bpm:.1f}  full={sigma_full:.1f}us  "
              f"steady={sigma_steady:.1f}us  trim={trim}  pp={pp:.0f}us")

    if not rows:
        return

    # ---- console summary ----
    print("\n# ---- summary ----")
    print(f"{'label':<20}{'bpm':>7}{'period_ms':>11}{'ticks':>8}"
          f"{'trim':>7}{'sig_full':>10}{'sig_steady':>12}{'pp_us':>9}")
    for r in rows:
        print(f"{r['label']:<20}{r['bpm']:>7.1f}{r['period_ms']:>11.3f}"
              f"{r['ticks']:>8}{r['trim']:>7}{r['sigma_full']:>10.1f}"
              f"{r['sigma_steady']:>12.1f}{r['pp']:>9.0f}")

    # ---- LaTeX rows: BPM & period[ms] & ticks & sigma_full & sigma_steady & pp ----
    print("\n# ---- LaTeX rows ----")
    for r in rows:
        print(f"{r['bpm_label']} & {r['period_ms']:.3f} & {r['ticks']} & "
              f"{r['sigma_full']:.0f} & {r['sigma_steady']:.0f} & {r['pp']:.0f} \\\\")


if __name__ == "__main__":
    main()
