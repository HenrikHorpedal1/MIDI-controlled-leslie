#!/usr/bin/env python3
"""Passive impulse/ringdown modal test of the belt drive.

Identifies the high-frequency belt mode (w_h, zeta_h) WITHOUT driving it through
the current loop -- the resonance is excited mechanically (you tap/pluck the drum
or belt by hand) while the motor is de-energised and freewheeling.  The script
just logs the encoder at high rate; the structure rings at its damped natural
frequency and decays, and the log-decrement of the envelope gives the damping.

This avoids a swept-sine FRF, which would require the torque controller to remain
stable at the resonance -- exactly the regime we suspect is fragile.

    uv run python moteus-config/scripts/capture_ringdown.py -t 1 --duration 4

Procedure: run it, wait for "TAP NOW", give the drum ONE sharp tap, let it ring
out.  Repeat with --label to average several taps if desired.

Output: w_h (rad/s and Hz) from the dominant ringing peak, and zeta_h from a
log-decrement fit of the positive-peak envelope, plus a plot.
"""

import argparse
import asyncio
import time

import moteus
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run


def analyse(t, x, f_lo, f_hi):
    """Return (w_d, zeta, n_peaks) from a decaying oscillation x(t).

    x is detrended; positive local maxima are picked and an exponential is fit to
    their amplitudes (log-decrement).  w_d is taken from the mean peak spacing and
    cross-checked against the FFT.
    """
    x = x - np.mean(x)
    dt = np.median(np.diff(t))

    # FFT estimate of the ring frequency (within the band of interest) first,
    # so we can band-pass tightly around it before peak-picking.
    X = np.abs(np.fft.rfft(x))
    fr = np.fft.rfftfreq(len(x), dt)
    band = (fr >= f_lo) & (fr <= f_hi)
    if not band.any():
        return float("nan"), float("nan"), 0
    f_fft = fr[band][np.argmax(X[band])]

    # zero-phase band-pass (FFT mask) around the identified peak +/- one octave,
    # killing the low-freq drift and the HF encoder-quantisation noise that
    # otherwise spawns hundreds of spurious peaks.
    lo, hi = 0.5 * f_fft, 2.0 * f_fft
    Xc = np.fft.rfft(x)
    Xc[(fr < lo) | (fr > hi)] = 0.0
    xf = np.fft.irfft(Xc, n=len(x))

    # positive local maxima with an amplitude threshold (prominence)
    thr = 0.05 * np.max(np.abs(xf))
    pk = np.where((xf[1:-1] > xf[:-2]) & (xf[1:-1] > xf[2:])
                  & (xf[1:-1] > thr))[0] + 1
    if len(pk) < 3:
        return 2 * np.pi * f_fft, float("nan"), len(pk)
    tp, ap = t[pk], xf[pk]
    # keep only the decaying tail: from the global max peak onward
    i0 = np.argmax(ap)
    tp, ap = tp[i0:], ap[i0:]
    ap = np.maximum(ap, 1e-12)
    # log-decrement: ln(A) = ln(A0) - alpha * t
    coef = np.polyfit(tp - tp[0], np.log(ap), 1)
    alpha = -coef[0]                       # envelope decay rate [1/s]
    w_d = 2 * np.pi * f_fft                # damped natural freq [rad/s]
    w_n = np.hypot(w_d, alpha)
    zeta = alpha / w_n if w_n else float("nan")
    return w_d, zeta, len(pk)


async def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", "-t", type=int, default=1,
                   help="moteus CAN id (1=drum, 2=horn)")
    p.add_argument("--duration", type=float, default=4.0,
                   help="record time [s] (tap once near the start)")
    p.add_argument("--countdown", type=float, default=2.0,
                   help="seconds of logging before the 'TAP NOW' prompt")
    p.add_argument("--f-lo", type=float, default=40.0,
                   help="low edge of the ring-frequency search band [Hz]")
    p.add_argument("--f-hi", type=float, default=120.0,
                   help="high edge of the ring-frequency search band [Hz]")
    p.add_argument("--hold", action="store_true",
                   help="hold position with the current gains instead of "
                        "freewheeling, so the motor side is anchored like in "
                        "operation (excites the held/operating belt mode, not the "
                        "free-free mode). Tap the drum against the hold.")
    p.add_argument("--max-torque", type=float, default=0.3,
                   help="maximum_torque clamp when --hold [Nm]")
    p.add_argument("--label", default=None,
                   help="optional tag appended to the artifact folder name")
    p.add_argument("--no-show", action="store_true",
                   help="save the plot without opening a window")
    args = p.parse_args()

    run = Run("ringdown", target=args.target, label=args.label)

    qr = moteus.QueryResolution()
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    controller = moteus.Controller(id=args.target, query_resolution=qr)
    stream = moteus.Stream(controller)

    await stream.write_message(b"tel stop")
    await stream.flush_read()
    await controller.set_stop()
    await asyncio.sleep(0.2)

    hold_pos = None
    if args.hold:
        # capture current position and hold it with the live gains, so the motor
        # side is anchored like in operation -- the belt mode then rings at the
        # operating (held) frequency, not the free-free one.
        r0 = await controller.set_position(position=None, query=True)
        hold_pos = r0.values[moteus.Register.POSITION]
        mode = f"HOLDING position {hold_pos:.3f} rev (live gains)"
    else:
        mode = "OFF (freewheeling)"

    print(f"Logging for {args.duration:.1f} s on moteus id={args.target} "
          f"({run.motor}). Motor is {mode}.")
    ts, ps, vs = [], [], []
    t0 = time.monotonic()
    tapped = False
    while True:
        t = time.monotonic() - t0
        if t > args.duration:
            break
        if not tapped and t >= args.countdown:
            print(">>> TAP NOW (one sharp tap on the drum) <<<")
            tapped = True
        if args.hold:
            r = await controller.set_position(
                position=hold_pos, velocity=0.0,
                maximum_torque=args.max_torque, query=True)
        else:
            r = await controller.query()
        ts.append(t)
        ps.append(r.values[moteus.Register.POSITION])
        vs.append(r.values[moteus.Register.VELOCITY])

    await controller.set_stop()

    t = np.array(ts)
    pos = np.array(ps)
    vel = np.array(vs)
    fs = 1.0 / np.median(np.diff(t))

    # analyse the velocity ring (emphasises the oscillation) after the tap
    mask = t >= args.countdown
    w_d, zeta, npk = analyse(t[mask], vel[mask], args.f_lo, args.f_hi)
    f_d = w_d / (2 * np.pi)
    Q = 1.0 / (2 * zeta) if zeta and not np.isnan(zeta) else float("nan")

    print(f"\nRingdown result ({run.motor}):  sample rate {fs:.0f} Hz, {npk} peaks")
    print(f"  damped natural freq  w_h = {w_d:7.1f} rad/s  ({f_d:5.1f} Hz)")
    print(f"  damping ratio      zeta_h = {zeta:.4f}")
    print(f"  quality factor        Q_h = {Q:.1f}")

    run.save_csv(
        [dict(t=float(a), pos=float(b), vel=float(c))
         for a, b, c in zip(t, pos, vel)], "ringdown.csv")
    run.set_meta(
        duration_s=args.duration, sample_rate_hz=float(fs),
        band_hz=[args.f_lo, args.f_hi], n_peaks=int(npk),
        omega_h_rad_s=float(w_d), omega_h_hz=float(f_d),
        zeta_h=float(zeta), Q_h=float(Q),
    )

    _, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax1.plot(t, pos, lw=0.7)
    ax1.axvline(args.countdown, color="r", ls="--", lw=0.8, label="tap")
    ax1.set_ylabel("position [rev]")
    ax1.set_title(f"Ringdown ({run.motor}, id={args.target}): "
                  f"f_h={f_d:.1f} Hz, zeta={zeta:.3f}")
    ax1.legend(fontsize=8)
    ax1.grid(True)
    ax2.plot(t, vel, lw=0.7)
    ax2.axvline(args.countdown, color="r", ls="--", lw=0.8)
    ax2.set_ylabel("velocity [rev/s]")
    ax2.set_xlabel("time [s]")
    ax2.grid(True)
    plt.tight_layout()
    run.save_fig(plt, "ringdown.png")
    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
