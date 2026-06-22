#!/usr/bin/env python3
"""Stepped-sine frequency response of the drive: T_m -> motor velocity.

Injects an open-loop torque
    T(t) = T_bias + A * sin(2*pi*f*t)
at a sequence of frequencies f, and fits the resulting velocity oscillation at
each f to recover the magnitude and phase of  H(jw) = d(theta_m)/T_m (jw).

This is the experimental validation of the two-mass belt model
(eq. belt-Htheta-m in the thesis):
  * at low frequency  |H| ~ 1/(Jbar * w)   -> reflected inertia Jbar
  * the belt mode appears as the antiresonance (w_a) / resonance (w_1) pair.

The torque is applied with the position/velocity gains scaled to zero
(kp_scale=kd_scale=0) so only the feed-forward torque drives the motor -- i.e.
an open-loop torque command.  A non-zero --bias is required so the rotor turns
at a steady operating point (overcoming Coulomb friction); take it from
identify_friction.py (T_C + b*w0 at the desired operating speed w0).

SAFETY: open-loop torque has no position feedback.  Keep --bias modest, set a
sane --max-torque, and be ready to power down.  The rotor spins continuously
during the sweep.

Run from the software/ uv project:
    uv run python moteus-config/scripts/identify_frf.py -t 1 --bias 0.05 --amp 0.03
"""

import argparse
import asyncio
import math
import time

import moteus
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run

SETTLE_CYCLES = 3      # excitation cycles to discard before capturing
CAPTURE_CYCLES = 6     # excitation cycles to fit over
MIN_SETTLE = 0.5       # s, floor for the settle time at high frequency
MIN_CAPTURE = 1.0      # s, floor for the capture time at high frequency


async def sweep_one(controller, qr, f_hz, amp, max_torque,
                    anchored, hold_velocity, bias,
                    settle_cycles=SETTLE_CYCLES, capture_cycles=CAPTURE_CYCLES):
    """Excite at f_hz, fit the velocity response, return (H complex, n, drift).

    anchored=False: open-loop torque  T = bias + amp*sin  (kp_scale=kd_scale=0).
        Simple, but with low friction the operating speed drifts (slow velocity
        pole), corrupting the fit.
    anchored=True : closed-loop velocity hold at hold_velocity (using the
        configured gains) with amp*sin injected as feed-forward torque on top, so
        the operating speed cannot drift.  The measured d(theta)/T_ff is then
        P/(1+PC); above the velocity-loop bandwidth (~1 Hz here) PC<<1 so it
        equals the open-loop plant P -- which is where the belt resonance lives.
    """
    period = 1.0 / f_hz
    settle = max(MIN_SETTLE, settle_cycles * period)
    capture = max(MIN_CAPTURE, capture_cycles * period)

    async def send(t, query):
        sine = amp * math.sin(2 * math.pi * f_hz * t)
        if anchored:
            # velocity hold (configured gains) + feed-forward sine on top
            return await controller.set_position(
                position=float("nan"), velocity=hold_velocity,
                feedforward_torque=sine,
                maximum_torque=max_torque, query=query, query_override=qr)
        return await controller.set_position(
            position=float("nan"), velocity=float("nan"),
            feedforward_torque=bias + sine, kp_scale=0.0, kd_scale=0.0,
            maximum_torque=max_torque, query=query, query_override=qr)

    t0 = time.monotonic()

    # settle
    while time.monotonic() - t0 < settle:
        await send(time.monotonic() - t0, False)

    # capture
    ts, vs, tqs = [], [], []
    tcap0 = time.monotonic()
    while time.monotonic() - tcap0 < capture:
        t = time.monotonic() - t0
        r = await send(t, True)
        ts.append(t)
        vs.append(r.values[moteus.Register.VELOCITY])
        tqs.append(r.values[moteus.Register.TORQUE])

    ts = np.array(ts)
    vs = np.array(vs)

    # Least-squares fit of velocity to [sin, cos, 1, t] at the known frequency.
    # The 1 and t columns absorb the DC operating speed and any slow drift.
    wt = 2 * math.pi * f_hz * ts
    M = np.column_stack([np.sin(wt), np.cos(wt), np.ones_like(ts), ts])
    coef, *_ = np.linalg.lstsq(M, vs, rcond=None)
    a, b, drift = coef[0], coef[1], coef[3]
    # vs ~ a*sin(wt) + b*cos(wt) = Re{vel_phasor * exp(j*wt)} with
    # vel_phasor = b - j*a.  (The earlier a - j*b gave the right magnitude but a
    # mirrored phase, phase_meas = 90 deg - phase_true; CSVs saved before this
    # fix can be corrected in analyze_frf.py with --fix-phase.)
    vel_phasor = b - 1j * a
    # Input torque phasor is A * sin = Re{(-jA) exp(j wt)}; H = Vel/Torque.
    torque_phasor = -1j * amp
    H = vel_phasor / torque_phasor
    return H, len(ts), float(drift), list(ts), list(vs), list(tqs)


async def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1,
                        help="moteus CAN ID (1=drum, 2=horn)")
    parser.add_argument("--f-min", type=float, default=0.5,
                        help="lowest excitation frequency [Hz]")
    parser.add_argument("--f-max", type=float, default=60.0,
                        help="highest excitation frequency [Hz]")
    parser.add_argument("--points", type=int, default=25,
                        help="log-spaced frequency points")
    parser.add_argument("--bias", type=float, default=0.006,
                        help="open-loop DC bias torque [Nm] (ignored when --anchored)")
    parser.add_argument("--amp", type=float, default=0.03,
                        help="sinusoid torque amplitude [Nm]")
    parser.add_argument("--max-torque", type=float, default=0.5,
                        help="moteus maximum_torque clamp [Nm] (safety)")
    parser.add_argument("--anchored", action="store_true",
                        help="hold a steady speed with the velocity loop and inject "
                             "the sine as feed-forward torque (kills DC drift; use for "
                             "low-friction plants). Valid above the ~1 Hz loop bandwidth.")
    parser.add_argument("--hold-velocity", type=float, default=5.0,
                        help="operating speed for --anchored mode [rev/s]")
    parser.add_argument("--capture-cycles", type=int, default=CAPTURE_CYCLES,
                        help="excitation cycles averaged per point; raise for better "
                             "SNR at the antiresonance (default %(default)s)")
    parser.add_argument("--settle-cycles", type=int, default=SETTLE_CYCLES,
                        help="excitation cycles discarded before capture (default %(default)s)")
    parser.add_argument("--label", default=None,
                        help="optional tag appended to the artifact folder name")
    parser.add_argument("--save-raw", action="store_true",
                        help="save per-sample (t, velocity, torque) for every "
                             "frequency point to frf_raw_samples.csv")
    parser.add_argument("--no-show", action="store_true",
                        help="save the plot without opening a window")
    args = parser.parse_args()

    run = Run("frf", target=args.target, label=args.label)
    transport = moteus.get_singleton_transport(args)
    qr = moteus.QueryResolution()
    qr.velocity = moteus.F32
    qr.torque = moteus.F32
    controller = moteus.Controller(id=args.target, transport=transport,
                                   query_resolution=qr)
    stream = moteus.Stream(controller)

    await stream.write_message(b"tel stop")
    await stream.flush_read()
    await controller.set_stop()
    await asyncio.sleep(0.3)

    freqs = np.logspace(math.log10(args.f_min), math.log10(args.f_max),
                        args.points)

    rows = []
    raw_rows = []
    mode = (f"anchored @ {args.hold_velocity} rev/s" if args.anchored
            else f"open-loop bias={args.bias} Nm")
    print(f"Target: moteus id={args.target}  {mode}  amp={args.amp} Nm")
    print(f"{'f[Hz]':>8} {'|H|[dB]':>10} {'phase[deg]':>12} {'n':>5} {'drift':>10}")
    print("-" * 50)
    try:
        for f in freqs:
            H, n, drift, ts_raw, vs_raw, tqs_raw = await sweep_one(
                controller, qr, float(f),
                args.amp, args.max_torque,
                args.anchored, args.hold_velocity,
                args.bias,
                args.settle_cycles, args.capture_cycles)
            mag_db = 20 * math.log10(abs(H)) if abs(H) > 0 else float("nan")
            phase = math.degrees(np.angle(H))
            print(f"{f:8.3f} {mag_db:10.2f} {phase:12.2f} {n:5d} {drift:10.4f}")
            rows.append(dict(f_hz=f, mag=abs(H), mag_db=mag_db, phase_deg=phase,
                             re=H.real, im=H.imag, n=n, drift=drift))
            if args.save_raw:
                for ti, vi, tqi in zip(ts_raw, vs_raw, tqs_raw):
                    raw_rows.append(dict(f_hz=f, t=ti, velocity=vi, torque=tqi))
    finally:
        await controller.set_stop()

    run.save_csv(rows, "frf.csv")
    if args.save_raw and raw_rows:
        run.save_csv(raw_rows, "frf_raw_samples.csv")
        print(f"  saved {len(raw_rows)} raw samples")
    print(f"\nSaved {len(rows)} points")

    f = np.array([r["f_hz"] for r in rows])
    mag = np.array([r["mag"] for r in rows])
    phase = np.array([r["phase_deg"] for r in rows])

    # Estimate Jbar from the low-frequency 1/(Jbar*w) rolloff: at low f,
    # |H| ~ 1/(Jbar * 2*pi*f)  ->  Jbar ~ 1/(2*pi*f*|H|).  Average the lowest
    # few points where the rigid-body asymptote should hold.
    if args.anchored:
        # The velocity loop suppresses the low-frequency response, so the
        # 1/(Jbar*w) asymptote is not valid here -- take Jbar from CAD or an
        # open-loop low-f sweep instead.
        Jbar = float("nan")
    else:
        lo = np.argsort(f)[:max(3, len(f) // 6)]
        Jbar = float(np.median(1.0 / (2 * math.pi * f[lo] * mag[lo])))
    # Resonance / antiresonance from the magnitude extrema.
    f_res = float(f[np.argmax(mag)])
    f_anti = float(f[np.argmin(mag)])
    if math.isnan(Jbar):
        print("\n  Jbar: not estimated in anchored mode (low-f suppressed by the "
              "velocity loop) -- use CAD or an open-loop low-f sweep")
    else:
        print(f"\n  Jbar (low-f asymptote) ~ {Jbar:.6e}  [Nm/(rev/s^2)]")
    print(f"  magnitude peak (resonance w_1)   ~ {f_res:.2f} Hz "
          f"({2*math.pi*f_res:.1f} rad/s)")
    print(f"  magnitude dip  (antiresonance w_a) ~ {f_anti:.2f} Hz "
          f"({2*math.pi*f_anti:.1f} rad/s)")
    print("  (Use w_1 as the position-loop ceiling in compute_position_gains.py.)")

    run.set_meta(
        n_points=len(rows),
        anchored=args.anchored, hold_velocity_rev_s=args.hold_velocity,
        bias_torque_Nm=args.bias, amp_torque_Nm=args.amp,
        max_torque_Nm=args.max_torque,
        freq_range_hz=[float(args.f_min), float(args.f_max)],
        Jbar_Nm_per_revs2=Jbar,
        resonance_w1_hz=f_res, resonance_w1_rad_s=2 * math.pi * f_res,
        antiresonance_wa_hz=f_anti, antiresonance_wa_rad_s=2 * math.pi * f_anti,
    )

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax1.semilogx(f, 20 * np.log10(mag), "o-")
    if math.isfinite(Jbar):
        rigid = 1.0 / (2 * math.pi * f * Jbar)
        ax1.semilogx(f, 20 * np.log10(rigid), "k--",
                     label=f"rigid-body $1/(\\bar{{J}}\\omega)$, $\\bar{{J}}$={Jbar:.3e}")
    ax1.axvline(f_anti, color="orange", ls="--", lw=1,
                label=f"$\\omega_a$ = {f_anti:.2f} Hz")
    ax1.axvline(f_res, color="green", ls="--", lw=1,
                label=f"$\\omega_1$ = {f_res:.2f} Hz")
    ax1.legend(fontsize=8)
    ax1.set_ylabel(r"$|\dot\theta_m / T_m|$  [dB]")
    ax1.set_title(f"Drive FRF ({run.motor}, moteus id={args.target})")
    ax1.grid(True, which="both")
    ax2.semilogx(f, phase, "o-")
    ax2.axvline(f_anti, color="orange", ls="--", lw=1)
    ax2.axvline(f_res, color="green", ls="--", lw=1)
    ax2.set_ylabel("phase  [deg]")
    ax2.set_xlabel("frequency  [Hz]")
    ax2.grid(True, which="both")
    fig.tight_layout()
    run.save_fig(fig, "frf_bode.svg")
    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
