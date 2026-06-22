#!/usr/bin/env python3
"""Find the acceleration at which the drive belt grossly slips.

The position planner's accel_limit must stay below the point where the inertial
reaction torque (tau = 2*pi*Jbar*alpha) exceeds what the friction belt can
transmit; above it the belt slips grossly and the load no longer follows the
motor.  This ceiling is the hard upper bound on every acceleration curve
(chorale<->tremolo ramps) and is the value quoted as the traction limit in the
thesis.

Method:
  The MA600 on the aux port measures the LOAD angle (ENCODER_1_POSITION) while
  VELOCITY is the motor shaft.  A baseline transmission ratio i0 = wm/wL is
  measured during a gentle ramp.  The commanded accel is then raised step by
  step; for each step the average ratio DURING THE ACCEL PHASE is compared to
  the baseline.  When it rises past --slip-threshold the belt is slipping
  grossly (motor races ahead of the load); that accel is the onset.  The
  recommended accel_limit is the onset scaled by --backoff.

    uv run python moteus-config/scripts/measure_traction_limit.py -t 1 \\
        --v-top 6 --accels 4,6,8,10,15,20,30

SAFETY: this deliberately pushes toward slip.  Keep --max-torque sane, start the
accel grid low, and be ready to power down.
"""

import argparse
import asyncio
import math
import time

import moteus
import moteus.multiplex as mp_res
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run

SAMPLE_PERIOD = 0.005   # s (~200 Hz)


def unwrap_fractional(prev, curr):
    """Delta in revolutions between two fractional [0,1) encoder readings."""
    d = curr - prev
    if d > 0.5:
        d -= 1.0
    elif d < -0.5:
        d += 1.0
    return d


async def accel_pass(controller, qr, v_top, accel, max_torque):
    """Ramp 0 -> v_top at accel; return (ratio_during_accel, wm_mean, peak_tau).

    The ratio is total motor revs / total load revs accumulated while the planner
    is still ramping (measured velocity below 0.9*v_top), i.e. during the accel
    phase where slip shows up.
    """
    # brake and wait until motor has actually stopped
    await controller.set_brake()
    t_stop = time.monotonic()
    while True:
        r = await controller.set_brake(query=True)
        if abs(r.values[moteus.Register.VELOCITY]) < 0.05:
            break
        if time.monotonic() - t_stop > 5.0:
            break
        await asyncio.sleep(0.05)
    await asyncio.sleep(0.2)

    t0 = time.monotonic()
    prev_enc = None
    load_rev = 0.0
    motor_rev = 0.0
    prev_t = None
    wm_samples = []
    peak_tau = 0.0
    while True:
        t = time.monotonic() - t0
        r = await controller.set_position(
            position=math.nan, velocity=v_top, accel_limit=accel,
            maximum_torque=max_torque, query=True, query_override=qr)
        vm = r.values[moteus.Register.VELOCITY]
        enc = r.values[moteus.Register.ENCODER_1_POSITION]
        tau = r.values[moteus.Register.TORQUE]
        if prev_enc is not None and prev_t is not None:
            dt = t - prev_t
            load_rev += unwrap_fractional(prev_enc, enc)
            motor_rev += vm * dt
            if vm < 0.9 * v_top:        # still accelerating
                wm_samples.append(vm)
                peak_tau = max(peak_tau, abs(tau))
        prev_enc, prev_t = enc, t
        if vm >= 0.97 * v_top or t > 2.0 * v_top / accel + 1.5:
            break
        await asyncio.sleep(SAMPLE_PERIOD)

    await controller.set_brake()
    await asyncio.sleep(0.2)
    ratio = motor_rev / load_rev if abs(load_rev) > 1e-4 else float("nan")
    return ratio, float(np.mean(wm_samples)) if wm_samples else float("nan"), peak_tau


async def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(p)
    p.add_argument("--target", "-t", type=int, default=1,
                   help="moteus CAN id (1=drum, 2=horn)")
    p.add_argument("--v-top", type=float, default=6.0,
                   help="top speed of each ramp [rev/s]")
    p.add_argument("--accels", type=str, default=None,
                   help="comma-separated accel grid [rev/s^2] (overrides auto sweep)")
    p.add_argument("--min-accel", type=float, default=1.0,
                   help="starting accel for auto sweep [rev/s^2] (default 1.0)")
    p.add_argument("--accel-scale", type=float, default=1.4,
                   help="accel multiplier per step in auto sweep (default 1.4)")
    p.add_argument("--max-accel", type=float, default=60.0,
                   help="safety ceiling for auto sweep [rev/s^2] (default 60)")
    p.add_argument("--slip-threshold", type=float, default=0.05,
                   help="fractional ratio rise vs baseline that counts as gross "
                        "slip (default 0.05 = 5%%)")
    p.add_argument("--backoff", type=float, default=0.5,
                   help="recommended accel_limit = onset * backoff (default 0.5)")
    p.add_argument("--max-torque", type=float, default=0.5,
                   help="maximum_torque clamp [Nm] (safety)")
    p.add_argument("--label", default=None,
                   help="optional tag appended to the artifact folder name")
    p.add_argument("--no-show", action="store_true",
                   help="save the plot without opening a window")
    args = p.parse_args()

    if args.accels is not None:
        accels = sorted(float(x) for x in args.accels.split(","))
    else:
        # auto-generate: multiply by accel_scale until max_accel
        accels = []
        a = args.min_accel
        while a <= args.max_accel:
            accels.append(round(a, 3))
            a *= args.accel_scale

    run = Run("traction", target=args.target, label=args.label)
    transport = moteus.get_singleton_transport(args)
    qr = moteus.QueryResolution()
    qr.velocity = moteus.F32
    qr.torque = moteus.F32
    qr._extra[moteus.Register.ENCODER_1_POSITION] = mp_res.F32
    controller = moteus.Controller(id=args.target, transport=transport,
                                   query_resolution=qr)
    stream = moteus.Stream(controller)
    await stream.write_message(b"tel stop")
    await stream.flush_read()
    await controller.set_stop()
    await asyncio.sleep(0.3)

    rows = []
    baseline = None
    onset = None
    print(f"Target: moteus id={args.target}  v_top={args.v_top} rev/s")
    print(f"{'accel':>8} {'ratio':>10} {'slip%':>9} {'wm':>8} {'peak_tau':>10}")
    print("-" * 50)
    try:
        for a in accels:
            ratio, wm, peak_tau = await accel_pass(
                controller, qr, args.v_top, a, args.max_torque)
            if not math.isfinite(ratio):
                print(f"{a:8.2f}  load encoder not moving -- skip")
                continue
            if baseline is None:
                baseline = ratio          # lowest accel = no-slip reference
            slip = ratio / baseline - 1.0
            flag = "  <-- GROSS SLIP" if slip > args.slip_threshold else ""
            print(f"{a:8.2f} {ratio:10.5f} {slip*100:8.2f}% {wm:8.3f} "
                  f"{peak_tau:10.4f}{flag}")
            rows.append(dict(accel=a, ratio=ratio, slip=slip, wm=wm,
                             peak_tau=peak_tau))
            if onset is None and slip > args.slip_threshold:
                onset = a
    finally:
        await controller.set_stop()

    if not rows:
        print("No data collected.")
        return

    run.save_csv(rows, "traction.csv")

    if onset is None:
        print(f"\nNo gross slip up to {accels[-1]} rev/s^2 -- raise --accels to "
              f"find the ceiling. Using the top accel as a lower bound.")
        rec = accels[-1] * args.backoff
    else:
        rec = onset * args.backoff
        print(f"\nGross-slip onset      = {onset:.2f} rev/s^2")
    tau_slip = 2 * math.pi * 7.30e-3 * (onset if onset else accels[-1])
    print(f"Recommended accel_limit = {rec:.2f} rev/s^2  ({args.backoff:g}x onset)")
    print(f"  (corresponding inertial torque ~ {tau_slip:.3f} Nm at Jbar=7.30e-3)")

    run.set_meta(
        v_top_rev_s=args.v_top, accels_rev_s2=accels,
        slip_threshold=args.slip_threshold, backoff=args.backoff,
        baseline_ratio=baseline, gross_slip_onset_rev_s2=onset,
        recommended_accel_limit_rev_s2=rec,
        max_torque_Nm=args.max_torque,
    )

    a_arr = np.array([r["accel"] for r in rows])
    slip = np.array([r["slip"] for r in rows]) * 100
    plt.figure(figsize=(7, 5))
    plt.plot(a_arr, slip, "o-", zorder=3)
    plt.axhline(args.slip_threshold * 100, c="r", ls="--",
                label=f"gross-slip threshold ({args.slip_threshold*100:g}%)")
    if onset:
        plt.axvline(onset, c="grey", ls=":", label=f"onset {onset:.1f} rev/s^2")
        plt.axvline(rec, c="g", ls=":", label=f"recommended {rec:.1f} rev/s^2")
    plt.xlabel("commanded acceleration  [rev/s^2]")
    plt.ylabel("ratio rise vs baseline  [%]  (gross slip)")
    plt.title(f"Belt traction limit ({run.motor}, id={args.target})")
    plt.legend(fontsize=8)
    plt.grid(True)
    plt.tight_layout()
    run.save_fig(plt, "traction.svg")
    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
