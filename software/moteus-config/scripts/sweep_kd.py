#!/usr/bin/env python3
"""Sweep the derivative gain k_d to find the noise-limited stability ceiling.

k_d is the one position-loop gain NOT fixed by the plant model: it is bounded by
velocity-estimate noise, not by the rigid-body dynamics.  Above a hardware ceiling
the differentiated encoder-quantisation noise drives the lightly-damped structural
mode into a limit cycle (see the discussion chapter).  This script locates that
ceiling empirically and recommends a backed-off value.

For each k_d in the grid it sets servo.pid_position.kd (RAM only), runs a
velocity ramp like capture_velocity_ramp.py, and records two metrics:

  * overshoot  -- tracking quality (should pass through a shallow optimum)
  * hf_rms     -- RMS of the velocity above --hf-cut Hz, i.e. the energy in the
                  emerging limit cycle. This rises sharply once k_d passes the
                  ceiling and is the objective stability indicator.

A controller fault (e.g. the limit cycle tripping a power-off) is caught and
marks that k_d unstable.  The recommended k_d is the largest one whose hf_rms is
below --hf-limit, scaled by --backoff.  The original k_d is restored at the end.

    uv run python moteus-config/scripts/sweep_kd.py -t 1 \\
        --kds 0,0.005,0.01,0.015,0.02,0.03 --target-vel 8 --accel-limit 8

SAFETY: this deliberately approaches instability. Keep --max-torque sane and be
ready to power down.
"""

import argparse
import asyncio
import math
import time

import moteus
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run


async def conf_get_float(stream, name):
    for _ in range(5):
        r = await stream.command(f"conf get {name}".encode(), allow_any_response=True)
        try:
            return float(r.decode().strip())
        except (ValueError, AttributeError):
            continue
    raise RuntimeError(f"could not read {name}")


async def run_ramp(controller, qr, target_vel, accel, pre, duration, max_torque):
    """One accel-limited velocity ramp; return (t, vel, tau) arrays."""
    ts, vs, taus = [], [], []
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t > pre + duration:
            break
        target_v = target_vel if t >= pre else 0.0
        r = await controller.set_position(
            position=math.nan, velocity=target_v, accel_limit=accel,
            maximum_torque=max_torque, query=True, query_override=qr)
        ts.append(t)
        vs.append(r.values[moteus.Register.VELOCITY])
        taus.append(r.values[moteus.Register.TORQUE])
    return np.array(ts), np.array(vs), np.array(taus)


def hf_rms(t, v, hf_cut):
    """RMS of the velocity signal above hf_cut Hz (the limit-cycle energy)."""
    if len(t) < 16:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    x = v - np.mean(v)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), dt)
    X[f < hf_cut] = 0.0
    xf = np.fft.irfft(X, n=len(x))
    return float(np.sqrt(np.mean(xf ** 2)))


async def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(p)
    p.add_argument("--target", "-t", type=int, default=1,
                   help="moteus CAN id (1=drum, 2=horn)")
    p.add_argument("--kds", type=str, default="0,0.005,0.01,0.015,0.02,0.03",
                   help="comma-separated k_d grid (motor frame), ascending")
    p.add_argument("--target-vel", type=float, default=8.0,
                   help="ramp target velocity [rev/s]")
    p.add_argument("--accel-limit", type=float, default=8.0,
                   help="planner accel limit [rev/s^2] (below traction limit)")
    p.add_argument("--pre", type=float, default=0.3,
                   help="standstill record time before the ramp [s]")
    p.add_argument("--duration", type=float, default=3.0,
                   help="record time after the velocity command [s]")
    p.add_argument("--hf-cut", type=float, default=30.0,
                   help="limit-cycle band lower edge [Hz] (above the loop bw)")
    p.add_argument("--hf-limit", type=float, default=0.05,
                   help="absolute hf_rms floor [rev/s] (default 0.05)")
    p.add_argument("--hf-scale", type=float, default=3.0,
                   help="instability = hf_rms > max(hf-limit, hf-scale × baseline) "
                        "(default 3.0)")
    p.add_argument("--backoff", type=float, default=0.66,
                   help="recommended k_d = largest stable * backoff (default 0.66)")
    p.add_argument("--max-torque", type=float, default=0.25,
                   help="maximum_torque clamp [Nm] (safety)")
    p.add_argument("--label", default=None,
                   help="optional tag appended to the artifact folder name")
    p.add_argument("--no-show", action="store_true",
                   help="save the plot without opening a window")
    args = p.parse_args()

    kds = [float(x) for x in args.kds.split(",")]

    run = Run("kd_sweep", target=args.target, label=args.label)
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

    kd_orig = await conf_get_float(stream, "servo.pid_position.kd")
    print(f"Original servo.pid_position.kd = {kd_orig} (restored at end)")

    # measure noise baseline at kd=0 to set adaptive threshold
    await controller.set_stop()
    await stream.command(b"conf set servo.pid_position.kd 0")
    await asyncio.sleep(0.2)
    t0b, v0b, _ = await run_ramp(
        controller, qr, args.target_vel, args.accel_limit,
        args.pre, args.duration, args.max_torque)
    post0 = t0b >= args.pre
    hf_baseline = hf_rms(t0b[post0], v0b[post0], args.hf_cut)
    hf_thresh = max(args.hf_limit, hf_baseline * args.hf_scale)
    print(f"Baseline hf_rms (kd=0) = {hf_baseline:.5f} rev/s  "
          f"→ instability threshold = {hf_thresh:.5f} rev/s  "
          f"({args.hf_scale:g}× baseline)")

    rows = []
    prev_hf = hf_baseline
    print(f"\nTarget: moteus id={args.target}  ramp {args.target_vel} rev/s "
          f"@ {args.accel_limit} rev/s^2")
    print(f"{'kd':>8} {'overshoot%':>11} {'hf_rms':>10} {'verdict':>10}")
    print("-" * 42)
    try:
        for kd in kds:
            await controller.set_brake()
            t_stop = time.monotonic()
            while time.monotonic() - t_stop < 5.0:
                r = await controller.set_brake(query=True)
                if abs(r.values[moteus.Register.VELOCITY]) < 0.05:
                    break
                await asyncio.sleep(0.05)
            await controller.set_stop()
            await asyncio.sleep(0.2)
            await stream.command(f"conf set servo.pid_position.kd {kd}".encode())
            await asyncio.sleep(0.2)
            try:
                t, v, _ = await run_ramp(
                    controller, qr, args.target_vel, args.accel_limit,
                    args.pre, args.duration, args.max_torque)
            except Exception as e:                       # fault / power-off
                print(f"{kd:8.4f}  FAULT during ramp ({e}) -- unstable")
                rows.append(dict(kd=kd, overshoot=float("nan"),
                                 hf_rms=float("nan"), stable=False))
                await controller.set_stop()
                continue

            post = t >= args.pre
            tail = t >= (t[-1] - 0.5)
            v_ss = float(np.mean(v[tail])) if tail.any() else float("nan")
            over = ((np.max(v[post]) - args.target_vel) / args.target_vel * 100
                    if args.target_vel else float("nan"))
            hf = hf_rms(t[post], v[post], args.hf_cut)
            # unstable if absolute threshold exceeded OR sharp rise vs previous step
            step_rise = (hf / prev_hf) if (prev_hf > 0 and math.isfinite(hf)) else 1.0
            stable = math.isfinite(hf) and hf < hf_thresh and step_rise < args.hf_scale
            prev_hf = hf if math.isfinite(hf) else prev_hf
            verdict = "ok" if stable else "UNSTABLE"
            print(f"{kd:8.4f} {over:11.2f} {hf:10.5f} {verdict:>10}")
            rows.append(dict(kd=kd, overshoot=float(over), hf_rms=hf,
                             v_ss=v_ss, stable=stable,
                             t_trace=t.tolist(), v_trace=v.tolist()))
            trace_rows = [dict(kd=kd, t=float(ti), v=float(vi))
                          for ti, vi in zip(t, v)]
            run.save_csv(trace_rows, f"trace_kd_{kd:.4f}.csv")
            if not stable:
                print("  → instability detected, stopping sweep")
                break
    finally:
        await stream.command(f"conf set servo.pid_position.kd {kd_orig}".encode())
        await controller.set_stop()
        print(f"\nRestored servo.pid_position.kd = {kd_orig}")

    run.save_csv(rows, "kd_sweep.csv")

    stable_kds = [r["kd"] for r in rows if r["stable"]]
    if stable_kds:
        kd_max = max(stable_kds)
        kd_rec = kd_max * args.backoff
        # also report the overshoot optimum among the stable set
        best = min((r for r in rows if r["stable"]),
                   key=lambda r: abs(r["overshoot"]))
        print(f"\nLargest stable k_d   = {kd_max:.4f}")
        print(f"Overshoot optimum    = {best['kd']:.4f} "
              f"({best['overshoot']:.1f}% overshoot)")
        print(f"Recommended k_d      = {kd_rec:.4f}  ({args.backoff:g}x ceiling)")
    else:
        kd_max = kd_rec = float("nan")
        print("\nNo stable k_d found in the grid -- lower the grid or check setup.")

    run.set_meta(
        kds=kds, target_vel_rev_s=args.target_vel,
        accel_limit_rev_s2=args.accel_limit, hf_cut_hz=args.hf_cut,
        hf_limit_rev_s=args.hf_limit, backoff=args.backoff,
        kd_original=kd_orig, kd_max_stable=kd_max, kd_recommended=kd_rec,
        max_torque_Nm=args.max_torque,
        hf_baseline=float(hf_baseline), hf_threshold=float(hf_thresh),
    )

    n = len(rows)
    cmap = plt.cm.RdYlGn_r(np.linspace(0.0, 0.85, max(n, 1)))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # top: velocity traces, green→red as kd increases; plot high-kd first (background)
    for r, color in zip(reversed(rows), reversed(cmap.tolist())):
        if "t_trace" not in r:
            continue
        t_tr = np.array(r["t_trace"])
        v_tr = np.array(r["v_trace"])
        lw = 1.0 if r["stable"] else 1.5
        ls = "-" if r["stable"] else "--"
        ax1.plot(t_tr, v_tr, color=color, lw=lw, ls=ls,
                 label=f"kd={r['kd']:.3f}" + ("" if r["stable"] else " ✗"))
    ax1.axhline(args.target_vel, c="k", ls=":", lw=0.8, label="target vel")
    ax1.axvline(args.pre, c="grey", ls=":", lw=0.8)
    ax1.set_ylabel("velocity  [rev/s]")
    ax1.set_xlabel("time  [s]")
    ax1.set_title(f"k_d sweep — velocity ramps ({run.motor}, id={args.target})")
    ax1.legend(fontsize=7, ncol=2)
    ax1.grid(True)

    # bottom: hf_rms vs kd
    kk = np.array([r["kd"] for r in rows])
    hh = np.array([r["hf_rms"] for r in rows])
    for r, color in zip(rows, cmap):
        ax2.scatter(r["kd"], r["hf_rms"], color=color, zorder=3, s=50)
    ax2.plot(kk, hh, "-", color="grey", lw=0.8, zorder=2)
    ax2.axhline(hf_thresh, c="grey", ls="--",
                label=f"threshold ({hf_thresh:.3f})")
    if math.isfinite(kd_max):
        ax2.axvline(kd_max, c="green", ls=":", label=f"ceiling kd={kd_max:.3f}")
    ax2.set_ylabel(f"hf_rms  >{args.hf_cut:g} Hz  [rev/s]")
    ax2.set_xlabel("k_d  (motor frame)")
    ax2.legend(fontsize=8)
    ax2.grid(True)

    fig.tight_layout()
    run.save_fig(fig, "kd_sweep.svg")
    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
