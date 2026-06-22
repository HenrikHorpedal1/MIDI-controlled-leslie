#!/usr/bin/env python3
"""Closed-loop position step response of the drive.

Commands a position step and records position + velocity vs time, so the tuned
loop can be evaluated (rise time, overshoot, settling) and the model-based gains
compared against the hand tune.

The trajectory planner is bypassed by commanding very high velocity/accel limits,
so the controller sees a near-instant position reference change -- i.e. the
response is that of the closed PID loop, not the planner.

    uv run python moteus-config/scripts/capture_step.py -t 1 --step 0.25

SAFETY: the rotor jumps to the new setpoint.  Keep --step modest and --max-torque
sane, and be ready to power down.
"""

import argparse
import asyncio
import math
import time

import moteus
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", "-t", type=int, default=1,
                        help="moteus CAN id (1=drum, 2=horn)")
    parser.add_argument("--step", type=float, default=0.25,
                        help="position step size [output rev]")
    parser.add_argument("--duration", type=float, default=2.5,
                        help="record time after the step [s]")
    parser.add_argument("--pre", type=float, default=0.3,
                        help="settle/record time before the step [s]")
    parser.add_argument("--hold-velocity", type=float, default=0.0,
                        help="if non-zero, advance the position reference at this "
                             "speed [rev/s] so the rotor never stops -- keeps the "
                             "step out of the stiction/cogging low-speed zone "
                             "(moving-step mode). 0 = standstill step.")
    parser.add_argument("--max-torque", type=float, default=0.6,
                        help="moteus maximum_torque clamp [Nm] (safety)")
    parser.add_argument("--vel-limit", type=float, default=500.0,
                        help="planner velocity limit during the step [rev/s] (high = step)")
    parser.add_argument("--accel-limit", type=float, default=5000.0,
                        help="planner accel limit during the step [rev/s^2] (high = step)")
    parser.add_argument("--label", default=None,
                        help="optional tag appended to the artifact folder name")
    parser.add_argument("--no-show", action="store_true",
                        help="save the plot without opening a window")
    args = parser.parse_args()

    run = Run("step", target=args.target, label=args.label)

    qr = moteus.QueryResolution()
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.torque = moteus.F32
    controller = moteus.Controller(id=args.target, query_resolution=qr)
    stream = moteus.Stream(controller)

    await stream.write_message(b"tel stop")
    await stream.flush_read()
    await controller.set_stop()
    await asyncio.sleep(0.2)

    # current position becomes the start reference; hold/track it, then step.
    hv = args.hold_velocity
    r0 = await controller.set_position(position=math.nan, velocity=0.0,
                                       maximum_torque=args.max_torque, query=True)
    p_start = r0.values[moteus.Register.POSITION]

    ts, ps, vs, taus = [], [], [], []
    t0 = time.monotonic()
    t_step = args.pre

    async def cmd(pos, query):
        return await controller.set_position(
            position=pos, velocity=hv,
            velocity_limit=args.vel_limit, accel_limit=args.accel_limit,
            maximum_torque=args.max_torque, query=query, query_override=qr)

    try:
        while True:
            t = time.monotonic() - t0
            if t > t_step + args.duration:
                break
            # reference advances at hv (0 = standstill); add the step at t_step
            ref = p_start + hv * t + (args.step if t >= t_step else 0.0)
            r = await cmd(ref, True)
            ts.append(t)
            ps.append(r.values[moteus.Register.POSITION])
            vs.append(r.values[moteus.Register.VELOCITY])
            taus.append(r.values[moteus.Register.TORQUE])
    finally:
        await controller.set_stop()

    ts = np.array(ts)
    # subtract the moving ramp so the step response is 0 -> step regardless of hv
    t = ts - t_step                    # t=0 at the step
    p = np.array(ps) - p_start - hv * ts
    v = np.array(vs)
    tau = np.array(taus)

    # step-response metrics over the post-step window
    post = t >= 0
    tp, pp = t[post], p[post]
    final = args.step
    overshoot = (pp.max() - final) / final * 100 if final else float("nan")
    # 2% settling time
    band = 0.02 * abs(final)
    settled = np.where(np.abs(pp - final) > band)[0]
    t_settle = tp[settled[-1]] if len(settled) else 0.0
    # 10-90% rise time
    try:
        t10 = tp[np.where(pp >= 0.1 * final)[0][0]]
        t90 = tp[np.where(pp >= 0.9 * final)[0][0]]
        t_rise = t90 - t10
    except IndexError:
        t_rise = float("nan")

    print(f"Step {args.step} rev on moteus id={args.target} ({run.motor})")
    print(f"  rise time (10-90%) = {t_rise*1e3:.0f} ms")
    print(f"  overshoot          = {overshoot:.1f} %")
    print(f"  settling (2%)      = {t_settle*1e3:.0f} ms")
    print(f"  peak torque        = {np.abs(tau).max():.3f} Nm")

    run.save_csv(
        [dict(t=float(a), pos=float(b), vel=float(c), torque=float(d))
         for a, b, c, d in zip(t, p, v, tau)], "step.csv")
    run.set_meta(
        step_rev=args.step, hold_velocity_rev_s=hv, max_torque_Nm=args.max_torque,
        vel_limit_rev_s=args.vel_limit, accel_limit_rev_s2=args.accel_limit,
        rise_time_ms=t_rise * 1e3, overshoot_pct=overshoot,
        settling_2pct_ms=t_settle * 1e3, peak_torque_Nm=float(np.abs(tau).max()),
    )

    _, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax1.axhline(final, color="grey", ls="--", lw=0.8, label="setpoint")
    ax1.plot(t, p, ".-")
    ax1.axvline(0, color="k", lw=0.4)
    ax1.set_ylabel("position  [rev]")
    ax1.set_title(f"Step response ({run.motor}, moteus id={args.target}, "
                  f"step={args.step} rev)")
    ax1.legend(fontsize=8)
    ax1.grid(True)
    ax2.plot(t, v, ".-", label="velocity")
    ax2.plot(t, tau, ".-", label="torque")
    ax2.axvline(0, color="k", lw=0.4)
    ax2.set_ylabel("vel [rev/s] / torque [Nm]")
    ax2.set_xlabel("time  [s]")
    ax2.legend(fontsize=8)
    ax2.grid(True)
    plt.tight_layout()
    run.save_fig(plt, "step.png")
    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
