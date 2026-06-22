#!/usr/bin/env python3
"""Velocity-ramp tracking test -- the real Leslie operating mode.

A Leslie drum is a velocity device: it spins at a chorale/tremolo speed and ramps
smoothly between speeds, never taking instantaneous position steps.  This commands
a target velocity with a finite acceleration limit and lets MOTEUS'S OWN PLANNER
generate the ramp, so the PID is fed a smoothly advancing reference (position
error stays small, torque stays bounded) and servo.inertia_feedforward acts during
the acceleration phase -- exactly the regime where feedforward is designed to help.

Unlike capture_step.py, the trajectory is NOT injected by this script; we issue one
set_position(velocity=target, accel_limit=...) and record how the measured velocity
tracks the planned ramp.

    uv run python moteus-config/scripts/capture_velocity_ramp.py -t 1 \\
        --target-vel 8 --accel-limit 8

SAFETY: keep --accel-limit below the belt-traction limit (tau = Jbar*alpha must
stay under the slip torque) and be ready to power down.
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
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", "-t", type=int, default=1,
                   help="moteus CAN id (1=drum, 2=horn)")
    p.add_argument("--target-vel", type=float, default=8.0,
                   help="commanded velocity [rev/s]")
    p.add_argument("--accel-limit", type=float, default=8.0,
                   help="planner acceleration limit [rev/s^2] (keep below "
                        "tau_slip/Jbar to avoid belt slip)")
    p.add_argument("--duration", type=float, default=3.0,
                   help="record time after the velocity command [s]")
    p.add_argument("--pre", type=float, default=0.3,
                   help="record time at standstill before the ramp [s]")
    p.add_argument("--max-torque", type=float, default=0.25,
                   help="moteus maximum_torque clamp [Nm] (safety)")
    p.add_argument("--label", default=None,
                   help="optional tag appended to the artifact folder name")
    p.add_argument("--no-show", action="store_true",
                   help="save the plot without opening a window")
    args = p.parse_args()

    run = Run("vramp", target=args.target, label=args.label)

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

    ts, vs, taus, vcmd = [], [], [], []
    t0 = time.monotonic()
    t_cmd = args.pre
    accel = args.accel_limit

    try:
        while True:
            t = time.monotonic() - t0
            if t > t_cmd + args.duration:
                break
            target_v = args.target_vel if t >= t_cmd else 0.0
            # position=nan -> pure velocity command; moteus plans the ramp to
            # target_v at accel_limit and holds there.  No per-cycle reference
            # injection: the planner owns the trajectory.
            r = await controller.set_position(
                position=math.nan, velocity=target_v,
                accel_limit=accel, maximum_torque=args.max_torque,
                query=True, query_override=qr)
            ts.append(t)
            vs.append(r.values[moteus.Register.VELOCITY])
            taus.append(r.values[moteus.Register.TORQUE])
            # ideal accel-limited reference (for the tracking-error overlay)
            dt = max(0.0, t - t_cmd)
            ramp = min(args.target_vel, accel * dt) if t >= t_cmd else 0.0
            vcmd.append(ramp)
    finally:
        await controller.set_stop()

    t = np.array(ts)
    v = np.array(vs)
    tau = np.array(taus)
    vref = np.array(vcmd)

    post = t >= t_cmd
    # steady-state error over the last 0.5 s (assumed settled at target)
    tail = t >= (t[-1] - 0.5)
    v_ss = float(np.mean(v[tail]))
    ss_err = v_ss - args.target_vel
    v_over = (np.max(v[post]) - args.target_vel) / args.target_vel * 100 \
        if args.target_vel else float("nan")
    peak_tau = float(np.abs(tau).max())
    # required inertial torque for this accel, as a slip check
    Jbar = 1.57e-3
    tau_inertia = Jbar * accel * 2 * np.pi

    print(f"Velocity ramp to {args.target_vel} rev/s @ {accel} rev/s^2 "
          f"(id={args.target}, {run.motor})")
    print(f"  steady-state vel   = {v_ss:.3f} rev/s  (err {ss_err:+.3f})")
    print(f"  velocity overshoot = {v_over:.1f} %")
    print(f"  peak torque        = {peak_tau:.3f} Nm")
    print(f"  inertial torque req= {tau_inertia:.3f} Nm  (slip if > traction)")

    run.save_csv(
        [dict(t=float(a), vel=float(b), vel_ref=float(c), torque=float(d))
         for a, b, c, d in zip(t, v, vref, tau)], "vramp.csv")
    run.set_meta(
        target_vel_rev_s=args.target_vel, accel_limit_rev_s2=accel,
        max_torque_Nm=args.max_torque,
        steady_state_vel_rev_s=v_ss, steady_state_err_rev_s=ss_err,
        overshoot_pct=v_over, peak_torque_Nm=peak_tau,
        inertial_torque_req_Nm=float(tau_inertia),
    )

    _, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax1.plot(t, vref, "--", color="grey", lw=1.0, label="planned ramp (ideal)")
    ax1.plot(t, v, ".-", label="measured")
    ax1.axhline(args.target_vel, color="k", lw=0.4)
    ax1.axvline(t_cmd, color="k", lw=0.4)
    ax1.set_ylabel("velocity [rev/s]")
    ax1.set_title(f"Velocity ramp ({run.motor}, id={args.target}): "
                  f"{args.target_vel} rev/s @ {accel} rev/s^2")
    ax1.legend(fontsize=8)
    ax1.grid(True)
    ax2.plot(t, tau, ".-", color="tab:red", label="torque")
    ax2.axvline(t_cmd, color="k", lw=0.4)
    ax2.set_ylabel("torque [Nm]")
    ax2.set_xlabel("time [s]")
    ax2.legend(fontsize=8)
    ax2.grid(True)
    plt.tight_layout()
    run.save_fig(plt, "vramp.png")
    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
