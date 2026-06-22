#!/usr/bin/env python3
"""Velocity ramp test: compare feedforward conditions during a velocity ramp.

Ramps the motor from rest to --v-target at --accel-limit, holds for --hold
seconds, then decelerates back to rest.  Six conditions are tested in sequence:

    no FF          : feedback only
    ext friction   : friction(v_ref) externally via feedforward_torque
    ext inertia    : J*a_ref externally via feedforward_torque
    ext full       : friction(v_ref) + J*a_ref externally
    mot inertia    : servo.inertia_feedforward = J, no external FF
    mot+friction   : servo.inertia_feedforward = J, external friction(v_ref)

In velocity mode (position=NaN) moteus ramps control_velocity linearly at
accel_limit each servo cycle, so control_acceleration is a clean constant
during the ramp — unlike position mode where discrete 200 Hz commands make it
noisy.  This makes it worth testing whether the moteus internal inertia FF
works correctly here.

The reference ramp is reconstructed analytically:
    v_ref(tau) = clamp(a_ramp * tau, 0, v_target)   during accel
    v_ref(tau) = v_target                             during cruise
    v_ref(tau) = clamp(v_target - a_ramp*(tau-t2),0,v_target)  during decel
    a_ref      = ±a_ramp during ramp phases, 0 during cruise/hold

    uv run python moteus-config/scripts/test_velocity_ramp.py -t 1 \\
        --v-target 1.0 --accel-limit 2.0 --hold 2.0 \\
        --c 2.82e-3 --b 2.28e-4 --a 2.70e-5 --J 7.30e-3 --no-show
"""

import argparse
import asyncio
import math
import time

import moteus
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run

SAMPLE_DT = 0.005   # 200 Hz


def ramp_ref(tau, v_target, a_max, hold, decel):
    """Reference (v_ref, a_ref) at time tau into the ramp profile.

    Profile: accel from 0 → v_target, cruise for hold seconds,
    optionally decel back to 0 if decel=True.

    Returns (v_ref, a_ref) in rev/s and rev/s².
    """
    if v_target == 0 or a_max <= 0:
        return 0.0, 0.0
    t_acc = v_target / a_max
    t1 = t_acc
    t2 = t_acc + hold
    t3 = t_acc + hold + t_acc  # decel complete

    if tau < 0:
        return 0.0, 0.0
    elif tau < t1:
        return a_max * tau, a_max
    elif tau < t2:
        return v_target, 0.0
    elif decel and tau < t3:
        return max(0.0, v_target - a_max * (tau - t2)), -a_max
    else:
        return 0.0, 0.0


def friction_ff(v, c, b, a):
    if abs(v) < 1e-4:
        return 0.0
    return math.copysign(c + b * abs(v) + a * v ** 2, v)


async def conf_get_float(stream, name):
    for _ in range(5):
        r = await stream.command(f"conf get {name}".encode(), allow_any_response=True)
        try:
            return float(r.decode().strip())
        except (ValueError, AttributeError):
            continue
    raise RuntimeError(f"could not read {name}")


async def settle_stop(controller):
    await controller.set_brake()
    t = time.monotonic()
    while time.monotonic() - t < 4.0:
        r = await controller.set_brake(query=True)
        if abs(r.values[moteus.Register.VELOCITY]) < 0.05:
            break
        await asyncio.sleep(0.05)
    await controller.set_stop()
    await asyncio.sleep(0.2)


async def run_ramp(controller, stream, qr, ff_mode, moteus_J, args):
    """One ramp-hold-(decel) run.

    ff_mode:
      "off"      feedback only
      "friction" external friction(v_ref)
      "inertia"  external J*a_ref
      "ref"      external friction(v_ref) + J*a_ref
    moteus_J:
      value written to servo.inertia_feedforward before this run (0 or args.J)
    """
    await stream.command(
        f"conf set servo.inertia_feedforward {moteus_J}".encode())

    await settle_stop(controller)

    kwargs = dict(maximum_torque=args.max_torque, query=True, query_override=qr,
                  accel_limit=args.accel_limit)

    t_acc  = args.v_target / args.accel_limit
    duration = args.pre + t_acc + args.hold + (t_acc if args.decel else 0) + args.post

    samples = []
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t > duration:
            break

        tau = t - args.pre
        if tau < 0:
            v_ref, a_ref = 0.0, 0.0
            v_cmd = 0.0
        else:
            v_ref, a_ref = ramp_ref(tau, args.v_target, args.accel_limit,
                                    args.hold, args.decel)
            v_cmd = v_ref

        if ff_mode == "friction":
            T_ff = friction_ff(v_ref, args.c, args.b, args.a)
        elif ff_mode == "inertia":
            T_ff = args.J * a_ref
        elif ff_mode == "ref":
            T_ff = friction_ff(v_ref, args.c, args.b, args.a) + args.J * a_ref
        else:
            T_ff = 0.0

        r = await controller.set_position(
            position=math.nan, velocity=v_cmd,
            feedforward_torque=T_ff, **kwargs)

        samples.append(dict(
            t=t, tau=tau, v_ref=v_ref, a_ref=a_ref, T_ff=T_ff,
            vel=r.values[moteus.Register.VELOCITY],
            torque=r.values[moteus.Register.TORQUE]))

        await asyncio.sleep(SAMPLE_DT)

    await controller.set_stop()

    t    = np.array([s["t"]     for s in samples])
    vel  = np.array([s["vel"]   for s in samples])
    vref = np.array([s["v_ref"] for s in samples])

    # Metrics over cruise phase only
    cruise_mask = (np.array([s["tau"] for s in samples]) >=
                   args.v_target / args.accel_limit) & \
                  (np.array([s["tau"] for s in samples]) <=
                   args.v_target / args.accel_limit + args.hold)

    err = vel - vref
    rms_err   = float(np.sqrt(np.mean(err ** 2)))
    rms_cruise = float(np.sqrt(np.mean(err[cruise_mask] ** 2))) if cruise_mask.any() else float("nan")
    peak_err  = float(np.max(np.abs(err)))

    return samples, dict(rms_err=rms_err, rms_cruise=rms_cruise, peak_err=peak_err)


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1)
    parser.add_argument("--v-target", type=float, default=1.0,
                        help="target velocity [rev/s] (default 1.0)")
    parser.add_argument("--accel-limit", type=float, default=2.0,
                        help="ramp acceleration [rev/s²] (default 2.0)")
    parser.add_argument("--hold", type=float, default=2.0,
                        help="cruise hold time [s] (default 2.0)")
    parser.add_argument("--decel", action="store_true",
                        help="decelerate back to 0 after hold")
    parser.add_argument("--pre", type=float, default=0.5,
                        help="pre-ramp record time [s] (default 0.5)")
    parser.add_argument("--post", type=float, default=0.5,
                        help="post-ramp record time [s] (default 0.5)")
    parser.add_argument("--c", type=float, default=2.82e-3, help="Coulomb [Nm]")
    parser.add_argument("--b", type=float, default=2.28e-4, help="viscous [Nm·s/rev]")
    parser.add_argument("--a", type=float, default=2.70e-5, help="quadratic [Nm·s²/rev²]")
    parser.add_argument("--J", type=float, default=7.30e-3,
                        help="reflected inertia [Nm·s²/rev]")
    parser.add_argument("--max-torque", type=float, default=0.3, help="clamp [Nm]")
    parser.add_argument("--label", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    run = Run("velocity_ramp_test", target=args.target, label=args.label)
    transport = moteus.get_singleton_transport(args)

    qr = moteus.QueryResolution()
    qr.velocity = moteus.F32
    qr.torque   = moteus.F32

    controller = moteus.Controller(id=args.target, transport=transport,
                                   query_resolution=qr)
    stream = moteus.Stream(controller)
    await stream.write_message(b"tel stop")
    await stream.flush_read()

    inertia_orig = await conf_get_float(stream, "servo.inertia_feedforward")
    t_acc = args.v_target / args.accel_limit
    print(f"servo.inertia_feedforward original = {inertia_orig} (restored at end)")
    print(f"Ramp: 0 → {args.v_target} rev/s @ {args.accel_limit} rev/s²  "
          f"(t_acc={t_acc:.2f}s, cruise={args.hold}s)")
    print(f"FF model: c={args.c} b={args.b} a={args.a} J={args.J}\n")

    conditions = [
        ("no FF",        "off",      0.0),
        ("ext friction", "friction", 0.0),
        ("ext inertia",  "inertia",  0.0),
        ("ext full",     "ref",      0.0),
        ("mot inertia",  "off",      args.J),
        ("mot+friction", "friction", args.J),
    ]

    results = {}
    try:
        for name, ff_mode, moteus_J in conditions:
            samples, m = await run_ramp(
                controller, stream, qr, ff_mode, moteus_J, args)
            results[name] = dict(samples=samples, metrics=m)
            print(f"  {name:14s}: rms_err {m['rms_err']*60:6.2f}'/s  "
                  f"cruise_rms {m['rms_cruise']*60:6.2f}'/s  "
                  f"peak_err {m['peak_err']*60:6.2f}'/s")
    finally:
        await stream.command(
            f"conf set servo.inertia_feedforward {inertia_orig}".encode())
        await controller.set_stop()
        print(f"\nRestored servo.inertia_feedforward = {inertia_orig}")

    all_rows = []
    for name, res in results.items():
        for s in res["samples"]:
            all_rows.append({**s, "condition": name})
    run.save_csv(all_rows, "velocity_ramp_test.csv")
    run.set_meta(
        v_target=args.v_target, accel_limit=args.accel_limit,
        hold_s=args.hold, decel=args.decel,
        c=args.c, b=args.b, a=args.a, J=args.J,
        metrics={k: v["metrics"] for k, v in results.items()},
    )

    # ---- plot ----
    def arr(samples, key):
        return np.array([s[key] for s in samples])

    colors = {
        "no FF":        "tab:blue",
        "ext friction": "tab:orange",
        "ext inertia":  "tab:purple",
        "ext full":     "tab:red",
        "mot inertia":  "tab:brown",
        "mot+friction": "tab:pink",
    }
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    for name, res in results.items():
        s = res["samples"]
        ax1.plot(arr(s, "t"), arr(s, "vel"),   color=colors[name], lw=1, label=name)
        ax2.plot(arr(s, "t"), arr(s, "T_ff"),  color=colors[name], lw=1, label=name)

    # reference from first condition
    s0 = results["no FF"]["samples"]
    ax1.plot(arr(s0, "t"), arr(s0, "v_ref"), "k:", lw=1, label="v_ref")
    ax1.axhline(args.v_target, color="grey", ls="--", lw=0.8, label="target")

    ax1.set_ylabel("velocity  [rev/s]")
    ax1.set_title(f"Velocity ramp test — {run.motor}, id={args.target}  "
                  f"(0→{args.v_target} rev/s @ {args.accel_limit} rev/s²)")
    ax1.legend(fontsize=8); ax1.grid(True)

    ax2.axhline(0, c="k", lw=0.5)
    ax2.set_ylabel("feedforward torque  [Nm]")
    ax2.set_xlabel("time  [s]")
    ax2.legend(fontsize=8); ax2.grid(True)

    fig.tight_layout()
    run.save_fig(fig, "velocity_ramp_test.svg")
    run.finish()

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
