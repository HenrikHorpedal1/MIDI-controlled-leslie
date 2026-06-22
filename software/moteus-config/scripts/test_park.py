#!/usr/bin/env python3
"""Park test: position step-and-hold with reference-based feedforward.

Steps the controller a fixed angle and holds, comparing two conditions:

    no FF  : feedback only
    FF     : feedforward torque computed EXTERNALLY from the reconstructed
             planner reference trajectory (as the ESP32 will do), i.e.
                 T_ff = sign(v_ref)*(c + b|v_ref| + a v_ref^2)   (friction)
                      + J * a_ref                                (inertia)

The moteus internal servo.inertia_feedforward is forced to 0 for both runs —
inertia compensation is done here from the smooth reference, not from moteus'
planner-derived acceleration (which we found destabilising).

The planner trapezoid is reconstructed analytically from the step, --velocity-limit
and --accel-limit, so v_ref / a_ref are smooth.

    uv run python moteus-config/scripts/test_park.py -t 1 \\
        --step 0.25 --c 2.82e-3 --b 2.28e-4 --a 2.70e-5 --J 7.30e-3 \\
        --velocity-limit 1 --accel-limit 2 --no-show
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


def trapezoid_ref(tau, D, v_max, a_max):
    """Reference (p, v, a) at time tau into a trapezoidal move of signed distance D.

    Returns (p_ref, v_ref, a_ref) in rev, rev/s and rev/s² (signed).
    p_ref is integrated position offset from start (0 at tau=0, D when done)."""
    if D == 0 or v_max <= 0 or a_max <= 0:
        return float(D), 0.0, 0.0
    s = math.copysign(1.0, D)
    dist = abs(D)
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc ** 2

    if 2 * d_acc >= dist:                 # triangular (never reaches v_max)
        v_peak = math.sqrt(dist * a_max)
        t_acc = v_peak / a_max
        t_total = 2 * t_acc
        if tau < t_acc:
            p = s * 0.5 * a_max * tau ** 2
            return p, s * a_max * tau, s * a_max
        elif tau < t_total:
            dt = tau - t_acc
            p = s * (d_acc + v_peak * dt - 0.5 * a_max * dt ** 2)
            return p, s * (v_peak - a_max * dt), -s * a_max
        return float(D), 0.0, 0.0
    else:                                  # trapezoid with cruise
        d_cruise = dist - 2 * d_acc
        t_cruise = d_cruise / v_max
        t1 = t_acc
        t2 = t_acc + t_cruise
        t3 = 2 * t_acc + t_cruise
        if tau < t1:
            p = s * 0.5 * a_max * tau ** 2
            return p, s * a_max * tau, s * a_max
        elif tau < t2:
            dt = tau - t1
            p = s * (d_acc + v_max * dt)
            return p, s * v_max, 0.0
        elif tau < t3:
            dt = tau - t2
            p = s * (d_acc + d_cruise + v_max * dt - 0.5 * a_max * dt ** 2)
            return p, s * (v_max - a_max * dt), -s * a_max
        return float(D), 0.0, 0.0


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


async def run_park(controller, stream, qr, ff_mode, moteus_J, args):
    """One step-and-hold.

    ff_mode:
      "off"      feedback only
      "friction" friction compensation only: sign(v_ref)*(c+b|v|+a*v^2)
      "inertia"  inertia compensation only: J*a_ref
      "ref"      full reference FF: friction(v_ref) + J*a_ref
      "state"    feedforward from measured state: friction(v_meas)+J*a_meas_filt
    """
    await stream.command(
        f"conf set servo.inertia_feedforward {moteus_J}".encode())

    await settle_stop(controller)

    r0 = await controller.set_position(position=math.nan, query=True)
    pos0 = r0.values[moteus.Register.POSITION]
    target = pos0 + args.step

    # Moteus internal planner generates the reference trajectory; we reconstruct
    # the same trapezoid analytically to compute aligned feedforward torques.
    kwargs = dict(maximum_torque=args.max_torque, query=True, query_override=qr)
    if args.velocity_limit and args.velocity_limit > 0:
        kwargs["velocity_limit"] = args.velocity_limit
    if args.accel_limit and args.accel_limit > 0:
        kwargs["accel_limit"] = args.accel_limit

    samples = []
    t0 = time.monotonic()
    duration = args.pre + args.hold
    p_ref = pos0
    v_meas_prev = 0.0
    t_prev = None
    a_meas_filt = 0.0       # EMA-filtered measured acceleration [rev/s²]
    while True:
        t = time.monotonic() - t0
        if t > duration:
            break

        if t < args.pre:
            cmd = pos0
            p_ref = pos0
            v_ref = a_ref = 0.0
        else:
            cmd = target
            tau = t - args.pre
            dp, v_ref, a_ref = trapezoid_ref(
                tau, args.step, args.velocity_limit, args.accel_limit)
            p_ref = pos0 + dp

        if ff_mode == "friction":
            T_ff = friction_ff(v_ref, args.c, args.b, args.a)
        elif ff_mode == "inertia":
            T_ff = args.J * a_ref
        elif ff_mode == "ref":
            T_ff = friction_ff(v_ref, args.c, args.b, args.a) + args.J * a_ref
        elif ff_mode == "state":
            T_ff = (friction_ff(v_meas_prev, args.c, args.b, args.a)
                    + args.J * a_meas_filt)
        else:
            T_ff = 0.0

        r = await controller.set_position(
            position=cmd, velocity=0.0, feedforward_torque=T_ff, **kwargs)
        v_meas = r.values[moteus.Register.VELOCITY]

        # update state-based acceleration estimate (filtered finite difference)
        if t_prev is not None:
            dt = t - t_prev
            if dt > 0:
                a_raw = (v_meas - v_meas_prev) / dt
                a_meas_filt = 0.8 * a_meas_filt + 0.2 * a_raw
        v_meas_prev = v_meas
        t_prev = t

        samples.append(dict(
            t=t, p_ref=p_ref, v_ref=v_ref, a_ref=a_ref, T_ff=T_ff,
            pos=r.values[moteus.Register.POSITION],
            vel=v_meas, a_meas=a_meas_filt,
            torque=r.values[moteus.Register.TORQUE]))
        await asyncio.sleep(SAMPLE_DT)

    await controller.set_stop()

    t   = np.array([s["t"]   for s in samples])
    pos = np.array([s["pos"] for s in samples])
    post = t >= args.pre
    err = pos - target
    p_final = float(np.mean(pos[t >= (duration - 0.5)]))
    ss_err  = p_final - target

    if args.step > 0:
        overshoot = (float(np.max(pos[post])) - target) / args.step * 100
    elif args.step < 0:
        overshoot = (target - float(np.min(pos[post]))) / abs(args.step) * 100
    else:
        overshoot = float("nan")

    band = 0.05 * abs(args.step)
    settle_t = float("nan")
    for k in np.where(post)[0]:
        if np.all(np.abs(err[k:]) <= band):
            settle_t = t[k] - args.pre
            break

    hold_jitter = float(np.std(pos[t >= (duration - 1.0)]))

    return samples, dict(target=target, ss_err=ss_err, overshoot=overshoot,
                         settle_t=settle_t, hold_jitter=hold_jitter)


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1)
    parser.add_argument("--step", type=float, default=0.25,
                        help="position step [rev, motor frame] (default 0.25)")
    parser.add_argument("--c", type=float, default=2.82e-3, help="Coulomb [Nm]")
    parser.add_argument("--b", type=float, default=2.28e-4, help="viscous [Nm·s/rev]")
    parser.add_argument("--a", type=float, default=2.70e-5, help="quadratic [Nm·s²/rev²]")
    parser.add_argument("--J", type=float, default=7.30e-3,
                        help="reflected inertia for external inertia FF [Nm·s²/rev]")
    parser.add_argument("--hold", type=float, default=4.0, help="hold time [s]")
    parser.add_argument("--pre", type=float, default=0.5, help="pre-step record [s]")
    parser.add_argument("--velocity-limit", type=float, default=1.0,
                        help="planner velocity limit [rev/s]")
    parser.add_argument("--accel-limit", type=float, default=2.0,
                        help="planner accel limit [rev/s²]")
    parser.add_argument("--max-torque", type=float, default=0.3, help="clamp [Nm]")
    parser.add_argument("--label", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    run = Run("park_test", target=args.target, label=args.label)
    transport = moteus.get_singleton_transport(args)

    qr = moteus.QueryResolution()
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.torque   = moteus.F32

    controller = moteus.Controller(id=args.target, transport=transport,
                                   query_resolution=qr)
    stream = moteus.Stream(controller)
    await stream.write_message(b"tel stop")
    await stream.flush_read()

    inertia_orig = await conf_get_float(stream, "servo.inertia_feedforward")
    print(f"servo.inertia_feedforward original = {inertia_orig} (restored at end)")
    print(f"Park step = {args.step} rev, hold {args.hold}s, "
          f"planner {args.velocity_limit} rev/s @ {args.accel_limit} rev/s²")
    print(f"FF model: c={args.c} b={args.b} a={args.a} J={args.J}\n")

    # Each tuple: (display name, ff_mode, moteus_J)
    #   ff_mode   : external feedforward torque term
    #   moteus_J  : value written to servo.inertia_feedforward before each run
    conditions = [
        ("no FF",         "off",      0.0),
        ("ext friction",  "friction", 0.0),
        ("ext inertia",   "inertia",  0.0),
        ("ext full",      "ref",      0.0),
        ("mot inertia",   "off",      args.J),
        ("mot+friction",  "friction", args.J),
    ]

    results = {}
    try:
        for name, ff_mode, moteus_J in conditions:
            samples, m = await run_park(controller, stream, qr, ff_mode, moteus_J, args)
            results[name] = dict(samples=samples, metrics=m)
            st = (f"{m['settle_t']*1e3:.0f} ms" if math.isfinite(m["settle_t"])
                  else "not settled")
            print(f"  {name:14s}: overshoot {m['overshoot']:6.1f}%  "
                  f"settle {st:>11s}  "
                  f"ss-err {m['ss_err']*360*60:7.1f}'  "
                  f"hold-jitter {m['hold_jitter']*360*60:6.1f}'")
    finally:
        await stream.command(
            f"conf set servo.inertia_feedforward {inertia_orig}".encode())
        await controller.set_stop()
        print(f"\nRestored servo.inertia_feedforward = {inertia_orig}")

    all_rows = []
    for name, res in results.items():
        for s in res["samples"]:
            all_rows.append({**s, "condition": name})
    run.save_csv(all_rows, "park_test.csv")
    run.set_meta(
        step_rev=args.step, hold_s=args.hold,
        c=args.c, b=args.b, a=args.a, J=args.J,
        velocity_limit=args.velocity_limit, accel_limit=args.accel_limit,
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
        "FF state":     "tab:green",   # kept for optional use
    }
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    for name, res in results.items():
        s = res["samples"]
        ax1.plot(arr(s, "t"), arr(s, "pos"), color=colors[name], lw=1, label=name)
        ax2.plot(arr(s, "t"), arr(s, "vel"), color=colors[name], lw=1, label=name)
        ax3.plot(arr(s, "t"), arr(s, "T_ff"), color=colors[name], lw=1, label=name)

    for name, res in results.items():
        s = res["samples"]
        ax1.plot(arr(s, "t"), arr(s, "p_ref"), color=colors[name],
                 lw=0.8, ls="--", alpha=0.5, label=f"{name} ref")
        ax1.axhline(res["metrics"]["target"],
                    color=colors[name], ls=":", lw=0.6, alpha=0.4)
    ax1.set_ylabel("position  [rev]")
    ax1.set_title(f"Park test — {run.motor}, id={args.target}  (step {args.step} rev)")
    ax1.legend(fontsize=8); ax1.grid(True)

    ax2.axhline(0, c="k", lw=0.5)
    ax2.set_ylabel("velocity  [rev/s]"); ax2.legend(fontsize=8); ax2.grid(True)

    ax3.axhline(0, c="k", lw=0.5)
    ax3.set_ylabel("feedforward torque  [Nm]"); ax3.set_xlabel("time  [s]")
    ax3.legend(fontsize=8); ax3.grid(True)

    fig.tight_layout()
    run.save_fig(fig, "park_test.svg")
    run.finish()

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
