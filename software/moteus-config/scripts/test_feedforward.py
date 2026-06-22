#!/usr/bin/env python3
"""Test the computed-torque feedforward terms: inertia (moteus) + friction (external).

There are two feedforward channels:

  * inertia  J_bar * a_ref   handled INSIDE moteus by servo.inertia_feedforward,
                             which multiplies the planner's commanded acceleration.
                             It can ONLY act while the trajectory is accelerating.
  * friction sign(w)*(c+b|w|+a w^2)  sent by this script as feedforward_torque.

To make the inertia channel observable the profile must be acceleration-dominated,
so this script commands TRIANGULAR position moves (ramp up to v_top then straight
back down, no constant-velocity cruise) and lets moteus plan them so its commanded
acceleration is clean.  It sweeps four conditions, toggling servo.inertia_feedforward
in RAM between them:

    baseline : inertia_ff = 0,            friction off
    inertia  : inertia_ff = --inertia,    friction off
    friction : inertia_ff = 0,            friction on
    both     : inertia_ff = --inertia,    friction on

The original servo.inertia_feedforward is restored at the end.  Metric is the RMS
velocity-tracking error over the (all-ramp) moves.

    uv run python moteus-config/scripts/test_feedforward.py -t 1 \\
        --c 2.82e-3 --b 2.28e-4 --a 2.70e-5 --inertia 9.82e-3 \\
        --v-top 8 --accel 6 --cycles 4 --no-show
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


def friction_ff(v, c, b, a):
    """Friction feedforward torque [Nm] for velocity v [rev/s]."""
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
    """Brake and wait until the motor has actually stopped."""
    await controller.set_brake()
    t = time.monotonic()
    while time.monotonic() - t < 4.0:
        r = await controller.set_brake(query=True)
        if abs(r.values[moteus.Register.VELOCITY]) < 0.05:
            break
        await asyncio.sleep(0.05)
    await controller.set_stop()
    await asyncio.sleep(0.2)


async def triangular_move(controller, qr, direction, v_top, accel,
                          c, b, a, use_friction, max_torque, t_offset):
    """One triangular move (rest → v_top → rest) of distance v_top^2/accel.

    moteus plans the trapezoid; with the target distance set to exactly the
    triangular distance there is no cruise, so the whole move is ramp.  Returns
    samples with the reconstructed reference velocity (signed)."""
    d = direction * v_top ** 2 / accel
    r0 = await controller.set_position(position=math.nan, query=True)
    pos0 = r0.values[moteus.Register.POSITION]
    target = pos0 + d

    t_peak = v_top / accel
    t_total = 2 * t_peak + 0.3      # small tail to capture settling

    samples = []
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t > t_total:
            break

        # reconstruct the planned reference velocity (signed)
        if t < t_peak:
            v_ref = direction * accel * t
        elif t < 2 * t_peak:
            v_ref = direction * (v_top - accel * (t - t_peak))
        else:
            v_ref = 0.0

        v_now = samples[-1]["v_meas"] if samples else 0.0
        T_ff = friction_ff(v_now, c, b, a) if use_friction else 0.0

        r = await controller.set_position(
            position=target, velocity_limit=v_top, accel_limit=accel,
            maximum_torque=max_torque, feedforward_torque=T_ff,
            query=True, query_override=qr)

        v_meas = r.values[moteus.Register.VELOCITY]
        tau    = r.values[moteus.Register.TORQUE]
        samples.append(dict(t=t_offset + t, v_ref=v_ref, v_meas=v_meas,
                            v_err=v_meas - v_ref, T_ff=T_ff, torque=tau))
        await asyncio.sleep(SAMPLE_DT)

    return samples


async def run_condition(controller, qr, stream, name, inertia_ff, use_friction,
                        v_top, accel, cycles, c, b, a, max_torque):
    """Set inertia_ff in RAM, run `cycles` back-and-forth triangular moves."""
    await settle_stop(controller)
    await stream.command(f"conf set servo.inertia_feedforward {inertia_ff}".encode())
    await asyncio.sleep(0.2)

    samples = []
    t_off = 0.0
    for i in range(cycles):
        direction = 1.0 if (i % 2 == 0) else -1.0
        seg = await triangular_move(
            controller, qr, direction, v_top, accel,
            c, b, a, use_friction, max_torque, t_off)
        samples.extend(seg)
        t_off = seg[-1]["t"] + SAMPLE_DT
        await settle_stop(controller)

    rms = float(np.std([s["v_err"] for s in samples]))
    print(f"  {name:9s}: inertia_ff={inertia_ff:.4g}  friction={'on' if use_friction else 'off':3s}"
          f"  →  RMS error = {rms:.4f} rev/s  ({len(samples)} samples)")
    return samples, rms


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1)
    parser.add_argument("--c", type=float, default=2.82e-3, help="Coulomb [Nm]")
    parser.add_argument("--b", type=float, default=2.28e-4, help="viscous [Nm·s/rev]")
    parser.add_argument("--a", type=float, default=2.70e-5, help="quadratic [Nm·s²/rev²]")
    parser.add_argument("--inertia", type=float, default=9.82e-3,
                        help="servo.inertia_feedforward to test [Nm·s²/rev]")
    parser.add_argument("--v-top", type=float, default=8.0, help="peak velocity [rev/s]")
    parser.add_argument("--accel", type=float, default=6.0, help="accel limit [rev/s²]")
    parser.add_argument("--cycles", type=int, default=4,
                        help="back-and-forth triangular moves per condition")
    parser.add_argument("--max-torque", type=float, default=0.3, help="clamp [Nm]")
    parser.add_argument("--label", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    run = Run("feedforward_test", target=args.target, label=args.label)
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
    print(f"Original servo.inertia_feedforward = {inertia_orig} (restored at end)")
    print(f"Triangular moves: v_top={args.v_top} rev/s, accel={args.accel} rev/s², "
          f"{args.cycles} cycles/condition\n")

    conditions = [
        ("baseline", 0.0,           False),
        ("inertia",  args.inertia,  False),
        ("friction", 0.0,           True),
        ("both",     args.inertia,  True),
    ]

    results = {}
    try:
        for name, iff, use_fric in conditions:
            samples, rms = await run_condition(
                controller, qr, stream, name, iff, use_fric,
                args.v_top, args.accel, args.cycles,
                args.c, args.b, args.a, args.max_torque)
            results[name] = dict(samples=samples, rms=rms,
                                 inertia_ff=iff, friction=use_fric)
    finally:
        await stream.command(
            f"conf set servo.inertia_feedforward {inertia_orig}".encode())
        await controller.set_stop()
        print(f"\nRestored servo.inertia_feedforward = {inertia_orig}")

    # ---- save ----
    all_rows = []
    for name, res in results.items():
        for s in res["samples"]:
            all_rows.append({**s, "condition": name})
    run.save_csv(all_rows, "feedforward_test.csv")

    run.set_meta(
        c=args.c, b=args.b, a=args.a, inertia_ff_tested=args.inertia,
        inertia_ff_original=inertia_orig,
        v_top=args.v_top, accel=args.accel, cycles=args.cycles,
        rms={k: v["rms"] for k, v in results.items()},
    )

    # ---- plot ----
    def arr(samples, key):
        return np.array([s[key] for s in samples])

    colors = {"baseline": "tab:gray", "inertia": "tab:orange",
              "friction": "tab:green", "both": "tab:red"}

    fig, axes = plt.subplots(3, 1, figsize=(11, 10))

    # top: velocity error vs time, all conditions
    ax = axes[0]
    for name, res in results.items():
        s = res["samples"]
        ax.plot(arr(s, "t"), arr(s, "v_err"), lw=0.9, color=colors[name],
                label=f"{name} (RMS {res['rms']:.3f})")
    ax.axhline(0, c="k", lw=0.5)
    ax.set_ylabel("velocity error  [rev/s]")
    ax.set_xlabel("time  [s]")
    ax.set_title(f"Feedforward test (triangular moves) — {run.motor}, id={args.target}")
    ax.legend(fontsize=8)
    ax.grid(True)

    # middle: reference + measured velocity for the 'both' condition
    ax = axes[1]
    s = results["both"]["samples"]
    ax.plot(arr(s, "t"), arr(s, "v_ref"),  "k:", lw=1, label="reference")
    ax.plot(arr(s, "t"), arr(s, "v_meas"), color="tab:red", lw=0.9, label="measured (both)")
    ax.set_ylabel("velocity  [rev/s]")
    ax.set_xlabel("time  [s]")
    ax.legend(fontsize=8)
    ax.grid(True)

    # bottom: RMS bar chart
    ax = axes[2]
    names = list(results.keys())
    rms_vals = [results[n]["rms"] for n in names]
    bars = ax.bar(names, rms_vals, color=[colors[n] for n in names])
    for bar, v in zip(bars, rms_vals):
        ax.text(bar.get_x() + bar.get_width() / 2, v, f"{v:.3f}",
                ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("RMS velocity error  [rev/s]")
    ax.set_title("Tracking error per feedforward configuration")
    ax.grid(True, axis="y")

    fig.tight_layout()
    run.save_fig(fig, "feedforward_test.svg")
    run.finish()

    print("\nSummary (RMS velocity error, rev/s):")
    base = results["baseline"]["rms"]
    for n in names:
        delta = (base - results[n]["rms"]) / base * 100
        print(f"  {n:9s} = {results[n]['rms']:.4f}   ({delta:+.1f}% vs baseline)")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
