#!/usr/bin/env python3
"""Measure the breakaway (stiction) torque of the drive.

Stiction is the torque at which a rotor at rest first starts to move; it cannot
be obtained from the constant-velocity friction sweep (the velocity loop cannot
hold a steady speed in the stiction regime), so it is measured here directly.

A slowly rising OPEN-LOOP torque is commanded from rest -- the position/velocity
feedback is disabled (kp_scale = kd_scale = 0 and the integrator gain zeroed) so
the applied torque equals the commanded feedforward torque.  The torque at which
the rotor first breaks away (position moves past a small threshold) is the
breakaway/stiction torque.  Each trial is stopped the instant motion is detected,
because an open-loop torque on a freed rotor accelerates without bound.

The measurement is repeated in both directions and at several starting positions
(stiction varies with position through residual cogging).

    uv run python moteus-config/scripts/measure_stiction.py -t 1 --trials 6

SAFETY: torque is bounded by --max-torque and the rotor is stopped on first
motion, but be ready to power down.
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

ENC1 = moteus.Register.ENCODER_1_POSITION   # MA600 load-side angle (aux2)


async def conf_get_float(stream, name):
    for _ in range(5):
        r = await stream.command(f"conf get {name}".encode(), allow_any_response=True)
        try:
            return float(r.decode().strip())
        except (ValueError, AttributeError):
            continue
    raise RuntimeError(f"could not read {name}")


async def reposition(controller, advance, spin_vel, brake_time):
    """Advance the rotor by ~advance [motor rev] then actively brake to rest.

    Spreading the sample points across the output rotation requires moving the
    rotor between trials; doing it in closed loop also actively damps the ring
    left by the previous open-loop breakaway (far quicker than waiting for it to
    coast), which is the other half of the settling problem."""
    if advance != 0.0:
        dur = abs(advance) / spin_vel
        v = math.copysign(spin_vel, advance)
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            await controller.set_position(position=math.nan, velocity=v,
                                          accel_limit=8.0, query=False)
            await asyncio.sleep(0.01)
    # hold velocity 0 closed-loop to kill the ring before de-energising
    t0 = time.monotonic()
    while time.monotonic() - t0 < brake_time:
        await controller.set_position(position=math.nan, velocity=0.0,
                                      accel_limit=8.0, query=False)
        await asyncio.sleep(0.01)


async def wait_until_stopped(controller, v_stop, settle, dwell):
    """Hold de-energised until |velocity| stays below v_stop for `dwell` seconds.

    Final confirmation that the rotor is genuinely at rest (de-energised) before
    capturing p_start; gives up after `settle` s."""
    t0 = time.monotonic()
    t_below = None
    while time.monotonic() - t0 < settle:
        r = await controller.set_stop(query=True)
        v = abs(r.values[moteus.Register.VELOCITY])
        if v < v_stop:
            if t_below is None:
                t_below = time.monotonic()
            elif time.monotonic() - t_below > dwell:
                return True
        else:
            t_below = None
        await asyncio.sleep(0.02)
    return False


async def one_trial(controller, direction, args, advance):
    """Reposition, settle, then ramp open-loop torque until breakaway.

    Returns (breakaway_torque [Nm] or None, load_angle [rev, fractional])."""
    await reposition(controller, advance, args.spin_vel, args.brake_time)
    await controller.set_stop()
    await wait_until_stopped(controller, args.v_stop, args.settle, args.rest_dwell)
    r0 = await controller.set_position(position=math.nan, velocity=0.0,
                                       kp_scale=0.0, kd_scale=0.0,
                                       feedforward_torque=0.0, query=True)
    p_start = r0.values[moteus.Register.POSITION]
    load_angle = r0.values[ENC1] % 1.0

    t0 = time.monotonic()
    breakaway = None
    while True:
        t = time.monotonic() - t0
        tau = direction * args.rate * t
        if abs(tau) > args.max_torque:
            break  # never broke away within the torque budget
        r = await controller.set_position(
            position=math.nan, velocity=0.0,
            kp_scale=0.0, kd_scale=0.0,
            feedforward_torque=tau,
            maximum_torque=args.max_torque, query=True)
        p = r.values[moteus.Register.POSITION]
        if abs(p - p_start) > args.move_threshold:
            breakaway = abs(tau)
            break
    await controller.set_stop()
    return breakaway, load_angle


async def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", "-t", type=int, default=1, help="moteus CAN id")
    p.add_argument("--rate", type=float, default=0.005,
                   help="open-loop torque ramp rate [Nm/s] (slow = precise)")
    p.add_argument("--max-torque", type=float, default=0.1,
                   help="torque budget / safety clamp [Nm]")
    p.add_argument("--move-threshold", type=float, default=0.01,
                   help="position change that counts as breakaway [rev] "
                        "(large enough to reject cogging twitches)")
    p.add_argument("--settle", type=float, default=8.0,
                   help="max time to wait for the rotor to come to rest before "
                        "each trial [s]")
    p.add_argument("--v-stop", type=float, default=0.05,
                   help="|velocity| below which the rotor counts as at rest "
                        "[rev/s]")
    p.add_argument("--rest-dwell", type=float, default=1.0,
                   help="how long |velocity| must stay below v-stop to count as "
                        "at rest [s]")
    p.add_argument("--brake-time", type=float, default=1.5,
                   help="closed-loop velocity-0 hold to damp the ring between "
                        "trials [s]")
    p.add_argument("--spin-vel", type=float, default=2.0,
                   help="repositioning speed between trials [rev/s]")
    p.add_argument("--positions", type=int, default=8,
                   help="number of rest positions sampled per direction")
    p.add_argument("--span-output", type=float, default=1.0,
                   help="output-rotor span to sweep the samples over [rev]")
    p.add_argument("--ratio", type=float, default=4.14,
                   help="belt ratio i = motor/output rev, to convert the output "
                        "span to a motor-frame step")
    p.add_argument("--label", default=None)
    args = p.parse_args()

    run = Run("stiction", target=args.target, label=args.label)

    qr = moteus.QueryResolution()
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.torque = moteus.F32
    qr._extra[ENC1] = mp_res.F32
    controller = moteus.Controller(id=args.target, query_resolution=qr)
    stream = moteus.Stream(controller)
    await stream.write_message(b"tel stop")
    await stream.flush_read()

    # disable the integrator for the duration (restore afterwards)
    ki_orig = await conf_get_float(stream, "servo.pid_position.ki")
    await stream.command(b"conf set servo.pid_position.ki 0")

    # motor-frame advance between samples, spreading them over span_output revs
    step = args.span_output * args.ratio / args.positions
    results = {+1: [], -1: []}     # breakaway torques
    angles = {+1: [], -1: []}      # load angle at each sample
    try:
        for d in (+1, -1):
            for i in range(args.positions):
                tb, ang = await one_trial(controller, d, args, step)
                tag = "fwd" if d > 0 else "rev"
                if tb is None:
                    print(f"  {tag} {i+1}/{args.positions} "
                          f"(load {ang:.3f} rev): no breakaway below "
                          f"{args.max_torque} Nm")
                else:
                    print(f"  {tag} {i+1}/{args.positions} "
                          f"(load {ang:.3f} rev): breakaway = {tb*1e3:.2f} mNm")
                    results[d].append(tb)
                    angles[d].append(ang)
    finally:
        await stream.command(f"conf set servo.pid_position.ki {ki_orig}".encode())
        await controller.set_stop()

    fwd = np.array(results[+1]) if results[+1] else np.array([np.nan])
    rev = np.array(results[-1]) if results[-1] else np.array([np.nan])
    allb = np.concatenate([fwd, rev])

    def stats(x):
        return (np.nanmean(x) * 1e3, np.nanstd(x) * 1e3,
                np.nanmin(x) * 1e3, np.nanmax(x) * 1e3)

    print(f"\nBreakaway (stiction) torque vs output position, "
          f"moteus id={args.target} ({run.motor}):")
    print(f"{'dir':>8} {'mean':>8} {'std':>7} {'min':>7} {'max':>7}  [mNm]")
    for tag, x in (("forward", fwd), ("reverse", rev), ("combined", allb)):
        m, s, lo, hi = stats(x)
        print(f"{tag:>8} {m:8.2f} {s:7.2f} {lo:7.2f} {hi:7.2f}")
    print("\nFor the (motor-frame, position-independent) breakaway feed-forward:")
    print(f"  T_s mean (balanced)      = {np.nanmean(allb)*1e3:.2f} mNm")
    print(f"  T_s min  (no over-inject)= {np.nanmin(allb)*1e3:.2f} mNm  "
          f"<- leaves the position-dependent excess to the error dynamics")
    print(f"  T_s max  (kp lower bound)= {np.nanmax(allb)*1e3:.2f} mNm")

    run.set_meta(
        rate_Nm_per_s=args.rate, max_torque_Nm=args.max_torque,
        move_threshold_rev=args.move_threshold, positions=args.positions,
        span_output_rev=args.span_output, ratio=args.ratio,
        T_s_fwd_mean_Nm=float(np.nanmean(fwd)), T_s_rev_mean_Nm=float(np.nanmean(rev)),
        T_s_mean_Nm=float(np.nanmean(allb)), T_s_min_Nm=float(np.nanmin(allb)),
        T_s_max_Nm=float(np.nanmax(allb)), T_s_std_Nm=float(np.nanstd(allb)),
        breakaway_fwd_Nm=[float(x) for x in results[+1]],
        breakaway_rev_Nm=[float(x) for x in results[-1]],
        load_angle_fwd_rev=[float(x) for x in angles[+1]],
        load_angle_rev_rev=[float(x) for x in angles[-1]],
    )

    # ---- plot breakaway vs output-rotor angle ----
    plt.figure(figsize=(8, 5))
    if results[+1]:
        plt.scatter(angles[+1], np.array(results[+1]) * 1e3, label="forward")
    if results[-1]:
        plt.scatter(angles[-1], np.array(results[-1]) * 1e3, label="reverse")
    plt.axhline(np.nanmean(allb) * 1e3, c="grey", ls="--",
                label=f"mean {np.nanmean(allb)*1e3:.2f} mNm")
    plt.axhline(np.nanmin(allb) * 1e3, c="green", ls=":",
                label=f"min {np.nanmin(allb)*1e3:.2f} mNm")
    plt.xlabel("output-rotor angle  [rev]")
    plt.ylabel("breakaway torque  [mNm]")
    plt.title(f"Stiction vs output position ({run.motor}, id={args.target})")
    plt.legend(fontsize=8)
    plt.grid(True)
    plt.tight_layout()
    run.save_fig(plt, "stiction_vs_angle.png")
    run.finish()


if __name__ == "__main__":
    asyncio.run(main())
