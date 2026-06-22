#!/usr/bin/env python3
"""Beat-sync command structure test: replicate the ESP32 sendBeatSync command
and compare feedforward conditions.

Mirrors the firmware's sendBeatSync command exactly:

    cmd.position      = beat_face (advancing at vel_target each 5 ms)
    cmd.velocity      = vel_target          <- vf for the moteus planner
    cmd.accel_limit   = accel_up (spin-up) or TRIM_ACCEL (after lock)
    cmd.velocity_limit = vel_target * VEL_MARGIN

The firmware NEVER sets feedforward_torque.  This test adds it as a variable
to measure whether FF helps the spin-up transient and cruise tracking.

Conditions tested:
    no FF          firmware baseline (no feedforward_torque)
    ext friction   friction(v_ref) externally
    ext inertia    J*a_ref externally
    ext full       friction(v_ref) + J*a_ref externally
    mot inertia    servo.inertia_feedforward = J, no external FF
    mot+friction   servo.inertia_feedforward = J, external friction(v_ref)

v_ref / a_ref are reconstructed from the expected spin-up ramp:
    v_ref(tau) = min(vel_target, accel_up * tau)     during spin-up
               = vel_target                          during cruise
    a_ref(tau) = accel_up   (spin-up)  or  0 (cruise)

Metrics:
    lock_t     time from ramp start until |v - vel_target| < lock_threshold
    cruise_rms rms velocity error during the cruise hold (rev/s)
    phase_rms  rms position error from beat_face during cruise (arc-min)

    uv run python moteus-config/scripts/test_beat_sync.py -t 1 \\
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

SAMPLE_DT       = 0.005    # 200 Hz — matches ESP32 CONTROL_DT_S
TRIM_ACCEL      = 0.20     # rev/s²  — BEAT_CAM_TRIM_ACCEL_REV_S2 from firmware
VEL_MARGIN      = 1.02     # velocity_limit = vel_target * VEL_MARGIN
LOCK_THRESHOLD  = 0.05     # rev/s — BEAT_CAM_LOCK_VEL_REVS from firmware


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


async def run_beat_sync(controller, stream, qr, ff_mode, moteus_J, args):
    """One beat-sync replication run.

    ff_mode:
      "off"      no external feedforward_torque  (firmware baseline)
      "friction" external friction(v_ref)
      "inertia"  external J*a_ref
      "ref"      external friction(v_ref) + J*a_ref
    moteus_J:
      value written to servo.inertia_feedforward (0 or args.J)
    """
    await stream.command(
        f"conf set servo.inertia_feedforward {moteus_J}".encode())

    await settle_stop(controller)

    # Read starting position — beat_face starts here so phase error begins at 0.
    r0 = await controller.set_position(position=math.nan, query=True)
    pos0 = r0.values[moteus.Register.POSITION]

    t_acc    = args.v_target / args.accel_limit  # spin-up duration
    duration = args.pre + t_acc + args.hold

    samples   = []
    locked    = False
    lock_t    = float("nan")
    t0        = time.monotonic()

    while True:
        t   = time.monotonic() - t0
        tau = t - args.pre          # time since ramp start (negative during pre)
        if t > duration:
            break

        # Beat face: the moving position target the ESP32 sends.
        # During pre: hold at pos0 (motor is already there).
        # After pre:  advance at vel_target, mirroring the live beat phase.
        if tau <= 0.0:
            beat_face = pos0
            v_ref     = 0.0
            a_ref     = 0.0
            v_cmd     = 0.0
            accel     = args.accel_limit
            v_lim     = args.v_target * VEL_MARGIN
        else:
            beat_face = pos0 + args.v_target * tau
            # Reference ramp for FF computation
            v_ref = min(args.v_target, args.accel_limit * tau)
            a_ref = args.accel_limit if v_ref < args.v_target else 0.0
            v_cmd = args.v_target   # vf for moteus planner (firmware: plan.velRevS)
            # After lock use the trim accel (firmware: BEAT_CAM_TRIM_ACCEL_REV_S2)
            accel = TRIM_ACCEL if locked else args.accel_limit
            v_lim = args.v_target * VEL_MARGIN

        # External feedforward torque
        if ff_mode == "friction":
            T_ff = friction_ff(v_ref, args.c, args.b, args.a)
        elif ff_mode == "inertia":
            T_ff = args.J * a_ref
        elif ff_mode == "ref":
            T_ff = friction_ff(v_ref, args.c, args.b, args.a) + args.J * a_ref
        else:
            T_ff = 0.0

        # Send the beat-sync command (mirrors sendBeatSync in controller.cpp)
        r = await controller.set_position(
            position=beat_face,
            velocity=v_cmd,
            accel_limit=accel,
            velocity_limit=v_lim,
            feedforward_torque=T_ff,
            maximum_torque=args.max_torque,
            query=True,
            query_override=qr)

        v_meas   = r.values[moteus.Register.VELOCITY]
        pos_meas = r.values[moteus.Register.POSITION]

        # Lock detection — mirrors BEAT_CAM_LOCK_VEL_REVS check in firmware
        if tau > 0 and not locked and abs(v_meas - args.v_target) < LOCK_THRESHOLD:
            locked = True
            lock_t = tau

        samples.append(dict(
            t=t, tau=tau,
            beat_face=beat_face, v_ref=v_ref, a_ref=a_ref, T_ff=T_ff,
            pos=pos_meas, vel=v_meas,
            phase_err=(pos_meas - beat_face),   # position error from beat face [rev]
            vel_err=(v_meas - args.v_target),
            locked=float(locked),
            torque=r.values[moteus.Register.TORQUE]))

        await asyncio.sleep(SAMPLE_DT)

    await controller.set_stop()

    tau_arr    = np.array([s["tau"]       for s in samples])
    vel_arr    = np.array([s["vel"]       for s in samples])
    phase_arr  = np.array([s["phase_err"] for s in samples])

    cruise_mask = (tau_arr >= t_acc) & (tau_arr <= t_acc + args.hold)

    cruise_rms = (float(np.sqrt(np.mean((vel_arr[cruise_mask] - args.v_target) ** 2)))
                  if cruise_mask.any() else float("nan"))
    phase_rms  = (float(np.sqrt(np.mean(phase_arr[cruise_mask] ** 2))) * 360 * 60
                  if cruise_mask.any() else float("nan"))

    return samples, dict(lock_t=lock_t, cruise_rms=cruise_rms, phase_rms=phase_rms)


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1)
    parser.add_argument("--v-target", type=float, default=1.0,
                        help="cruise velocity [rev/s] (default 1.0)")
    parser.add_argument("--accel-limit", type=float, default=2.0,
                        help="spin-up accel [rev/s²] (default 2.0)")
    parser.add_argument("--hold", type=float, default=2.0,
                        help="cruise hold time after lock [s] (default 2.0)")
    parser.add_argument("--pre", type=float, default=0.5,
                        help="pre-ramp record time [s] (default 0.5)")
    parser.add_argument("--c", type=float, default=2.82e-3)
    parser.add_argument("--b", type=float, default=2.28e-4)
    parser.add_argument("--a", type=float, default=2.70e-5)
    parser.add_argument("--J", type=float, default=7.30e-3)
    parser.add_argument("--max-torque", type=float, default=0.3)
    parser.add_argument("--label", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    run = Run("beat_sync_test", target=args.target, label=args.label)
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
    t_acc = args.v_target / args.accel_limit
    print(f"servo.inertia_feedforward original = {inertia_orig} (restored at end)")
    print(f"Beat-sync: 0 → {args.v_target} rev/s @ {args.accel_limit} rev/s²  "
          f"(t_acc={t_acc:.2f}s)  cruise {args.hold}s")
    print(f"Trim accel = {TRIM_ACCEL} rev/s²,  vel_margin = {VEL_MARGIN},  "
          f"lock_threshold = {LOCK_THRESHOLD} rev/s")
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
            samples, m = await run_beat_sync(
                controller, stream, qr, ff_mode, moteus_J, args)
            results[name] = dict(samples=samples, metrics=m)
            lt = (f"{m['lock_t']*1e3:.0f} ms" if math.isfinite(m["lock_t"])
                  else "no lock")
            print(f"  {name:14s}: lock {lt:>8s}  "
                  f"cruise_rms {m['cruise_rms']*60:6.2f} rpm  "
                  f"phase_rms {m['phase_rms']:6.1f}'")
    finally:
        await stream.command(
            f"conf set servo.inertia_feedforward {inertia_orig}".encode())
        await controller.set_stop()
        print(f"\nRestored servo.inertia_feedforward = {inertia_orig}")

    all_rows = []
    for name, res in results.items():
        for s in res["samples"]:
            all_rows.append({**s, "condition": name})
    run.save_csv(all_rows, "beat_sync_test.csv")
    run.set_meta(
        v_target=args.v_target, accel_limit=args.accel_limit,
        hold_s=args.hold, trim_accel=TRIM_ACCEL,
        vel_margin=VEL_MARGIN, lock_threshold=LOCK_THRESHOLD,
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
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    for name, res in results.items():
        s = res["samples"]
        ax1.plot(arr(s, "t"), arr(s, "vel"),
                 color=colors[name], lw=1, label=name)
        ax2.plot(arr(s, "t"), arr(s, "phase_err") * 360 * 60,
                 color=colors[name], lw=1, label=name)
        ax3.plot(arr(s, "t"), arr(s, "T_ff"),
                 color=colors[name], lw=1, label=name)

    s0 = results["no FF"]["samples"]
    ax1.plot(arr(s0, "t"), arr(s0, "v_ref"), "k:", lw=1, label="v_ref")
    ax1.axhline(args.v_target, color="grey", ls="--", lw=0.8, label="target")

    ax1.set_ylabel("velocity  [rev/s]")
    ax1.set_title(f"Beat-sync test — {run.motor}, id={args.target}  "
                  f"(0 → {args.v_target} rev/s @ {args.accel_limit} rev/s²)")
    ax1.legend(fontsize=8); ax1.grid(True)

    ax2.axhline(0, c="k", lw=0.5)
    ax2.set_ylabel("phase error  [arc-min]")
    ax2.legend(fontsize=8); ax2.grid(True)

    ax3.axhline(0, c="k", lw=0.5)
    ax3.set_ylabel("feedforward torque  [Nm]")
    ax3.set_xlabel("time  [s]")
    ax3.legend(fontsize=8); ax3.grid(True)

    fig.tight_layout()
    run.save_fig(fig, "beat_sync_test.svg")
    run.finish()

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
