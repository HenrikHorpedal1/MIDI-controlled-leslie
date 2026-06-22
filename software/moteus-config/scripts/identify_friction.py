#!/usr/bin/env python3
"""Map the friction torque of the drive as a function of speed.

Commands a sequence of constant velocities in closed loop (both directions) and
records the steady-state torque required to hold each speed.  At constant speed
the inertial term is zero, so the holding torque IS the friction torque:

    T_m(steady) = T_f(omega)

The measured characteristic is fitted to the odd friction + drag model of the
thesis (eq. disturbance-model):

    T_f(w) = sign(w) * (c + a * w^2) + b * w

i.e. sign-aware Coulomb (c), aerodynamic drag (a, quadratic), and viscous (b).
The sign() factors make the model odd in w, matching the requirement that
friction reverse with direction.  The model is linear in (c, b, a), so it is fit
by ordinary least squares.  These coefficients are the velocity feed-forward
table the ESP32 evaluates on the reference velocity.

The breakaway / stiction term (T_s, theta_dot_s) is NOT fitted here -- it is
measured separately by measure_stiction.py.  The friction sweep should be run
with --min-vel set ABOVE the breakaway speed so the map stays out of the
stiction-dominated region.

The per-speed torque standard deviation is recorded as well: at very low speed
the commanded torque carries significant ripple and cannot be fully trusted, so
the spread is reported alongside the mean.

Run from the software/ uv project:
    uv run python moteus-config/scripts/identify_friction.py -t 1
"""

import argparse
import asyncio
import time

import moteus
import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

from _artifacts import Run

CAPTURE_TIME = 3.0    # s to average the holding torque over
SAMPLE_PERIOD = 0.01  # s between queries (~100 Hz)


async def measure_at_velocity(controller, v_cmd, qr, accel, settle_buffer):
    """Drive v_cmd [rev/s], wait for steady state, return mean (torque, q_A).

    The settle time is adaptive: the planner needs |v_cmd|/accel seconds just to
    ramp up to speed, so we wait that ramp time plus settle_buffer before
    averaging.  A fixed settle would start capturing mid-ramp at high speed.
    """
    settle_time = abs(v_cmd) / accel + settle_buffer
    t_settle = time.monotonic()
    while time.monotonic() - t_settle < settle_time:
        await controller.set_position(position=float("nan"), velocity=v_cmd,
                                       accel_limit=accel, query=False)
        await asyncio.sleep(SAMPLE_PERIOD)

    torque_samples = []
    q_samples = []
    vel_samples = []
    t_samples = []
    t_start = time.monotonic()
    while True:
        t = time.monotonic() - t_start
        if t >= CAPTURE_TIME:
            break
        r = await controller.set_position(position=float("nan"), velocity=v_cmd,
                                          accel_limit=accel, query=True,
                                          query_override=qr)
        torque_samples.append(r.values[moteus.Register.TORQUE])
        q_samples.append(r.values[moteus.Register.Q_CURRENT])
        vel_samples.append(r.values[moteus.Register.VELOCITY])
        t_samples.append(t)
        await asyncio.sleep(SAMPLE_PERIOD)

    return (float(np.mean(vel_samples)),
            float(np.mean(torque_samples)),
            float(np.mean(q_samples)),
            float(np.std(torque_samples)),
            float(np.std(vel_samples)),
            t_samples, torque_samples, vel_samples)


def friction_model(w, c, b, a):
    """Odd Coulomb + viscous + quadratic-drag model: sign(w)*(c+a*w^2) + b*w."""
    return np.sign(w) * (c + a * w ** 2) + b * w


async def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1,
                        help="moteus CAN ID (1=drum, 2=horn)")
    parser.add_argument("--min-vel", type=float, default=0.3,
                        help="minimum |velocity| [rev/s]")
    parser.add_argument("--max-vel", type=float, default=6.0,
                        help="maximum |velocity| [rev/s]")
    parser.add_argument("--steps", type=int, default=8,
                        help="velocity steps per direction")
    parser.add_argument("--accel", type=float, default=8.0,
                        help="accel limit while ramping [rev/s^2]")
    parser.add_argument("--settle-buffer", type=float, default=4.0,
                        help="settle time = |v|/accel + this buffer [s] "
                             "(default 4)")
    parser.add_argument("--save-raw", action="store_true",
                        help="also save the raw per-sample torque waveform at "
                             "each speed (for the low-speed ripple plot)")
    parser.add_argument("--both-directions", action="store_true", default=True,
                        help="sweep negative speeds too (default on)")
    parser.add_argument("--ilimit-margin", type=float, default=1.2,
                        help="ilimit = margin * max disturbance torque over the "
                             "measured map (default 1.2)")
    parser.add_argument("--label", default=None,
                        help="optional tag appended to the artifact folder name")
    parser.add_argument("--no-show", action="store_true",
                        help="save the plot without opening a window")
    args = parser.parse_args()

    run = Run("friction", target=args.target, label=args.label)
    transport = moteus.get_singleton_transport(args)
    qr = moteus.QueryResolution()
    qr.velocity = moteus.F32
    qr.q_current = moteus.F32
    qr.torque = moteus.F32
    controller = moteus.Controller(id=args.target, transport=transport,
                                   query_resolution=qr)
    stream = moteus.Stream(controller)

    await stream.write_message(b"tel stop")
    await stream.flush_read()
    await controller.set_stop()
    await asyncio.sleep(0.3)

    speeds = list(np.linspace(args.min_vel, args.max_vel, args.steps))
    if args.both_directions:
        speeds = [-v for v in reversed(speeds)] + speeds

    rows = []
    raw_rows = []
    print(f"Target: moteus id={args.target}")
    print(f"{'v_cmd':>8} {'omega':>10} {'w_std':>8} {'torque[Nm]':>12} "
          f"{'tq_std':>9} {'q_A':>8}")
    print("-" * 60)
    try:
        for v_cmd in speeds:
            w, tq, q, tq_std, w_std, t_raw, tq_raw, w_raw = \
                await measure_at_velocity(
                    controller, float(v_cmd), qr, args.accel, args.settle_buffer)
            print(f"{v_cmd:8.3f} {w:10.4f} {w_std:8.4f} {tq:12.5f} "
                  f"{tq_std:9.5f} {q:8.4f}")
            rows.append(dict(v_cmd=v_cmd, omega=w, omega_std=w_std, torque=tq,
                             torque_std=tq_std, q_A=q))
            if args.save_raw:
                for ti, tqi, wi in zip(t_raw, tq_raw, w_raw):
                    raw_rows.append(dict(v_cmd=v_cmd, omega_mean=w, t=ti,
                                         torque=tqi, omega=wi))
    finally:
        await controller.set_stop()

    run.save_csv(rows, "friction_map.csv")
    if args.save_raw:
        run.save_csv(raw_rows, "friction_raw_samples.csv")
        print(f"  saved {len(raw_rows)} raw torque samples")
    print(f"\nSaved {len(rows)} points")

    # ---- fit ----
    w = np.array([r["omega"] for r in rows])
    T = np.array([r["torque"] for r in rows])

    # T = sign(w)*(c + a*w^2) + b*w  -- linear in (c, b, a), ordinary least squares
    A = np.column_stack([np.sign(w), w, np.sign(w) * w ** 2])
    coef, *_ = np.linalg.lstsq(A, T, rcond=None)
    c, b, a = float(coef[0]), float(coef[1]), float(coef[2])
    print(f"\n--- Coulomb + viscous + drag fit (sign(w)*(c + a*w^2) + b*w) ---")
    print(f"  Coulomb torque  c = {c:.5f} Nm")
    print(f"  viscous coeff   b = {b:.6f} Nm/(rev/s)")
    print(f"  drag coeff      a = {a:.6f} Nm/(rev/s)^2")
    bias_1revs = c + b * 1.0 + a * 1.0
    print(f"\n  bias torque for identify_frf.py at a given speed w0: "
          f"c + b*w0 + a*w0^2  (e.g. {bias_1revs:.4f} Nm at 1 rev/s)")

    # ilimit comes straight from the measured disturbance map, NOT the fit: the
    # integrator only has to overcome the largest friction torque seen across the
    # operating speed range, plus a margin.  (The Coulomb/viscous split is only
    # used for the velocity feed-forward; see the thesis discussion.)
    T_dist_max = float(np.max(np.abs(T)))
    ilimit_rec = args.ilimit_margin * T_dist_max
    print(f"\n  max measured disturbance torque = {T_dist_max:.5f} Nm")
    print(f"  recommended ilimit = {args.ilimit_margin:g} * that "
          f"= {ilimit_rec:.5f} Nm")
    print(f"    conf set servo.pid_position.ilimit {ilimit_rec:.5f}")

    tq_std_max = float(np.max([r["torque_std"] for r in rows]))
    w_std_max = float(np.max([r["omega_std"] for r in rows]))
    run.set_meta(
        fit="coulomb+viscous+drag",
        n_points=len(rows),
        # velocity feed-forward table (ESP32 evaluates on reference velocity):
        #   T_ff(w) = sign(w)*(c + a*w^2) + b*w
        # (breakaway T_s/theta_dot_s come from measure_stiction.py, not here)
        c_Coulomb_Nm=c, b_viscous_Nm_per_revs=b, a_drag_Nm_per_revs2=a,
        bias_torque_at_1revs_Nm=bias_1revs,
        T_disturbance_max_Nm=T_dist_max,
        torque_std_max_Nm=tq_std_max,
        omega_std_max_revs=w_std_max,
        ilimit_margin=args.ilimit_margin, ilimit_recommended_Nm=ilimit_rec,
        speed_range_revs=[float(args.min_vel), float(args.max_vel)],
    )

    # ---- plot ----
    ws = np.linspace(w.min(), w.max(), 400)
    fig, ax = plt.subplots(figsize=(7, 5))
    tq_std = np.array([r["torque_std"] for r in rows])
    w_std = np.array([r["omega_std"] for r in rows])
    # draw ±1σ uncertainty boxes first so dots sit on top
    for xi, yi, dx, dy in zip(w, T, w_std, tq_std):
        ax.add_patch(mpatches.Rectangle(
            (xi - dx, yi - dy), 2 * dx, 2 * dy,
            linewidth=0, facecolor="steelblue", alpha=0.25))
    ax.scatter(w, T, s=18, color="steelblue", zorder=3,
               label="measured  (box = ±1σ speed / torque)")
    ax.plot(ws, friction_model(ws, c, b, a), "r--", label="fit")
    ax.axhline(0, c="k", lw=0.5)
    ax.axvline(0, c="k", lw=0.5)
    ax.set_xlabel("speed  [rev/s]")
    ax.set_ylabel("holding torque  [Nm]")
    ax.set_title(f"Friction map ({run.motor}, moteus id={args.target})")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    run.save_fig(fig, "friction_map.svg")

    # ---- raw torque-ripple plot (low vs high speed) ----
    if args.save_raw and raw_rows:
        speeds_seen = sorted({r["v_cmd"] for r in raw_rows if r["v_cmd"] > 0})
        if speeds_seen:
            lo, hi = speeds_seen[0], speeds_seen[-1]
            plt.figure(figsize=(8, 5))
            for vc in (lo, hi):
                tr = [(r["t"], r["torque"]) for r in raw_rows if r["v_cmd"] == vc]
                ta = np.array([p[0] for p in tr])
                qa = np.array([p[1] for p in tr])
                plt.plot(ta, qa, "-", lw=0.9,
                         label=f"{vc:.2f} rev/s  (σ={np.std(qa):.4f} Nm)")
            plt.xlabel("time  [s]")
            plt.ylabel("commanded torque  [Nm]")
            plt.title(f"Holding-torque ripple, low vs high speed "
                      f"({run.motor}, id={args.target})")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            run.save_fig(plt, "friction_ripple.svg")

    run.finish()
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
