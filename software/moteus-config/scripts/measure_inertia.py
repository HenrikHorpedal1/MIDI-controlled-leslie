#!/usr/bin/env python3
"""Measure motor shaft inertia using feedforward torque pulses.

Based on the upstream mjbots measure_inertia.py method:
  apply a known feedforward torque for 30 ms, measure the resulting
  angular acceleration ω̇ from velocity samples 25%-50% into the pulse,
  J = T / (2π · ω̇)  [kg·m²].

Run with the belt DISCONNECTED to measure motor-only inertia.

    uv run python moteus-config/scripts/measure_inertia.py -t 1 --runs 5

Saves per-torque traces (CSV), inertia estimates (CSV), and an SVG plot
to analysis/data/inertia/<motor>_<timestamp>/.
"""

import argparse
import asyncio
import math
import time

import moteus
import numpy as np
import matplotlib.pyplot as plt

from _artifacts import Run

PULSE_COUNT = 30      # samples per torque pulse
PULSE_DT    = 0.001   # s between samples during pulse


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1,
                        help="moteus CAN id (1=drum, 2=horn)")
    parser.add_argument("--runs", type=int, default=1,
                        help="number of full sweeps to average (default 1)")
    parser.add_argument("--scale", type=float, default=1.3,
                        help="torque multiplier each iteration (default 1.3)")
    parser.add_argument("--count", type=int, default=PULSE_COUNT,
                        help="samples per torque pulse (default 30)")
    parser.add_argument("--min-vel-std", type=float, default=0.003,
                        help="minimum velocity noise floor [rev/s]")
    parser.add_argument("--finish-metric", type=float, default=500,
                        help="stop when velocity swing > this × noise (default 500)")
    parser.add_argument("--max-torque", type=float, default=0.1,
                        help="upper torque limit [Nm] (default 0.1)")
    parser.add_argument("--label", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    run = Run("inertia", target=args.target, label=args.label)
    transport = moteus.get_singleton_transport(args)
    c = moteus.Controller(id=args.target, transport=transport)
    s = moteus.Stream(c)

    await s.write_message(b"tel stop")
    await s.flush_read()

    pll_hz = float(await s.command(
        b"conf get motor_position.sources.0.pll_filter_hz",
        allow_any_response=True))
    if pll_hz < 400:
        raise RuntimeError(
            f"pll_filter_hz={pll_hz} < 400. Set to 400 in RAM first:\n"
            "  uv run python moteus-config/scripts/apply_config.py -t 1 "
            "--no-write motor_position.sources.0.pll_filter_hz=400")

    try:
        await s.command(b"conf set servo.max_current_desired_rate 10000000")
    except moteus.CommandError:
        await s.command(b"conf set servo.pid_dq.max_desired_rate 10000000")

    all_inertia_rows = []
    all_good_traces  = {}   # torque -> list of (t, v) from last run (for plot)

    try:
        for i in range(args.runs):
            print(f"\n=== Run {i+1}/{args.runs} ===")
            rows, good = await _sweep(args, c)
            for r in rows:
                r["run"] = i + 1
            all_inertia_rows.extend(rows)
            all_good_traces = good   # keep last run's traces for plot
    finally:
        await c.set_stop()

    inertias_clean = [r["J_kg_m2"] for r in all_inertia_rows
                      if math.isfinite(r["J_kg_m2"]) and r["J_kg_m2"] > 0]
    J_median    = float(np.median(inertias_clean))
    J_mean      = float(np.mean(inertias_clean))
    J_std       = float(np.std(inertias_clean))
    J_nm_s2_rev = J_median * 2 * math.pi

    print(f"\n=== Grand summary ({args.runs} run(s), {len(inertias_clean)} estimates) ===")
    print(f"  J_motor (median)  = {J_median:.4e} kg·m²")
    print(f"  J_motor (mean)    = {J_mean:.4e} kg·m²")
    print(f"  J_motor (std)     = {J_std:.4e} kg·m²")
    print(f"  J_motor           = {J_nm_s2_rev:.4e} Nm·s²/rev")

    run.save_csv(all_inertia_rows, "inertia_estimates.csv")
    run.set_meta(
        n_runs=args.runs,
        n_estimates=len(inertias_clean),
        J_motor_kg_m2_median=J_median,
        J_motor_kg_m2_mean=J_mean,
        J_motor_kg_m2_std=J_std,
        J_motor_Nm_s2_rev=J_nm_s2_rev,
    )

    _plot(all_inertia_rows, all_good_traces, J_median, J_std, run, args)


async def _sweep(args, c):
    """One full torque sweep. Returns (inertia_rows, good_traces)."""
    scale_threshold = 0.5 * (args.scale - 1) + 1

    await c.set_stop()
    await asyncio.sleep(0.5)

    vel_noise = [
        (await c.query()).values[moteus.Register.VELOCITY]
        for _ in range(50)
    ]
    velocity_std = max(args.min_vel_std, float(np.std(vel_noise)))
    print(f"  noise σ = {velocity_std:.4f} rev/s")

    torque = 0.001
    all_traces = {}
    last_end_velocity = None

    while True:
        print(f"  torque = {torque:.5f} Nm … ", end="", flush=True)
        await c.set_stop()

        trace = []
        for _ in range(args.count):
            now = time.time()
            r = await c.set_position(
                position=math.nan, kp_scale=0.0, kd_scale=0.0,
                ilimit_scale=0.0, feedforward_torque=torque,
                ignore_position_bounds=True, query=True)
            trace.append((now, r.values[moteus.Register.VELOCITY]))
            await asyncio.sleep(PULSE_DT)

        finish_time = None
        while True:
            now = time.time()
            data = await c.set_brake(query=True)
            await asyncio.sleep(PULSE_DT)
            v = abs(data.values[moteus.Register.VELOCITY])
            if v < 0.1 and finish_time is None:
                finish_time = now + 0.5
            if finish_time and now > finish_time:
                break

        velocities = [v for _, v in trace]
        delta_v = max(velocities) - min(velocities)
        end_v   = trace[-1][1]
        print(f"Δv = {delta_v:.3f} rev/s")

        if last_end_velocity is not None:
            if (delta_v > args.finish_metric * velocity_std and
                    end_v < scale_threshold * last_end_velocity):
                print("  → sufficient signal, stopping sweep")
                break

        if data.values[moteus.Register.MODE] == 1:
            print(f"  → fault {data.values[moteus.Register.FAULT]}, stopping")
            break

        if delta_v > 4000 * velocity_std:
            print("  → very large Δv, stopping sweep")
            break

        all_traces[torque] = trace
        last_end_velocity = end_v
        torque *= args.scale
        if torque > args.max_torque:
            print(f"  → reached --max-torque {args.max_torque} Nm, stopping sweep")
            break

    def vel_change(tr):
        i0, i1 = len(tr) // 4, len(tr) // 2
        return abs(tr[i1][1] - tr[i0][1])

    if not all_traces:
        return [], {}

    max_dv = max(vel_change(tr) for tr in all_traces.values())
    good   = {tq: tr for tq, tr in all_traces.items()
              if vel_change(tr) > 0.10 * max_dv}

    inertia_rows = []
    for tq, tr in good.items():
        i0, i1 = len(tr) // 4, len(tr) // 2
        t0, v0 = tr[i0]
        t1, v1 = tr[i1]
        dt = t1 - t0
        accel = (v1 - v0) / dt if dt > 0 else float("nan")
        J = tq / (accel * 2 * math.pi) if accel != 0 else float("nan")
        inertia_rows.append(dict(torque_Nm=tq, accel_rev_s2=accel, J_kg_m2=J))

    return inertia_rows, good


def _plot(all_rows, traces, J_median, J_std, run, args):
    torques = np.array([r["torque_Nm"] for r in all_rows])
    Js      = np.array([r["J_kg_m2"]   for r in all_rows])
    runs    = np.array([r.get("run", 1) for r in all_rows])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))

    # left: velocity traces from the last sweep
    cmap = plt.cm.viridis(np.linspace(0.15, 0.85, len(traces)))
    for (tq, tr), color in zip(traces.items(), cmap):
        t_arr = np.array([ts - tr[0][0] for ts, _ in tr]) * 1e3
        v_arr = np.array([v for _, v in tr])
        n = len(tr)
        i0, i1 = n // 4, n // 2
        # dashed before and after window, solid inside
        ax1.plot(t_arr[:i0+1],  v_arr[:i0+1],  color=color, lw=1.2, ls="--")
        ax1.plot(t_arr[i0:i1+1], v_arr[i0:i1+1], color=color, lw=2.0, ls="-",
                 label=f"{tq*1e3:.1f} mNm")
        ax1.plot(t_arr[i1:],    v_arr[i1:],    color=color, lw=1.2, ls="--")

    ax1.set_xlabel("time in pulse  [ms]")
    ax1.set_ylabel("motor velocity  [rev/s]")
    ax1.set_title("Velocity traces (shaded = measurement window)")
    ax1.legend(fontsize=7, ncol=2, loc="upper left")
    ax1.grid(True)

    # right: all J estimates, coloured by run
    n_runs = int(runs.max()) if len(runs) else 1
    run_cmap = plt.cm.tab10(np.linspace(0, 0.9, max(n_runs, 1)))
    for ri in range(1, n_runs + 1):
        mask = runs == ri
        ax2.scatter(torques[mask] * 1e3, Js[mask] * 1e6,
                    color=run_cmap[ri - 1], zorder=3, s=40,
                    label=f"run {ri}")
    ax2.axhline(J_median * 1e6, ls="--", color="r",
                label=f"median  {J_median*1e6:.2f} µkg·m²")
    t_min = torques.min() * 1e3 * 0.8
    t_max = torques.max() * 1e3 * 1.2
    ax2.fill_between([t_min, t_max],
                     (J_median - J_std) * 1e6, (J_median + J_std) * 1e6,
                     alpha=0.15, color="r", label=f"±1σ  ({J_std*1e6:.2f} µkg·m²)")
    ax2.set_xlabel("feedforward torque  [mNm]")
    ax2.set_ylabel("inertia estimate  [µkg·m²]")
    ax2.set_title(f"J per torque level  ({n_runs} run(s))")
    ax2.legend(fontsize=8)
    ax2.grid(True)

    fig.suptitle(f"Motor inertia — {run.motor} (belt off),  "
                 f"J = {J_median*1e6:.2f} ± {J_std*1e6:.2f} µkg·m²")
    fig.tight_layout()
    run.save_fig(fig, "inertia.svg")
    run.finish()

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    asyncio.run(main())
