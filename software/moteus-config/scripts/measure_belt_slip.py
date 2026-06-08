#!/usr/bin/env python3
"""Measure friction-belt slip at multiple velocities and fit the slip model.

Physical setup:
  - VELOCITY register  → motor shaft ω_m [rev/s]  (rotor_to_output_ratio = 1)
  - ENCODER_1_POSITION → MA600 load position [0-1 per load rev], used to derive ω_L

Slip model (Isermann):
  i  = ω_m / ω_L  (actual transmission ratio)
  i0 = ideal (no-slip) ratio = r_L / r_m
  s  = 1 - i0/i  = 1 - i0 * ω_L / ω_m

"Scales linearly" check: if ω_L vs ω_m is linear through the origin → s is constant.
"""

import asyncio
import csv
import time
import argparse
from datetime import datetime

import moteus
import moteus.multiplex as mp_res
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

SETTLE_TIME = 10.0   # s to wait for steady state after velocity command
CAPTURE_TIME = 3.0   # s to average over
SAMPLE_PERIOD = 0.01 # s between queries (~100 Hz)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def unwrap_fractional(prev, curr):
    """Return delta in revolutions from two fractional [0-1) positions."""
    d = curr - prev
    if d > 0.5:
        d -= 1.0
    elif d < -0.5:
        d += 1.0
    return d


async def measure_at_velocity(controller, v_cmd, qr):
    """
    Command v_cmd [rev/s] on motor, wait for steady state, then capture
    CAPTURE_TIME seconds of data.

    Returns (omega_m, omega_L, q_A) as scalar means over the capture window.
    """
    # Keep sending the velocity command so the watchdog doesn't trip.
    t_settle = time.monotonic()
    while time.monotonic() - t_settle < SETTLE_TIME:
        await controller.set_position(position=float("nan"), velocity=v_cmd, query=False)
        await asyncio.sleep(SAMPLE_PERIOD)

    # ----- capture loop -----
    t_start = time.monotonic()
    prev_enc = None
    prev_t   = None
    cum_enc_delta = 0.0   # cumulative load revolutions
    motor_vel_samples = []
    q_samples = []
    n = 0

    while time.monotonic() - t_start < CAPTURE_TIME:
        t = time.monotonic()
        result = await controller.set_position(
            position=float("nan"), velocity=v_cmd, query=True, query_override=qr)

        enc = result.values[moteus.Register.ENCODER_1_POSITION]
        vm  = result.values[moteus.Register.VELOCITY]
        q   = result.values[moteus.Register.Q_CURRENT]

        if prev_enc is not None:
            cum_enc_delta += unwrap_fractional(prev_enc, enc)

        prev_enc = enc
        motor_vel_samples.append(vm)
        q_samples.append(q)
        n += 1
        await asyncio.sleep(SAMPLE_PERIOD)

    elapsed = time.monotonic() - t_start

    omega_m = float(np.mean(motor_vel_samples))
    # average load velocity from cumulative position change
    omega_L = cum_enc_delta / elapsed if elapsed > 0 else float("nan")
    q_mean  = float(np.mean(q_samples))

    return omega_m, omega_L, q_mean


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

async def main():
    parser = argparse.ArgumentParser(
        description="Fit friction-belt slip model at multiple velocities."
    )
    moteus.make_transport_args(parser)
    parser.add_argument("--target", "-t", type=int, default=1,
                        help="moteus CAN ID (1=drum, 2=horn)")
    parser.add_argument("--i0", type=float, default=None,
                        help="Ideal transmission ratio i0 = r_L/r_m.  "
                             "If omitted, estimated as min observed ratio.")
    parser.add_argument("--min-vel", type=float, default=0.5,
                        help="Minimum motor velocity [rev/s] (default 0.5)")
    parser.add_argument("--max-vel", type=float, default=6.0,
                        help="Maximum motor velocity [rev/s] (default 6.0)")
    parser.add_argument("--steps", type=int, default=8,
                        help="Number of velocity steps (default 8)")
    parser.add_argument("--output", "-o", default=None,
                        help="CSV output file (default: slip_measurements_<timestamp>.csv)")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.output is None:
        args.output = f"slip_measurements_{timestamp}.csv"
    plot_output = args.output.replace(".csv", ".png")

    transport = moteus.get_singleton_transport(args)

    qr = moteus.QueryResolution()
    qr.velocity  = moteus.F32
    qr.q_current = moteus.F32
    qr._extra[moteus.Register.ENCODER_1_POSITION] = mp_res.F32

    controller = moteus.Controller(id=args.target, transport=transport,
                                   query_resolution=qr)
    stream = moteus.Stream(controller)

    await stream.write_message(b"tel stop")
    await stream.flush_read()
    await controller.set_stop()
    await asyncio.sleep(0.3)

    velocity_steps = np.linspace(args.min_vel, args.max_vel, args.steps)

    rows = []
    print(f"Target: moteus id={args.target}")
    print(f"{'Cmd':>6} {'ω_m':>10} {'ω_L':>10} {'i=ω_m/ω_L':>12} {'q_A':>8}")
    print("-" * 52)

    try:
        for v_cmd in velocity_steps:
            omega_m, omega_L, q_mean = await measure_at_velocity(
                controller, float(v_cmd), qr)

            if abs(omega_L) < 1e-4:
                print(f"{v_cmd:6.2f}  load encoder not moving — skip")
                continue

            ratio_i = omega_m / omega_L
            print(f"{v_cmd:6.2f} {omega_m:10.4f} {omega_L:10.4f} {ratio_i:12.5f} {q_mean:8.4f}")
            rows.append(dict(v_cmd=v_cmd, omega_m=omega_m, omega_L=omega_L,
                             ratio_i=ratio_i, q_A=q_mean))

    finally:
        await controller.set_stop()

    if not rows:
        print("No data collected.")
        return

    # ---- save ----
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSaved {len(rows)} points → {args.output}")

    # ---- analysis ----
    om = np.array([r["omega_m"] for r in rows])
    oL = np.array([r["omega_L"] for r in rows])
    ri = np.array([r["ratio_i"] for r in rows])
    q  = np.array([r["q_A"]    for r in rows])

    # i0: use provided value, or estimate from min observed ratio
    if args.i0 is not None:
        i0 = args.i0
        print(f"\ni0 (provided)            = {i0:.5f}")
    else:
        i0 = float(np.min(ri))
        print(f"\ni0 (min observed ratio)  = {i0:.5f}  (pass --i0 for a known value)")

    slip = 1.0 - i0 / ri    # s = 1 - i0/i

    # Linearity: force through origin  (ω_L = slope * ω_m)
    slope_forced = float(np.dot(om, oL) / np.dot(om, om))
    residuals     = oL - slope_forced * om
    ss_res = float(np.sum(residuals**2))
    ss_tot = float(np.sum((oL - np.mean(oL))**2))
    r2_forced = 1.0 - ss_res / ss_tot

    # Free linear fit
    lr = linregress(om, oL)
    slope_free: float = float(lr.slope)
    intercept:  float = float(lr.intercept)
    r2_free:    float = float(lr.rvalue ** 2)

    print(f"\n--- Linearity (ω_L vs ω_m) ---")
    print(f"  Forced-through-origin slope  = {slope_forced:.5f}   R² = {r2_forced:.6f}")
    print(f"  Free linear fit slope        = {slope_free:.5f},  intercept = {intercept:.5f},  R² = {r2_free:.6f}")
    print(f"  Implied i from forced slope  = {1/slope_forced:.5f}")

    print(f"\n--- Slip s = 1 - i0/i ---")
    print(f"  mean  = {np.mean(slip)*100:.3f}%")
    print(f"  std   = {np.std(slip)*100:.3f}%")
    print(f"  range = [{np.min(slip)*100:.3f}%, {np.max(slip)*100:.3f}%]")

    # Check if slip scales linearly with speed or with torque (q_A ∝ T)
    sl_om: float = 0.0
    r2_som: float = 0.0
    sl_q:   float = 0.0
    r2_sq:  float = 0.0
    if len(rows) >= 3:
        lr_om = linregress(om, slip)
        sl_om  = float(lr_om.slope)
        r2_som = float(lr_om.rvalue ** 2)
        lr_q   = linregress(q, slip)
        sl_q   = float(lr_q.slope)
        r2_sq  = float(lr_q.rvalue ** 2)
        print(f"\n  s vs ω_m  :  slope={sl_om*100:.4f}%/(rev/s),  R²={r2_som:.4f}")
        print(f"  s vs q_A  :  slope={sl_q*100:.4f}%/A,          R²={r2_sq:.4f}")

    # ---- plots ----
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    # 1. ω_L vs ω_m  (linearity)
    ax = axes[0]
    ax.scatter(om, oL, zorder=3, label="measured")
    x = np.array([0, om.max() * 1.05])
    ax.plot(x, slope_forced * x, "--",
            label=f"forced-origin fit  (R²={r2_forced:.4f})")
    ax.plot(x, slope_free * x + intercept, ":",
            label=f"free fit  (R²={r2_free:.4f})")
    ax.set_xlabel("Motor ω_m  [rev/s]")
    ax.set_ylabel("Load ω_L  [rev/s]")
    ax.set_title("Linearity check")
    ax.legend(fontsize=8)
    ax.grid(True)

    # 2. slip vs speed
    ax = axes[1]
    ax.scatter(om, slip * 100, zorder=3)
    ax.axhline(np.mean(slip) * 100, ls="--", c="r",
               label=f"mean s = {np.mean(slip)*100:.3f}%")
    if len(rows) >= 3 and r2_som > 0:
        ax.plot(om, (sl_om * om + (np.mean(slip) - sl_om * np.mean(om))) * 100,
                ":", c="grey", label=f"linear fit  R²={r2_som:.3f}")
    ax.set_xlabel("Motor ω_m  [rev/s]")
    ax.set_ylabel("Slip  s  [%]")
    ax.set_title("Slip vs speed")
    ax.legend(fontsize=8)
    ax.grid(True)

    # 3. slip vs q-current (∝ torque)
    ax = axes[2]
    ax.scatter(q, slip * 100, zorder=3)
    ax.set_xlabel("q-current  [A]  (∝ torque)")
    ax.set_ylabel("Slip  s  [%]")
    ax.set_title("Slip vs torque")
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(plot_output, dpi=150)
    print(f"\nPlot saved → {plot_output}")
    plt.show()


if __name__ == "__main__":
    asyncio.run(main())
