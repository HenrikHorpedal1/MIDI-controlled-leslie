#!/usr/bin/env python3
"""Compute Moteus position-loop PID gains from identified plant parameters.

Implements the gain selection of the thesis (eq. pos-gains-rigid, eq. routh-bound).
The closed loop with inertial feedforward is THIRD order (eq. error-charpoly-rigid),

    Jbar*s^3 + kd*s^2 + kp*s + ki = 0,

because the inertia Jbar cannot be cancelled (no acceleration feedback; it is the
plant itself).  kd is therefore NOT computed from the inertia -- it is fixed at the
empirical velocity-noise ceiling k_d,max found by sweep_kd.py and passed in with
--kd.  With kd fixed, the dominant pole pair is matched to a second-order target
(w_n, zeta):

    kp = 2*zeta*w_n*kd        ki = w_n^2*kd

w_n (rad/s) is bounded from above by the cascade rule (~1/5 of the current loop),
the belt antiresonance (place at/below w_a to stay rigid), AND the Routh-Hurwitz
condition of the cubic, ki < kp*kd/Jbar, which reduces to w_n < 2*zeta*kd/Jbar.
The smallest bound wins.  Gains are in Moteus per-revolution units throughout
(kd, Jbar are supplied in those units), so no 2*pi conversion is needed.

If --ratio i is given, the gains are also rescaled by i^2 for the output frame
(rotor_to_output_ratio = 1/i); the Routh margin is frame-invariant under this
rescale, so the response is preserved with no re-tuning.

No hardware connection -- this is a pure calculator that prints the `conf set`
lines to apply.  Inputs: Jbar (inertia section), w_a (identify_frf.py), kd
(sweep_kd.py), and the desired damping.

    uv run python moteus-config/scripts/compute_position_gains.py \\
        --J 1.57e-3 --kd 0.015 --omega-a 90 --zeta 0.8
"""

import argparse
import math

from _artifacts import Run


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--J", type=float, required=True,
                   help="reflected inertia Jbar [Nm/(rev/s^2)], motor frame")
    p.add_argument("--place", action="store_true",
                   help="full-cubic pole placement: place a dominant pair "
                        "(zeta, --bw) and a real pole at --real-pole-mult*w_n; "
                        "kd comes out of the placement and is checked against "
                        "--kd-ceiling")
    p.add_argument("--real-pole-mult", type=float, default=5.0,
                   help="[--place] real pole at m*w_n; larger m = faster, more "
                        "non-dominant real pole (default 5)")
    p.add_argument("--kd-ceiling", type=float, default=0.07,
                   help="[--place] noise ceiling kd must stay below (default 0.07)")
    p.add_argument("--kd", type=float, default=None,
                   help="derivative gain (Moteus per-rev units) fixed at the "
                        "velocity-noise ceiling k_d,max from sweep_kd.py "
                        "(heuristic mode; not used with --place)")
    p.add_argument("--omega-a", type=float, default=None,
                   help="belt antiresonance w_a [rad/s] from identify_frf.py "
                        "(rigid-body assumption holds below it)")
    p.add_argument("--bw", type=float, default=None,
                   help="explicit target bandwidth w_n [rad/s] (overrides auto)")
    p.add_argument("--bw-frac-omega-a", type=float, default=0.5,
                   help="w_n as a fraction of w_a if --bw not given (default 0.5)")
    p.add_argument("--zeta", type=float, default=0.8,
                   help="target damping ratio (default 0.8)")
    p.add_argument("--fc", type=float, default=200.0,
                   help="current-loop bandwidth [Hz] (default 200)")
    p.add_argument("--cascade-frac", type=float, default=0.2,
                   help="w_n cap as fraction of current-loop bw (default 0.2)")
    p.add_argument("--routh-margin", type=float, default=0.7,
                   help="keep w_n at this fraction of the Routh ceiling "
                        "2*zeta*kd/Jbar (default 0.7, i.e. 30%% margin)")
    p.add_argument("--ratio", type=float, default=None,
                   help="actual transmission ratio i = wm/wL for output-frame "
                        "rescale (sets rotor_to_output_ratio = 1/i)")
    p.add_argument("--target", type=int, default=None,
                   help="moteus CAN id (1=drum, 2=horn), for the artifact label")
    p.add_argument("--label", default=None,
                   help="optional tag appended to the artifact folder name")
    args = p.parse_args()

    J = args.J

    # ---------------- full-cubic pole-placement mode ----------------
    if args.place:
        if args.bw is None:
            p.error("--place requires --bw (dominant pair frequency w_n [rad/s])")
        w_n = args.bw
        zeta = args.zeta
        m = args.real_pole_mult

        kd = J * w_n * (2 * zeta + m)
        kp = J * w_n ** 2 * (1 + 2 * zeta * m)
        ki = J * w_n ** 3 * m
        p_real = m * w_n

        print("Full-cubic pole placement "
              "(s^2 + 2*zeta*w_n*s + w_n^2)(s + p):")
        print(f"  dominant pair : zeta = {zeta},  w_n = {w_n:.3f} rad/s "
              f"({w_n/(2*math.pi):.3f} Hz)")
        print(f"  real pole     : p   = {p_real:.3f} rad/s "
              f"({p_real/(2*math.pi):.3f} Hz)  = {m:g}*w_n")
        print(f"\nResulting gains (motor frame, per-rev units):")
        print(f"  kp = {kp:.5f}   ki = {ki:.5f}   kd = {kd:.6f}")
        kd_ok = kd <= args.kd_ceiling
        print(f"\n  kd = {kd:.5f}  vs noise ceiling {args.kd_ceiling:.5f}  "
              f"-> {'OK' if kd_ok else 'EXCEEDS CEILING'}")
        if not kd_ok:
            w_max = args.kd_ceiling / (J * (2 * zeta + m))
            print(f"  lower --bw to <= {w_max:.3f} rad/s "
                  f"({w_max/(2*math.pi):.3f} Hz), raise --zeta, or lower "
                  f"--real-pole-mult to fit under the ceiling.")

        frame = "motor frame (rotor_to_output_ratio = 1)"
        ratio_line = None
        if args.ratio is not None:
            i = args.ratio
            kp *= i ** 2
            ki *= i ** 2
            kd *= i ** 2
            frame = f"output frame (rescaled by i^2 = {i**2:.4f})"
            ratio_line = f"conf set motor_position.rotor_to_output_ratio {1.0/i:.6f}"
            print(f"\nRescaled to {frame}:")
            print(f"  kp = {kp:.5f}   ki = {ki:.5f}   kd = {kd:.6f}")

        print("\nApply with:")
        if ratio_line:
            print(f"  {ratio_line}")
        print(f"  conf set servo.pid_position.kp {kp:.5f}")
        print(f"  conf set servo.pid_position.ki {ki:.5f}")
        print(f"  conf set servo.pid_position.kd {kd:.6f}")

        run = Run("position_gains", target=args.target, label=args.label)
        run.set_meta(
            mode="pole_placement", J_Nm_per_revs2=J,
            zeta=zeta, omega_n_rad_s=w_n, omega_n_hz=w_n / (2 * math.pi),
            real_pole_mult=m, real_pole_rad_s=p_real,
            kd_ceiling=args.kd_ceiling, kd_ok=bool(kd_ok),
            ratio_i=args.ratio, frame=frame, kp=kp, ki=ki, kd=kd,
        )
        run.finish()
        return

    # ---------------- kd-first heuristic mode ----------------
    if args.kd is None:
        p.error("heuristic mode requires --kd (or use --place)")
    kd = args.kd                       # fixed at the noise ceiling, per-rev units
    w_cascade = args.cascade_frac * 2 * math.pi * args.fc
    # Routh-Hurwitz ceiling on the cubic: ki < kp*kd/Jbar  <=>  w_n < 2*zeta*kd/J
    w_routh = args.routh_margin * 2 * args.zeta * kd / J

    # choose w_n: the smallest of the active upper bounds
    bounds = {"cascade (frac*2pi*fc)": w_cascade,
              "Routh (margin*2*zeta*kd/J)": w_routh}
    if args.bw is not None:
        w_n = args.bw
        bounds["explicit --bw"] = args.bw
    else:
        candidates = [w_cascade, w_routh]
        if args.omega_a is not None:
            w_belt = args.bw_frac_omega_a * args.omega_a
            bounds["belt (frac*omega_a)"] = w_belt
            candidates.append(w_belt)
        w_n = min(candidates)

    print("Bandwidth bounds [rad/s]:")
    for k, v in bounds.items():
        print(f"  {k:28s} = {v:8.2f}  ({v/(2*math.pi):6.2f} Hz)")
    print(f"  --> chosen w_n            = {w_n:8.2f}  ({w_n/(2*math.pi):6.2f} Hz), "
          f"zeta = {args.zeta}")
    if args.omega_a is not None and w_n > args.omega_a:
        print("  WARNING: w_n is above the antiresonance w_a -- the rigid-body "
              "assumption no longer holds; expect to excite the belt resonance.")

    # kd-first gain selection (eq. pos-gains-rigid), Moteus per-rev units
    kp = 2 * args.zeta * w_n * kd
    ki = w_n ** 2 * kd

    # verify the cubic Routh-Hurwitz bound ki < kp*kd/Jbar
    ki_ceiling = kp * kd / J
    routh_ok = ki < ki_ceiling
    print(f"\nRouth-Hurwitz check (cubic Jbar*s^3 + kd*s^2 + kp*s + ki):")
    print(f"  ki = {ki:.5f}  must be <  kp*kd/Jbar = {ki_ceiling:.5f}  "
          f"-> {'OK' if routh_ok else 'VIOLATED'}")
    if not routh_ok:
        print("  (lower w_n or raise --routh-margin; the integral gain is the "
              "binding constraint here.)")

    frame = "motor frame (rotor_to_output_ratio = 1)"
    ratio_line = None
    if args.ratio is not None:
        i = args.ratio
        kp *= i ** 2
        ki *= i ** 2
        kd *= i ** 2
        frame = f"output frame (rescaled by i^2 = {i**2:.4f})"
        ratio_line = f"conf set motor_position.rotor_to_output_ratio {1.0/i:.6f}"

    print(f"\nMoteus position-loop gains -- {frame}:")
    print(f"  kp = {kp:.5f}   ki = {ki:.5f}   kd = {kd:.6f}")
    print("\nApply with:")
    if ratio_line:
        print(f"  {ratio_line}")
    print(f"  conf set servo.pid_position.kp {kp:.5f}")
    print(f"  conf set servo.pid_position.ki {ki:.5f}")
    print(f"  conf set servo.pid_position.kd {kd:.6f}")
    print(f"  conf write")
    print("\nNote: tune in the motor frame first (no --ratio). Re-run with "
          "--ratio i once measure_belt_slip.py has the ratio, to get the "
          "output-frame gains -- the i^2 rescale preserves the tuned response.")

    run = Run("position_gains", target=args.target, label=args.label)
    run.set_meta(
        J_Nm_per_revs2=J, kd_ceiling=args.kd, omega_a_rad_s=args.omega_a,
        omega_n_rad_s=w_n, omega_n_hz=w_n / (2 * math.pi), zeta=args.zeta,
        fc_hz=args.fc, w_routh_rad_s=w_routh, routh_margin=args.routh_margin,
        ki_ceiling=ki_ceiling, routh_ok=bool(routh_ok),
        ratio_i=args.ratio, frame=frame,
        kp=kp, ki=ki, kd=kd,
    )
    run.finish()


if __name__ == "__main__":
    main()
