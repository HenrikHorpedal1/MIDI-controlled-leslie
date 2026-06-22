#!/usr/bin/env python3
"""Compute the slip-estimator low-pass time constant from the MA600 eccentricity.

The load-side MA600 (an off-axis encoder) feeds the slip estimate of the thesis
(eq. slip-lowpass): a FIRST-order low-pass with time constant tau acting on the
residual theta_s,raw = theta_m/i_hat - theta_L,meas.  The encoder's dominant error
is a once-per-load-revolution sinusoid from mounting eccentricity (plus INL),
measured by compensate_encoder_velocity.py.  That ripple sits at the LOAD rotation
frequency f_ecc = omega_L [Hz]; at the drum's operating speeds this is only a few
Hz, the same band as the slip dynamics we want to observe -- so tau must be slow
enough to reject the eccentricity yet fast enough not to lag the true slip.

This is a pure calculator (no hardware).  A first-order low-pass with corner
f_c = 1/(2*pi*tau) attenuates a tone at f_ecc (>> f_c) by roughly f_c/f_ecc in
amplitude (-20 dB/dec).  Given the measured eccentricity amplitude and a tolerable
residual velocity ripple, it solves for the f_c (hence tau) that brings the
eccentricity-induced ripple under tolerance, and checks that f_c still clears the
slip bandwidth.

    uv run python moteus-config/scripts/compute_slip_filter.py \\
        --ecc-amp-rev 0.0008 --load-speed-min 0.5 --ripple-tol-revs 0.01
"""

import argparse
import math

from _artifacts import Run


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ecc-amp-rev", type=float, required=True,
                   help="eccentricity position-error amplitude [rev], peak, from "
                        "compensate_encoder_velocity.py (the 1/rev component)")
    p.add_argument("--load-speed-min", type=float, default=0.5,
                   help="lowest steady LOAD speed of interest [rev/s] = lowest "
                        "f_ecc [Hz] (worst case for rejection)")
    p.add_argument("--load-speed-max", type=float, default=2.0,
                   help="highest steady LOAD speed of interest [rev/s]")
    p.add_argument("--ripple-tol-revs", type=float, default=0.01,
                   help="tolerable residual LOAD velocity ripple [rev/s]")
    p.add_argument("--slip-bw", type=float, default=0.2,
                   help="slip-dynamics bandwidth to preserve [Hz] (f_c must "
                        "exceed this)")
    p.add_argument("--target", type=int, default=None,
                   help="moteus CAN id (for the artifact label)")
    p.add_argument("--label", default=None,
                   help="optional tag appended to the artifact folder name")
    args = p.parse_args()

    # At load speed w (rev/s) the eccentricity tone is at f_ecc = w [Hz] and its
    # contribution to the VELOCITY estimate has amplitude
    #   v_ripple_in = 2*pi*f_ecc * ecc_amp          [rev/s]
    # A FIRST-order low-pass at corner f_c attenuates a tone at f_ecc (>> f_c) by
    #   |H| ~ f_c / f_ecc        (-20 dB/dec).
    # Require v_ripple_in * (f_c/f_ecc) <= tol at the WORST speed, then also at
    # the top speed -- worst case is whichever speed gives the larger required
    # attenuation; we solve at both and take the tighter (smaller) f_c.
    speeds = [args.load_speed_min, args.load_speed_max]
    fc_candidates = []
    print("Eccentricity-driven velocity ripple before filtering:")
    for w in speeds:
        f_ecc = w
        v_in = 2 * math.pi * f_ecc * args.ecc_amp_rev
        # need (f_c/f_ecc) <= tol / v_in  ->  f_c <= f_ecc * (tol/v_in)
        if v_in <= args.ripple_tol_revs:
            print(f"  {w:5.2f} rev/s : {v_in:.5f} rev/s already under tolerance")
            continue
        fc = f_ecc * (args.ripple_tol_revs / v_in)
        fc_candidates.append(fc)
        print(f"  {w:5.2f} rev/s : {v_in:.5f} rev/s  -> needs f_c <= {fc:.3f} Hz")

    if not fc_candidates:
        print("\nEccentricity ripple is already within tolerance at all speeds; "
              "set the slip filter from the slip bandwidth alone.")
        fc = max(5 * args.slip_bw, args.slip_bw)
    else:
        fc = min(fc_candidates)

    tau = 1.0 / (2 * math.pi * fc)
    print(f"\nRecommended slip-filter corner f_c = {fc:.3f} Hz")
    print(f"  -> first-order time constant  tau = {tau:.4f} s  (eq. slip-lowpass)")
    if fc <= args.slip_bw:
        print(f"  WARNING: f_c = {fc:.3f} Hz does not clear the slip bandwidth "
              f"({args.slip_bw} Hz).  The eccentricity and slip dynamics overlap; "
              f"reduce eccentricity (re-seat/trim the MA600) or accept a slower "
              f"slip estimate.")
    else:
        print(f"  clears the slip bandwidth ({args.slip_bw} Hz) by "
              f"{fc/args.slip_bw:.1f}x.")
    print(f"\nApply as the slip-estimator low-pass time constant tau = {tau:.4f} s "
          f"in the ESP32 motion controller.")

    run = Run("slip_filter", target=args.target, label=args.label)
    run.set_meta(
        ecc_amp_rev=args.ecc_amp_rev,
        load_speed_range_revs=[args.load_speed_min, args.load_speed_max],
        ripple_tol_revs=args.ripple_tol_revs, slip_bw_hz=args.slip_bw,
        slip_filter_fc_hz=fc, slip_filter_tau_s=tau,
        clears_slip_bw=bool(fc > args.slip_bw),
    )
    run.finish()


if __name__ == "__main__":
    main()
