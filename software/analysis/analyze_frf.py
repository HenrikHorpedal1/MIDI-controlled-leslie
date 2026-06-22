#!/usr/bin/env python3
"""Re-plot an identify_frf.py CSV as position/torque (theta_m/T_m).

The sweep measures velocity/torque (d theta_m / T_m).  Dividing by s = j*omega
gives position/torque, which is the form Gravdahl/Egeland plot:

    theta_m / T_m = (1 / (j*2*pi*f)) * (d theta_m / T_m)

i.e. magnitude -20 dB/dec steeper, phase shifted -90 deg.  Pure post-processing
of the saved frf.csv -- no hardware.

    uv run python analysis/analyze_frf.py <frf.csv> [--no-show]

With --fit, a two-mass belt model is least-squares fit to the complex
velocity/torque FRF, giving continuous (off-grid) estimates of the reflected
inertia Jbar, the antiresonance (w_a, zeta_a) and resonance (w_1, zeta_1),
plus a goodness-of-fit residual -- instead of reading the extrema off the grid.
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def two_mass_model(f, p):
    """Velocity/torque FRF of a two-mass (motor-belt-load) drive.

        Hv(jw) = 1/(Jbar*s) * (1 + 2*za*s/wa + (s/wa)^2)
                            / (1 + 2*z1*s/w1 + (s/w1)^2),   s = j*2*pi*f

    DC-normalised so the numerator/denominator are unity at s->0: the low-f
    asymptote is exactly 1/(Jbar*w), so Jbar is the low-frequency (rigid, both
    masses together) reflected inertia.  The numerator zero pair is the
    antiresonance (wa, za), the denominator pole pair the resonance (w1, z1);
    the high-f asymptote rolls off as 1/(Jbar*(wa/w1)^2*w) -- the motor-only
    inertia.  p = [Jbar, wa, za, w1, z1] with wa, w1 in rad/s.
    """
    Jbar, wa, za, w1, z1 = p
    s = 1j * 2 * np.pi * f
    num = 1 + 2 * za * s / wa + (s / wa) ** 2
    den = 1 + 2 * z1 * s / w1 + (s / w1) ** 2
    return num / (Jbar * s * den)


def fit_two_mass(f, Hv):
    """Fit two_mass_model to the measured FRF *magnitude*.  Returns (params, info).

    The fit is done on log-magnitude (dB), not the complex value: the stepped-
    sine measurement carries a constant phase-convention offset (the inertia
    1/(Jbar*s) branch should read -90 deg at low f but the data sits near
    +150 deg), so a complex fit wastes itself rotating phase.  dB-magnitude
    fitting is the standard Bode approach -- Jbar/wa/w1 come from the slope, dip
    and peak; the dampings from dip-depth and peak-height.  Phase is left as an
    independent visual cross-check.  Seeds: low-f inertia asymptote + extrema.
    """
    from scipy.optimize import least_squares

    mag = np.abs(Hv)
    mag_db = 20 * np.log10(mag)
    w = 2 * np.pi * f

    lo = np.argsort(f)[: max(3, len(f) // 6)]
    Jbar0 = float(np.median(1.0 / (w[lo] * mag[lo])))
    wa0 = float(w[np.argmin(mag)])
    w10 = float(w[np.argmax(mag)])
    p0 = [Jbar0, wa0, 0.05, w10, 0.05]

    wmin, wmax = w.min(), w.max()
    # the resonance may sit at/just past the top of the sweep, so allow w1 to
    # float modestly above the band edge (its downslope then constrains it only
    # weakly -- flagged via the residual / a w1 near 3*wmax is under-determined).
    bounds = ([1e-9, wmin, 1e-3, wmin, 1e-3],
              [1e3, wmax, 2.0, 3 * wmax, 2.0])

    def residual(p):
        return 20 * np.log10(np.abs(two_mass_model(f, p))) - mag_db

    sol = least_squares(residual, p0, bounds=bounds)
    rms_db = float(np.sqrt(np.mean(sol.fun**2)))   # rms magnitude residual [dB]
    return sol.x, {"rms_db": rms_db, "p0": p0, "success": sol.success}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", help="frf.csv from identify_frf.py")
    p.add_argument("--f-max", type=float, default=None,
                   help="only plot up to this frequency [Hz] (drop high-f artifact)")
    p.add_argument("--fit", action="store_true",
                   help="fit a two-mass belt model to the complex FRF and report "
                        "Jbar, antiresonance/resonance (with damping) + residual")
    p.add_argument("--fix-phase", action="store_true",
                   help="correct the mirrored phase of CSVs captured before the "
                        "identify_frf.py phasor fix (swaps re/im; magnitude was "
                        "already correct)")
    p.add_argument("--unwrap", action="store_true",
                   help="continuously unwrap the phase instead of the default "
                        "Gravdahl branch (which wraps into (-360, 0] so the low-f "
                        "double-integrator asymptote sits at -180 deg)")
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--output", "-o", type=str, default=None,
                   help="output PNG path (default: frf_position_bode.png next to CSV)")
    args = p.parse_args()

    rows = list(csv.DictReader(open(args.csv, newline="")))
    f = np.array([float(r["f_hz"]) for r in rows])
    Hv = np.array([float(r["re"]) + 1j * float(r["im"]) for r in rows])

    if args.fix_phase:
        # legacy bug: stored H = a-j*b instead of b-j*a, i.e. phase mirrored to
        # 90deg-phase_true with magnitude intact.  H_true = j*conj(H) = im+j*re.
        Hv = Hv.imag + 1j * Hv.real

    if args.f_max is not None:
        keep = f <= args.f_max
        f, Hv = f[keep], Hv[keep]

    fit_p = None
    if args.fit:
        fit_p, info = fit_two_mass(f, Hv)
        Jbar, wa, za, w1, z1 = fit_p
        print("Two-mass model fit (velocity/torque):")
        print(f"  Jbar           = {Jbar:.6e}  [Nm/(rev/s^2)]")
        print(f"  antiresonance  w_a = {wa/(2*np.pi):8.3f} Hz "
              f"({wa:7.2f} rad/s)   zeta_a = {za:.4f}")
        print(f"  resonance      w_1 = {w1/(2*np.pi):8.3f} Hz "
              f"({w1:7.2f} rad/s)   zeta_1 = {z1:.4f}")
        print(f"  rms magnitude residual = {info['rms_db']:.3f} dB"
              f"   (converged: {info['success']})")
        grid_a = f[np.argmin(np.abs(Hv))]
        grid_1 = f[np.argmax(np.abs(Hv))]
        print(f"  (grid extrema for comparison: w_a~{grid_a:.2f} Hz, "
              f"w_1~{grid_1:.2f} Hz)")

    # position/torque = velocity/torque divided by s = j*2*pi*f
    Hp = Hv / (1j * 2 * np.pi * f)

    def disp_phase(H):
        """Phase [deg] for display.  Default: Gravdahl branch -- wrap into
        (-360, 0] so the low-f double-integrator asymptote reads -180 deg and the
        antiresonance/resonance bump lifts toward 0.  --unwrap: continuous."""
        if args.unwrap:
            return np.degrees(np.unwrap(np.angle(H)))
        ph = np.degrees(np.angle(H))
        return np.where(ph > 0, ph - 360, ph)

    mag_v = 20 * np.log10(np.abs(Hv))
    mag_p = 20 * np.log10(np.abs(Hp))
    ph_p = disp_phase(Hp)

    _, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True)
    ax1.set_title(r"$\theta_m / T_m$")
    ax1.semilogx(f, mag_p, "o-", label="measured")
    ax1.set_ylabel("magnitude  [dB]")
    ax1.grid(True, which="both")
    ax2.semilogx(f, ph_p, "o-")
    ax2.set_ylabel("phase  [deg]")
    ax2.set_xlabel("frequency  [Hz]")
    ax2.grid(True, which="both")

    if fit_p is not None:
        ff = np.logspace(np.log10(f.min()), np.log10(f.max()), 400)
        Hp_fit = two_mass_model(ff, fit_p) / (1j * 2 * np.pi * ff)
        ax1.semilogx(ff, 20 * np.log10(np.abs(Hp_fit)), "r-", lw=1.5,
                     label="two inertia fit")
        ax2.semilogx(ff, disp_phase(Hp_fit), "r-", lw=1.5)
        wa, w1 = fit_p[1], fit_p[3]
        fa, f1 = wa / (2 * np.pi), w1 / (2 * np.pi)
        ax1.axvline(fa, color="darkorange", ls="--", lw=1.8, alpha=0.85,
                    label=rf"$\omega_a$ = {fa:.1f} Hz")
        ax1.axvline(f1, color="seagreen",   ls="--", lw=1.8, alpha=0.85,
                    label=rf"$\omega_1$ = {f1:.1f} Hz")
        ax2.axvline(fa, color="darkorange", ls="--", lw=1.8, alpha=0.85)
        ax2.axvline(f1, color="seagreen",   ls="--", lw=1.8, alpha=0.85)
        ax1.legend(fontsize=8)

    plt.tight_layout()

    out = Path(args.output) if args.output else Path(args.csv).with_name("frf_position_bode.svg")
    plt.savefig(out)
    print(f"Plot saved -> {out}")
    print(f"  (velocity-form |H| at first point: {mag_v[0]:.1f} dB; "
          f"position-form: {mag_p[0]:.1f} dB)")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
