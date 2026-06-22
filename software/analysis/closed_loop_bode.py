#!/usr/bin/env python3
"""Plant / open-loop / closed-loop Bode for the belt drive + moteus PID.

Pairs with analyze_frf.py, which *fits* the two-mass plant

    P(s) = theta_m / T_m = (1/(Jbar*s)) * (1 + 2 za s/wa + (s/wa)^2)
                                         / (1 + 2 z1 s/w1 + (s/w1)^2)

from a stepped-sine sweep.  This takes those five identified numbers plus the
moteus position-loop gains and shows what the loop does with them.  The moteus
position controller commands torque from position/velocity error,

    C(s) = kp + ki/s + kd*s          [Nm per rev, per rev, per rev/s]

so L = C*P (dimensionless) and T = L/(1+L).  Units are moteus-native (rev, Nm)
-- the frame the gains live in (rotor_to_output_ratio = 1, belt ratio handled
upstream in the ESP32), so no scaling here.  python-control does the Bode and
margins; this script is just plant-building + the 3-curve overlay.

    uv run python analysis/closed_loop_bode.py --meta <frf meta.json> \
        --kp 0.0237 --ki 0.0080 --kd 0.0230

--meta reads Jbar/w_a/w_1 from an identify_frf.py meta.json and fits the
dampings za/z1 from the sibling frf.csv; or pass --jbar/--wa/--za/--w1/--z1.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import control as ct
import matplotlib.pyplot as plt


def plant_tf(jbar, wa, za, w1, z1):
    """Two-mass position/torque transfer function (DC-normalised).

    theta_m/T_m = 1/(Jbar s^2) * (num/den): a *double* integrator -- the moteus
    position loop's C(s) acts on position error, so the loop plant is
    position/torque, not the velocity/torque 1/(Jbar s) the FRF sweep measures.
    """
    num = [1 / wa**2, 2 * za / wa, 1]
    den_res = [1 / w1**2, 2 * z1 / w1, 1]
    den = np.polymul(den_res, [jbar, 0, 0])  # * Jbar*s^2 (position/torque)
    return ct.tf(num, den)


def fit_complex_sk(f, Hv, iters=20):
    """Independent two-mass fit of the *velocity/torque* FRF, as a cross-check
    on analyze_frf.fit_two_mass (which fits log-magnitude, nonlinearly).

    This one is a *complex, linear* least-squares fit -- Levi's method with
    Sanathanan-Koerner reweighting (rows / |D_prev| each pass, otherwise the
    w^3 denominator over-weights high f and the inertia comes out negative).
    Model Hv = N/D with N = 1 + c1 s + c2 s^2 (DC-normalised), D = d1 s + d2 s^2
    + d3 s^3 (pole at the origin = rigid-body integrator).  Returns a dict; wa/za
    are flagged None when the antiresonance term c2 <= 0 (ill-conditioned: a
    shallow notch with few points does not pin the numerator zeros down).
    """
    s = 1j * 2 * np.pi * f
    weight = np.ones_like(f, dtype=float)
    c1 = c2 = d1 = d2 = d3 = 0.0
    for _ in range(max(1, iters)):
        A = np.column_stack([s, s**2, -Hv * s, -Hv * s**2, -Hv * s**3]) / weight[:, None]
        rhs = -np.ones_like(s) / weight
        x, *_ = np.linalg.lstsq(np.vstack([A.real, A.imag]),
                                np.concatenate([rhs.real, rhs.imag]), rcond=None)
        c1, c2, d1, d2, d3 = x
        weight = np.maximum(np.abs(d1 * s + d2 * s**2 + d3 * s**3), 1e-12)

    jbar, w1 = d1, np.sqrt(d1 / d3)
    z1 = d2 * w1 / (2 * d1)
    if c2 > 0:
        wa = 1 / np.sqrt(c2)
        za = c1 * wa / 2
    else:
        wa = za = None
    return dict(jbar=jbar, wa=wa, za=za, w1=w1, z1=z1)


def _load_frf(csv_path):
    import csv
    rows = list(csv.DictReader(open(csv_path, newline="")))
    f = np.array([float(r["f_hz"]) for r in rows])
    Hv = np.array([float(r["re"]) + 1j * float(r["im"]) for r in rows])
    return f, Hv


def run_fit(args, p):
    """--fit: compare three independent estimates of the plant params."""
    import sys
    sys.path.insert(0, str(Path(__file__).parent))
    import analyze_frf

    if args.csv:
        csv_path = Path(args.csv)
    elif args.meta:
        csv_path = Path(args.meta).with_name("frf.csv")
    else:
        p.error("--fit needs --csv or --meta to locate frf.csv")
    f, Hv = _load_frf(csv_path)

    def hz(x):
        return f"{x/2/np.pi:8.3f}" if x is not None else "    n/a "

    sk = fit_complex_sk(f, Hv)
    mp, _ = analyze_frf.fit_two_mass(f, Hv)            # Jbar, wa, za, w1, z1 (rad/s)

    print(f"cross-check of plant params  ({csv_path})")
    print(f"{'method':<34}{'Jbar':>12}{'wa[Hz]':>10}{'za':>9}"
          f"{'w1[Hz]':>10}{'z1':>9}")
    print(f"{'analyze_frf (mag, nonlinear)':<34}{mp[0]:12.4e}{mp[1]/2/np.pi:10.3f}"
          f"{mp[2]:9.4f}{mp[3]/2/np.pi:10.3f}{mp[4]:9.4f}")
    print(f"{'this (complex LS, Levi/SK)':<34}{sk['jbar']:12.4e}{hz(sk['wa']):>10}"
          f"{(f'{sk['za']:.4f}' if sk['za'] is not None else 'n/a'):>9}"
          f"{sk['w1']/2/np.pi:10.3f}{sk['z1']:9.4f}")
    if args.meta:
        m = json.loads(Path(args.meta).read_text())
        print(f"{'meta.json (grid extrema)':<34}{m['Jbar_Nm_per_revs2']:12.4e}"
              f"{m['antiresonance_wa_hz']:10.3f}{'n/a':>9}"
              f"{m['resonance_w1_hz']:10.3f}{'n/a':>9}")
    if sk["wa"] is None:
        print("\nNOTE: complex fit could not pin the antiresonance (numerator "
              "term <= 0): the notch is shallow / sparsely sampled, so w_a/z_a are "
              "weakly identified. Jbar and the resonance agreeing across methods is "
              "the meaningful confirmation.")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fit", action="store_true",
                   help="SECOND RUN MODE: don't plot the loop -- instead re-fit the "
                        "two-mass model from the FRF with an independent complex "
                        "least-squares (Levi/SK) method and print it next to "
                        "analyze_frf's magnitude fit + the meta grid extrema, as a "
                        "cross-check on the plant params. Uses --csv or --meta's "
                        "sibling frf.csv. Does NOT touch frf_position_bode.")
    p.add_argument("--csv", help="frf.csv for --fit (else taken from --meta's dir)")
    p.add_argument("--meta", help="identify_frf.py meta.json (reads Jbar/w_a/w_1, "
                   "fits za/z1 from sibling frf.csv)")
    p.add_argument("--jbar", type=float, help="reflected inertia [Nm/(rev/s^2)]")
    p.add_argument("--wa", type=float, help="antiresonance [rad/s, or Hz with --hz]")
    p.add_argument("--za", type=float, help="antiresonance damping")
    p.add_argument("--w1", type=float, help="resonance [rad/s, or Hz with --hz]")
    p.add_argument("--z1", type=float, help="resonance damping")
    p.add_argument("--kp", type=float, help="(required unless --fit)")
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--hz", action="store_true", help="--wa/--w1 are in Hz")
    p.add_argument("--f-min", type=float, default=0.1)
    p.add_argument("--f-max", type=float, default=500.0)
    p.add_argument("--out", help="figure path (default: closed_loop_bode.svg "
                   "beside --meta, else cwd)")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    if args.fit:
        run_fit(args, p)
        return
    if args.kp is None:
        p.error("--kp is required (omit only with --fit)")

    jbar, wa, za, w1, z1 = args.jbar, args.wa, args.za, args.w1, args.z1
    out_dir = Path.cwd()
    if args.meta:
        m = Path(args.meta)
        out_dir = m.parent
        meta = json.loads(m.read_text())
        jbar = jbar if jbar is not None else meta["Jbar_Nm_per_revs2"]
        wa = wa if wa is not None else meta["antiresonance_wa_rad_s"]
        w1 = w1 if w1 is not None else meta["resonance_w1_rad_s"]
        args.hz = args.hz and (args.wa is not None)  # meta values are rad/s
        if za is None or z1 is None:
            import sys
            sys.path.insert(0, str(Path(__file__).parent))
            import analyze_frf
            import csv
            rows = list(csv.DictReader(open(m.with_name("frf.csv"), newline="")))
            f = np.array([float(r["f_hz"]) for r in rows])
            Hv = np.array([float(r["re"]) + 1j * float(r["im"]) for r in rows])
            fp, _ = analyze_frf.fit_two_mass(f, Hv)
            za = za if za is not None else fp[2]
            z1 = z1 if z1 is not None else fp[4]
            print(f"fitted dampings from frf.csv: za={za:.4f}, z1={z1:.4f}")

    miss = [n for n, v in dict(jbar=jbar, wa=wa, za=za, w1=w1, z1=z1).items()
            if v is None]
    if miss:
        p.error(f"missing plant params {miss} -- pass them or use --meta")
    if args.hz:
        wa, w1 = wa * 2 * np.pi, w1 * 2 * np.pi

    P = plant_tf(jbar, wa, za, w1, z1)
    C = ct.tf([args.kd, args.kp, args.ki], [1, 0])   # kd*s + kp + ki/s
    L = C * P
    T = ct.feedback(L, 1)

    gm, pm, wcg, wcp = ct.margin(L)   # gm linear, pm deg, wcg/wcp rad/s
    print(f"phase margin = {pm:6.1f} deg  @ {wcp/2/np.pi:7.3f} Hz" if not
          np.isnan(pm) else "phase margin = (no gain crossover)")
    print(f"gain margin  = {ct.mag2db(gm):6.1f} dB   @ {wcg/2/np.pi:7.3f} Hz" if not
          np.isnan(gm) else "gain margin  = (phase never reaches -180 in band)")

    w = 2 * np.pi * np.logspace(np.log10(args.f_min), np.log10(args.f_max), 2000)
    f = w / (2 * np.pi)
    Pr, Lr, Tr = (sys(1j * w) for sys in (P, L, T))
    peak = ct.mag2db(np.abs(Tr)).max()
    fpk = f[np.argmax(np.abs(Tr))]
    bw = ct.bandwidth(T) / (2 * np.pi)
    print(f"closed-loop peak Mp = {peak:.2f} dB @ {fpk:.2f} Hz;  "
          f"-3 dB bandwidth = {bw:.2f} Hz")

    _, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7.5), sharex=True)
    for r, lbl in ((Pr, "plant $P$"), (Lr, "open loop $L=CP$"),
                   (Tr, "closed loop $T=L/(1{+}L)$")):
        ax1.semilogx(f, ct.mag2db(np.abs(r)), lw=1.4, label=lbl)
        ax2.semilogx(f, np.degrees(np.unwrap(np.angle(r))), lw=1.4)
    ax1.axhline(0, color="k", lw=0.6)
    ax2.axhline(-180, color="r", ls="--", lw=0.8)
    for ax in (ax1, ax2):
        ax.axvline(wa / 2 / np.pi, color="orange", ls="--", lw=1)
        ax.axvline(w1 / 2 / np.pi, color="green", ls="--", lw=1)
        ax.grid(True, which="both")
    ax1.set_ylabel("magnitude  [dB]")
    ax1.legend(fontsize=8)
    ax2.set_ylabel("phase  [deg]")
    ax2.set_xlabel("frequency  [Hz]")
    ax1.set_title(
        f"PM {pm:.0f}°  GM {ct.mag2db(gm):.0f} dB  Mp {peak:.1f} dB  BW {bw:.0f} Hz",
        fontsize=10)

    plt.tight_layout()
    out = Path(args.out) if args.out else out_dir / "closed_loop_bode.svg"
    plt.savefig(out)
    print(f"Plot saved -> {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
