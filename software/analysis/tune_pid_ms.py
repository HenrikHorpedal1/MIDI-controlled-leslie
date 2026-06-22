#!/usr/bin/env python3
"""Ms-constrained PID tuning for the belt drive (MIGO-style).

Companion to closed_loop_bode.py.  That script *evaluates* a hand-picked
(kp, ki, kd); this one *solves* for them: it picks the moteus position-loop
gains that maximise integral gain ki (= low-frequency disturbance rejection)
subject to a sensitivity-peak bound

    Ms = max_w |S(jw)| <= --ms          S = 1/(1+L)

which is the single robustness metric that folds in phase margin, gain margin
*and* the belt resonance at once (Ms = 1/closest approach of L to -1).  This
replaces the "set wc = w_a/2 and check 6 dB GM afterwards" hand procedure.

The loop seen by the optimiser is L = G * plant_tf, where G captures the
controller AND the moteus encoder PLL.  The PLL is critically damped (zeta=1,
double pole at -w_n) and pll_filter_hz is its 3 dB bandwidth, so

    w_n = 2*pi * pll_filter_hz / 2.48

The two estimator outputs have DIFFERENT transfer functions, and kp/ki act on
the position estimate while kd acts on the velocity estimate:

    F_pos = (2 w_n s + w_n^2)/(s+w_n)^2     (lead zero)   <- kp, ki
    F_vel =          w_n^2 /(s+w_n)^2       (no zero)      <- kd

The FRF sweep records the *velocity* estimate, so the identified plant is
plant_tf = F_vel,capture * P_true.  De-embedding that, the open loop is

    L = [ (kp+ki/s) F_pos,des + kd s F_vel,des ] / F_vel,cap * plant_tf

When design==capture the F_vel on the kd path cancels (derivative sees the raw
plant) and the PI path keeps an extra (1 + 2 s/w_n) lead.

ki is maximised (= low-frequency disturbance rejection) subject to:
  * Ms = max|S| <= --ms      single robustness metric, folds in PM/GM/resonance
  * Mt = max|T| <= --mt      optional, bounds noise/HF peaking (MIGO uses both)
  * closed-loop stability    (explicit pole check -- a grid |S| bound is blind
                              to Nyquist encirclements)
  * crossover <= --wc-max    keep the loop sub-resonance (default 0.5*w_a)
  * kd <= kd_cap(pll)        noise ceiling, scaled with PLL bandwidth:
        kd_cap = --kd-cap * (--kd-cap-bw / pll_hz) ** --noise-exp

Lowering pll_filter_hz cuts velocity noise (raises usable kd) but adds lag to
the whole loop, so it can be a decision variable (--optimize-pll).

Examples
--------
    # fixed PLL at the captured 200 Hz, solve kp/ki/kd under Ms<=1.6
    uv run python analysis/tune_pid_ms.py --meta <frf>/meta.json \
        --ms 1.6 --kd-cap 0.030 --kd-cap-bw 200 --pll 200 --capture-pll-bw 200

    # also let it pick pll_filter_hz in [30,400], bound Mt too
    uv run python analysis/tune_pid_ms.py --meta <frf>/meta.json \
        --ms 1.6 --mt 1.6 --kd-cap 0.030 --kd-cap-bw 200 --optimize-pll \
        --wp-min 30 --wp-max 400 --capture-pll-bw 200

Plant params come from --meta (Jbar/w_a/w_1, dampings fitted off the sibling
frf.csv exactly as closed_loop_bode does) or are passed explicitly with
--jbar/--wa/--za/--w1/--z1 (--hz if wa/w1 are in Hz).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import control as ct
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).parent))
from closed_loop_bode import plant_tf  # noqa: E402  (same plant, same units)


# --------------------------------------------------------------------------- #
# plant parameter loading (mirrors closed_loop_bode.main, kept compact)
# --------------------------------------------------------------------------- #
def load_params(args, p):
    jbar, wa, za, w1, z1 = args.jbar, args.wa, args.za, args.w1, args.z1
    if args.meta:
        m = Path(args.meta)
        meta = json.loads(m.read_text())
        jbar = jbar if jbar is not None else meta["Jbar_Nm_per_revs2"]
        wa = wa if wa is not None else meta["antiresonance_wa_rad_s"]
        w1 = w1 if w1 is not None else meta["resonance_w1_rad_s"]
        args.hz = args.hz and (args.wa is not None)  # meta is rad/s
        if za is None or z1 is None:
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
    return jbar, wa, za, w1, z1


# --------------------------------------------------------------------------- #
# frequency-domain pieces (vectorised, evaluated on a fixed w grid)
# --------------------------------------------------------------------------- #
# moteus encoder PLL: critically damped (zeta=1), double pole at -w_n.
#   position estimate:  F_pos = (2 w_n s + w_n^2)/(s+w_n)^2   (has a lead zero)
#   velocity estimate:  F_vel =          w_n^2 /(s+w_n)^2     (no zero)
# pll_filter_hz is the 3 dB bandwidth; the natural frequency is f_3dB/2.48.
PLL_BW_TO_WN = 2.48  # w_n = 2*pi * f_pll_hz / 2.48


def wn_of(f_pll_hz):
    return 2 * np.pi * f_pll_hz / PLL_BW_TO_WN


def open_resp(jw, kp, ki, kd, wn_des, wn_cap):
    """G(jw): controller * PLL paths, with the capture PLL de-embedded.

    The plant is identified as plant_tf = F_vel,cap * P_true (sweep measures the
    velocity estimate).  kp/ki act on the position estimate (F_pos), kd on the
    velocity estimate (F_vel), so the open loop is

        L = G * plant_tf,
        G = [ (kp+ki/s) F_pos,des + kd s F_vel,des ] / F_vel,cap

    Set wn_cap huge (--capture-pll-bw large) to treat the plant as de-embedded.
    """
    Fpos_d = (2 * wn_des * jw + wn_des**2) / (jw + wn_des) ** 2
    Fvel_d = wn_des**2 / (jw + wn_des) ** 2
    Fvel_c = wn_cap**2 / (jw + wn_cap) ** 2
    return ((kp + ki / jw) * Fpos_d + kd * jw * Fvel_d) / Fvel_c


def open_poly(kp, ki, kd, wn_des, wn_cap):
    """(numG, denG) of G above, as polynomials (for closed-loop pole roots).

        numG = (s+wc)^2 * [ (kp s+ki)(2 wd s + wd^2) + kd wd^2 s^2 ]
        denG = s * (s+wd)^2 * wc^2
    """
    wd, wc = wn_des, wn_cap
    swc2 = np.polymul([1.0, wc], [1.0, wc])
    bracket = np.polyadd(np.polymul([kp, ki], [2 * wd, wd**2]),
                         [kd * wd**2, 0.0, 0.0])
    numG = np.polymul(swc2, bracket)
    denG = np.polymul(np.polymul([1.0, 0.0], np.polymul([1.0, wd], [1.0, wd])),
                      [wc**2])
    return numG, denG


def loop_metrics(jw, Pw, kp, ki, kd, wn_des, wn_cap):
    """Return (Ms, Mt) on the grid for these params."""
    Lw = open_resp(jw, kp, ki, kd, wn_des, wn_cap) * Pw
    S = 1.0 / (1.0 + Lw)
    T = Lw * S
    return np.abs(S).max(), np.abs(T).max()


def kd_ceiling(wp_hz, cap, cap_bw, exp):
    return cap * (cap_bw / wp_hz) ** exp


def clpoles_maxreal(numP, denP, kp, ki, kd, wn_des, wn_cap):
    """Largest real part of the closed-loop poles (roots of 1 + G*plant_tf).

    A grid |S| bound is blind to Nyquist encirclements, so stability has to be
    enforced separately: this returns max Re(pole); < 0 means stable.
    """
    numG, denG = open_poly(kp, ki, kd, wn_des, wn_cap)
    char = np.polyadd(np.polymul(denG, denP), np.polymul(numG, numP))
    return float(np.real(np.roots(char)).max())


# --------------------------------------------------------------------------- #
# optimisation: MAX BANDWIDTH via pole placement.
# variables x = [log w_n (rad/s), zeta, m], gains from compute_position_gains:
#     kd = J w_n (2 zeta + m)
#     kp = J w_n^2 (1 + 2 zeta m)
#     ki = J w_n^3 m
# maximise w_n s.t. Ms<=ms, Mt<=mt, stability, kd<=cap, (optional crossover cap).
# Pole placement keeps kp/ki/kd balanced -- avoids the degenerate near-zero-PM
# solutions that a free-gain "maximise ki" objective drifts into.
# --------------------------------------------------------------------------- #
def pp_gains(J, w_n, zeta, m):
    kd = J * w_n * (2 * zeta + m)
    kp = J * w_n**2 * (1 + 2 * zeta * m)
    ki = J * w_n**3 * m
    return kp, ki, kd


def solve(jw, Pw, args, pll_hz, numP, denP, wc_max, wn_cap, jbar):
    w = np.abs(jw)
    wn_des = wn_of(pll_hz)

    def unpack(x):
        w_n, zeta, m = np.exp(x[0]), x[1], x[2]
        kp, ki, kd = pp_gains(jbar, w_n, zeta, m)
        return kp, ki, kd, pll_hz, w_n, zeta, m

    def stab_margin(x):
        kp, ki, kd, *_ = unpack(x)
        return -clpoles_maxreal(numP, denP, kp, ki, kd, wn_des, wn_cap)

    def neg_bw(x):
        # maximise w_n (= x[0] in log), barrier-steer away from instability.
        return -x[0] + 1e3 * max(0.0, -stab_margin(x))

    def con_stab(x):
        return stab_margin(x) - 1e-6  # all closed-loop poles strictly in LHP

    def con_ms(x):
        kp, ki, kd, *_ = unpack(x)
        ms, _ = loop_metrics(jw, Pw, kp, ki, kd, wn_des, wn_cap)
        return args.ms - ms

    def con_mt(x):
        kp, ki, kd, *_ = unpack(x)
        _, mt = loop_metrics(jw, Pw, kp, ki, kd, wn_des, wn_cap)
        return args.mt - mt

    def con_kd(x):
        kp, ki, kd, *_ = unpack(x)
        return kd_ceiling(pll_hz, args.kd_cap, args.kd_cap_bw, args.noise_exp) - kd

    def con_wc(x):
        # OPTIONAL crossover ceiling: |L| < 1 above wc_max (only if --wc-max set)
        kp, ki, kd, *_ = unpack(x)
        Lw = open_resp(jw, kp, ki, kd, wn_des, wn_cap) * Pw
        return 1.0 - np.abs(Lw[w > wc_max]).max()

    cons = [{"type": "ineq", "fun": con_stab},
            {"type": "ineq", "fun": con_ms},
            {"type": "ineq", "fun": con_kd}]
    if args.mt is not None:
        cons.append({"type": "ineq", "fun": con_mt})
    if wc_max is not None:
        cons.append({"type": "ineq", "fun": con_wc})

    # bounds: w_n capped at the PLL natural frequency -- you cannot close the
    # loop faster than the estimator tracks (and beyond it the de-embedded model
    # has no roll-off, so the only real limiter is the kd noise cap). zeta, m too.
    wn_ceiling = min(0.5 * w.max(), wn_des)
    zmin = args.zeta_min
    bounds = [(np.log(2 * np.pi * 0.05), np.log(wn_ceiling)),
              (zmin, max(zmin + 0.1, 2.0)), (1.0, 12.0)]

    rng = np.random.default_rng(0)
    seeds = [np.array([np.log(2 * np.pi * f), z, m])
             for f in (0.3, 1.0, 3.0, 8.0) for z in (max(zmin, 1.0), zmin + 0.5)
             for m in (2.0, 5.0)]
    for _ in range(args.restarts):
        seeds.append(np.array([rng.uniform(*bounds[0]),
                               rng.uniform(*bounds[1]), rng.uniform(*bounds[2])]))

    best = None
    for s0 in seeds:
        r = minimize(neg_bw, s0, method="SLSQP", bounds=bounds,
                     constraints=cons, options={"maxiter": 400, "ftol": 1e-9})
        if not r.success:
            continue
        if (con_stab(r.x) < 0 or con_ms(r.x) < -1e-3 or con_kd(r.x) < -1e-4):
            continue
        if args.mt is not None and con_mt(r.x) < -1e-3:
            continue
        if wc_max is not None and con_wc(r.x) < -1e-3:
            continue
        score = -unpack(r.x)[4]  # maximise w_n
        if best is None or score < best[0]:
            best = (score, r)
    return (best[1] if best else None), unpack


# --------------------------------------------------------------------------- #
# trajectory check: peak position error tracking a trapezoidal-velocity ramp
# to Omega over Ta, feedback-only vs with inertia (accel) feedforward.
# --------------------------------------------------------------------------- #
def trajectory_error(L, P, jbar, omega_rev, ta, dt=1e-3):
    t = np.arange(0, ta * 2.5, dt)
    accel = omega_rev / ta
    a_ref = np.where(t < ta, accel, 0.0)          # accel command [rev/s^2]
    v_ref = np.clip(accel * t, 0, omega_rev)      # [rev/s]
    r = np.cumsum(v_ref) * dt                      # position ref [rev]

    T = ct.feedback(L, 1)
    S = ct.feedback(1, L)
    _, y_fb = ct.forced_response(T, t, r)
    e_fb = r - y_fb
    # 2-DOF: add inertia feedforward torque ff = jbar*a_ref through P*S (low-freq
    # trajectory << w_n, so plant_tf ~ P_true here -- the PLL detail is moot).
    ff = jbar * a_ref
    _, y_ff_extra = ct.forced_response(P * S, t, ff)
    y_ff = y_fb + y_ff_extra
    e_ff = r - y_ff
    # overshoot = position going PAST the reference (y > r)
    os_fb = float(max(0.0, (y_fb - r).max()))
    os_ff = float(max(0.0, (y_ff - r).max()))
    return float(np.abs(e_fb).max()), float(np.abs(e_ff).max()), os_fb, os_ff


def overshoot_pct(numG, denG, numP, denP, w_n, n=700):
    """Setpoint step overshoot of T (percent, 0 = monotonic).

    Overshoot here is dominated by the closed-loop ZERO that ki/kp create, not
    the pole damping -- so it must be constrained explicitly, not via zeta.
    Horizon scales with the dominant time constant (~ a few / w_n).
    """
    numL = np.polymul(numG, numP)
    char = np.polyadd(np.polymul(denG, denP), numL)
    T = ct.tf(list(numL), list(char))
    yf = numL[-1] / char[-1]  # DC value = true steady state (avoids settling err)
    if abs(yf) < 1e-9:
        return 0.0
    t = np.linspace(0, 25.0 / max(w_n, 1e-3), n)
    _, y = ct.step_response(T, t)
    return float(max(0.0, (y.max() - yf) / yf * 100.0))


# --------------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # plant
    p.add_argument("--meta", help="identify_frf.py meta.json")
    p.add_argument("--jbar", type=float)
    p.add_argument("--wa", type=float)
    p.add_argument("--za", type=float)
    p.add_argument("--w1", type=float)
    p.add_argument("--z1", type=float)
    p.add_argument("--hz", action="store_true", help="--wa/--w1 in Hz")
    # robustness targets
    p.add_argument("--ms", type=float, default=1.6, help="max sensitivity peak")
    p.add_argument("--mt", type=float, default=None,
                   help="optional max complementary-sensitivity peak")
    p.add_argument("--zeta-min", type=float, default=1.0,
                   help="min damping of the dominant pole pair (default 1.0 = "
                        "critically damped; lower allows more bandwidth/ringing)")
    # kd noise ceiling model
    p.add_argument("--kd-cap", type=float, required=True,
                   help="experimental kd ceiling at --kd-cap-bw")
    p.add_argument("--kd-cap-bw", type=float, default=200.0,
                   help="PLL bandwidth [Hz] at which --kd-cap was measured")
    p.add_argument("--noise-exp", type=float, default=0.5,
                   help="kd_cap ~ (cap_bw/wp)^exp (0.5 = sqrt-bandwidth)")
    # PLL (moteus pll_filter_hz = 3 dB bandwidth; w_n = 2*pi*f/2.48, zeta=1)
    p.add_argument("--pll", type=float, default=200.0,
                   help="fixed PLL pll_filter_hz [Hz]")
    p.add_argument("--capture-pll-bw", type=float, default=200.0,
                   help="pll_filter_hz the FRF was MEASURED through (sweep records "
                        "the velocity estimate -> plant carries F_vel,capture). "
                        "Set large (e.g. 1e6) if you de-embedded it before fitting.")
    p.add_argument("--wc-max", type=float, default=None,
                   help="OPTIONAL crossover ceiling [Hz] to force a sub-resonance "
                        "loop (default: none -- bandwidth is limited by Ms/Mt)")
    # trajectory spec
    p.add_argument("--omega-rev", type=float, default=33.75,
                   help="target velocity [rev/s]")
    p.add_argument("--ta", type=float, default=2.0, help="accel time [s]")
    # grid / output
    p.add_argument("--f-min", type=float, default=0.05)
    p.add_argument("--f-max", type=float, default=500.0)
    p.add_argument("--restarts", type=int, default=24)
    p.add_argument("--out", help="figure path (default beside --meta)")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    jbar, wa, za, w1, z1 = load_params(args, p)
    P = plant_tf(jbar, wa, za, w1, z1)

    w = 2 * np.pi * np.logspace(np.log10(args.f_min), np.log10(args.f_max), 3000)
    jw = 1j * w
    Pw = P(jw)
    numP, denP = P.num[0][0], P.den[0][0]
    wc_max = (args.wc_max * 2 * np.pi) if args.wc_max else None
    wn_cap = wn_of(args.capture_pll_bw)
    print(f"objective: MAX bandwidth (pole placement) s.t. Ms<={args.ms}"
          + (f", Mt<={args.mt}" if args.mt else "")
          + (f", crossover<={args.wc_max} Hz" if args.wc_max else "")
          + f"; FRF captured through pll_filter_hz={args.capture_pll_bw:.0f} "
          f"(w_n={wn_cap/2/np.pi:.1f} Hz)")

    best, unpack = solve(jw, Pw, args, args.pll, numP, denP, wc_max, wn_cap, jbar)
    if best is None:
        print(f"INFEASIBLE: no PID meets Ms<={args.ms} with kd<=cap. "
              f"Relax --ms/--mt or raise --kd-cap.")
        return
    kp, ki, kd, pll_hz, w_n, zeta, m = unpack(best.x)
    wn_des = wn_of(pll_hz)

    # final, exact metrics with python-control. L = G * plant_tf where G carries
    # both PLL paths (F_pos on kp/ki, F_vel on kd) with F_vel,capture de-embedded.
    numG, denG = open_poly(kp, ki, kd, wn_des, wn_cap)
    G = ct.tf(list(numG), list(denG))
    L = G * P
    S = ct.feedback(1, L)
    Tcl = ct.feedback(L, 1)
    Lw, Sw, Tw = L(jw), S(jw), Tcl(jw)
    ms, mt = np.abs(Sw).max(), np.abs(Tw).max()
    gm, pm, _wcg, wcp = ct.margin(L)
    bw = ct.bandwidth(Tcl) / (2 * np.pi)
    kd_cap_here = kd_ceiling(pll_hz, args.kd_cap, args.kd_cap_bw, args.noise_exp)

    e_fb, e_ff, os_fb, os_ff = trajectory_error(L, P, jbar, args.omega_rev, args.ta)
    step_os = overshoot_pct(numG, denG, numP, denP, w_n)

    print("\n=== solved gains (moteus-native: rev, Nm) ===")
    print(f"  kp = {kp:.5g}")
    print(f"  ki = {ki:.5g}")
    print(f"  kd = {kd:.5g}   (ceiling here = {kd_cap_here:.5g}, "
          f"{'AT CAP' if kd > 0.98 * kd_cap_here else 'below cap'})")
    print(f"  pole placement: w_n={w_n/2/np.pi:.3f} Hz, zeta={zeta:.2f}, "
          f"real-pole mult m={m:.2f}")
    print(f"  pll_filter_hz = {pll_hz:.1f} Hz (PLL w_n={wn_des/2/np.pi:.1f} Hz, fixed)")
    print("\n=== robustness / loop ===")
    print(f"  Ms = {ms:.3f}  (target {args.ms})"
          + (f"   Mt = {mt:.3f} (target {args.mt})" if args.mt else f"   Mt = {mt:.3f}"))
    if ms > 1.0:
        print(f"  implied bounds from Ms:  GM >= {ms/(ms-1):.2f}x "
              f"({ct.mag2db(ms/(ms-1)):.1f} dB),  "
              f"PM >= {np.degrees(2*np.arcsin(1/(2*ms))):.0f} deg")
    else:
        print("  Ms < 1: |S| < 1 at all frequencies -- no sensitivity "
              "amplification anywhere (very robust)")
    print(f"  actual:  GM = {ct.mag2db(gm):.1f} dB,  PM = {pm:.1f} deg "
          f"@ {wcp/2/np.pi:.3f} Hz")
    print(f"  closed-loop -3dB bandwidth = {bw:.2f} Hz")
    print(f"  antiresonance {wa/2/np.pi:.2f} Hz, resonance {w1/2/np.pi:.2f} Hz")
    print("\n=== trajectory check (ramp to "
          f"{args.omega_rev} rev/s in {args.ta} s) ===")
    print(f"  peak position error, feedback only      = {e_fb:.4f} rev "
          f"({e_fb*360:.2f} deg)")
    print(f"  peak position error, + inertia feedfwd  = {e_ff:.4f} rev "
          f"({e_ff*360:.2f} deg)")
    print("\n=== overshoot (should be ~0) ===")
    print(f"  setpoint-step overshoot of T            = {step_os:.2f} %")
    print(f"  ramp overshoot, + inertia feedfwd       = {os_ff*360:.3f} deg "
          f"({'OK' if os_ff*360 < 0.5 else 'CHECK'})")
    print(f"  ramp overshoot, feedback only           = {os_fb*360:.3f} deg")

    # ---- plot S, T, L ----
    import matplotlib.pyplot as plt
    f = w / (2 * np.pi)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 7.5), sharex=True)
    a1.semilogx(f, ct.mag2db(np.abs(Lw)), lw=1.4, label="open loop $L$")
    a1.semilogx(f, ct.mag2db(np.abs(Sw)), lw=1.4, label="sensitivity $S$")
    a1.semilogx(f, ct.mag2db(np.abs(Tw)), lw=1.4, label="compl. $T$")
    a1.axhline(ct.mag2db(args.ms), color="r", ls="--", lw=0.8,
               label=f"Ms target {args.ms}")
    a1.axhline(0, color="k", lw=0.6)
    a2.semilogx(f, np.degrees(np.unwrap(np.angle(Lw))), lw=1.4)
    a2.axhline(-180, color="r", ls="--", lw=0.8)
    for ax in (a1, a2):
        ax.axvline(wa / 2 / np.pi, color="orange", ls="--", lw=1)
        ax.axvline(w1 / 2 / np.pi, color="green", ls="--", lw=1)
        ax.grid(True, which="both")
    a1.set_ylabel("magnitude [dB]"); a1.legend(fontsize=8); a1.set_ylim(-60, 30)
    a2.set_ylabel("phase [deg]"); a2.set_xlabel("frequency [Hz]")
    a1.set_title(f"Ms {ms:.2f}  Mt {mt:.2f}  PM {pm:.0f}°  GM {ct.mag2db(gm):.0f}dB  "
                 f"BW {bw:.1f}Hz  | kp {kp:.4g} ki {ki:.4g} kd {kd:.4g} @ PLL {pll_hz:.0f}Hz",
                 fontsize=8)
    plt.tight_layout()
    out_dir = Path(args.meta).parent if args.meta else Path.cwd()
    out = Path(args.out) if args.out else out_dir / "tune_pid_ms.svg"
    plt.savefig(out)
    print(f"\nPlot saved -> {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
