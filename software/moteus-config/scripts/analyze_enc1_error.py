#!/usr/bin/env python3
"""Analyze MA600 position error profile and recommend slip-filter time constant.

Reads the integrated position-error file from compensate_encoder_velocity.py
(--write-integrated), plots the eccentricity profile, runs an FFT to find the
dominant harmonic, and calls compute_slip_filter to get the recommended tau.

    uv run python moteus-config/scripts/analyze_enc1_error.py /tmp/enc1_integrated.txt
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("integrated_file", help="path to --write-integrated output")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    data = np.loadtxt(args.integrated_file)
    angle = data[:, 0]   # [rev]  0..1
    err   = data[:, 1]   # [rev]  position error

    p2p = float(np.max(err) - np.min(err))
    rms = float(np.std(err))
    print(f"Position error  peak-to-peak : {p2p*360*60:.2f} arcmin  ({p2p*1e6:.1f} ppm-rev)")
    print(f"                RMS (1σ)     : {rms*360*60:.2f} arcmin  ({rms*1e6:.1f} ppm-rev)")

    # FFT — find dominant harmonic (should be 1× per rev = 1 cycle/rev)
    N = len(err)
    fft = np.fft.rfft(err - np.mean(err))
    freqs = np.fft.rfftfreq(N, d=1.0/N)   # cycles per revolution
    amps = np.abs(fft) * 2 / N
    dom_idx = int(np.argmax(amps[1:]) + 1)  # skip DC
    dom_freq_cpr = float(freqs[dom_idx])
    dom_amp = float(amps[dom_idx])
    print(f"\nDominant harmonic: {dom_freq_cpr:.1f} cycles/rev  "
          f"amplitude {dom_amp*360*60:.2f} arcmin")
    print(f"  → at chorale  40 RPM output ({40/60:.3f} Hz): "
          f"f_ecc = {dom_freq_cpr * 40/60:.3f} Hz")
    print(f"  → at tremolo 400 RPM output ({400/60:.3f} Hz): "
          f"f_ecc = {dom_freq_cpr * 400/60:.3f} Hz")

    # Recommend filter cutoff: attenuate eccentricity by ≥20 dB at chorale speed
    # |H| = f_c/f_ecc  →  f_c = f_ecc/10  (20 dB attenuation)
    f_ecc_chorale = dom_freq_cpr * 40 / 60
    f_c_rec = f_ecc_chorale / 10.0
    tau_rec = 1.0 / (2 * math.pi * f_c_rec)
    print(f"\nRecommended slip-filter cutoff : f_c = {f_c_rec:.4f} Hz")
    print(f"Recommended time constant      : tau = {tau_rec:.2f} s")
    print(f"  (conf set motor_position.sources.1.pll_filter_hz {f_c_rec:.4f})")

    # ---- plots ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7))

    ax1.plot(angle * 360, err * 360 * 60)
    ax1.set_xlabel("encoder angle  [deg]")
    ax1.set_ylabel("position error  [arcmin]")
    ax1.set_title("MA600 eccentricity profile (post-compensation)")
    ax1.grid(True)

    ax2.bar(freqs[1:N//2], amps[1:N//2] * 360 * 60, width=0.4)
    ax2.axvline(dom_freq_cpr, color="r", ls="--",
                label=f"dominant {dom_freq_cpr:.1f} cyc/rev")
    ax2.set_xlabel("frequency  [cycles/rev]")
    ax2.set_ylabel("amplitude  [arcmin]")
    ax2.set_title("FFT of position error")
    ax2.legend()
    ax2.grid(True)

    fig.tight_layout()
    out = Path(args.integrated_file).with_suffix(".svg")
    fig.savefig(str(out))
    print(f"\nSaved: {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
