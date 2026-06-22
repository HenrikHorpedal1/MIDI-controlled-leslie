#!/usr/bin/env python3
"""Shared artifact logger for the tuning/identification scripts.

Every measurement run is saved into a self-documenting folder under
`software/analysis/data/<step>/<motor>_<timestamp>[_<label>]/` containing:

    data.csv     raw measurement rows (optional)
    plot.png     the figure shown in the thesis (optional)
    meta.json    timestamp, target, all CLI args, fitted results, git commit

This keeps the data for the implementation chapter labelled and reproducible as
the bring-up proceeds, instead of scattering timestamped files in the cwd.

Usage:
    from _artifacts import Run
    run = Run("friction", target=args.target, label=args.label)
    run.save_csv(rows, "friction_map.csv")
    run.save_fig(plt, "friction_map.png")
    run.set_meta(T_C=T_C, b=b, fit="coulomb+viscous+stribeck")
    run.finish()
"""

import csv
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

# moteus CAN id -> human name used in folder/file labels.
MOTOR_NAMES = {1: "drum", 2: "horn"}


def motor_name(target):
    return MOTOR_NAMES.get(target, f"id{target}")


def _data_root():
    # this file lives at software/moteus-config/scripts/_artifacts.py;
    # data goes to software/analysis/data/.
    return Path(__file__).resolve().parents[2] / "analysis" / "data"


class Run:
    """One measurement run -> one timestamped folder under analysis/data/<step>/."""

    def __init__(self, step, target=None, label=None):
        self.step = step
        self.target = target
        self.motor = motor_name(target) if target is not None else "calc"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        slug = f"{self.motor}_{ts}" + (f"_{label}" if label else "")
        self.dir = _data_root() / step / slug
        self.dir.mkdir(parents=True, exist_ok=True)
        self._meta = {
            "step": step,
            "target": target,
            "motor": self.motor,
            "label": label,
            "timestamp_iso": datetime.now().isoformat(timespec="seconds"),
            "script": Path(sys.argv[0]).name,
            "argv": sys.argv[1:],
        }

    def path(self, filename):
        return str(self.dir / filename)

    def save_csv(self, rows, filename="data.csv"):
        if not rows:
            return None
        p = self.dir / filename
        with open(p, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        self._meta.setdefault("files", []).append(filename)
        return str(p)

    def save_fig(self, plt_or_fig, filename="plot.png", dpi=150):
        p = self.dir / filename
        plt_or_fig.savefig(str(p), dpi=dpi)
        self._meta.setdefault("files", []).append(filename)
        return str(p)

    def set_meta(self, **kwargs):
        self._meta.update(kwargs)

    def _git_commit(self):
        try:
            return subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=str(self.dir), stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            return None

    def finish(self, filename="meta.json"):
        commit = self._git_commit()
        if commit:
            self._meta.setdefault("git_commit", commit)
        with open(self.dir / filename, "w") as f:
            json.dump(self._meta, f, indent=2)
        print(f"\nArtifacts saved to: {self.dir}")
        return str(self.dir)
