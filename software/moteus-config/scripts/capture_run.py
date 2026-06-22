#!/usr/bin/env python3
"""Run an external tool and capture its output into analysis/data/<step>/.

Thin wrapper for the upstream moteus utilities (and anything else) that print
results and drop files but do not use the `_artifacts` logger themselves --
`moteus_tool --calibrate`, `compensate_encoder.py`, `compensate_cogging.py`,
`measure_velocity_performance.py`, `measure_inertia.py`, etc.

It creates a fresh artifact folder, runs the wrapped command with its working
directory set to that folder (so any files the tool writes -- calibration logs,
`--output` CSVs, `--write-integrated` dumps -- land there), tees all console
output to `output.log`, and writes `meta.json` with the command, return code,
git commit, and the list of files produced.  Paths in the command that exist
relative to the launch directory are resolved to absolute first, so the wrapped
script is still found after the working-directory change.

Plots opened with `plt.show()` are still shown interactively for inspection;
pass the tool's own data-dump flag (e.g. `--write-integrated`, `--output`) to
keep a re-plottable copy in the folder.

Usage (run from software/), with the wrapped command after `--`:

  # Step 1 -- current-loop calibration (belt off)
  uv run python moteus-config/scripts/capture_run.py --step current -t 1 -- \\
      python third_party/moteus/utils/moteus_tool.py -t 1 --calibrate

  # Step 1 -- onboard encoder linearisation (belt off)
  uv run python moteus-config/scripts/capture_run.py --step current -t 1 --label encoder -- \\
      python third_party/moteus/utils/compensate_encoder.py -t 1 --plot --write-integrated integrated.txt

  # Step 3 -- cogging compensation
  uv run python moteus-config/scripts/capture_run.py --step current -t 1 --label cogging -- \\
      python third_party/moteus/utils/compensate_cogging.py -t 1 --store --plot-results --output cogging.csv

  # Cross-check reflected inertia
  uv run python moteus-config/scripts/capture_run.py --step frf -t 1 --label inertia -- \\
      python third_party/moteus/utils/measure_inertia.py -t 1

  # Verify -- low-speed tracking
  uv run python moteus-config/scripts/capture_run.py --step verify -t 1 -- \\
      python third_party/moteus/utils/measure_velocity_performance.py -t 1 --measure-error
"""

import argparse
import os
import subprocess
import sys

from _artifacts import Run


def main():
    argv = sys.argv[1:]
    if "--" not in argv:
        print("error: separate the wrapped command with '--'.\n"
              "       capture_run.py --step <step> -t <id> -- <command ...>",
              file=sys.stderr)
        return 2
    split = argv.index("--")
    wrapper_argv, command = argv[:split], argv[split + 1:]

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        usage="capture_run.py --step <step> [-t <id>] [--label <tag>] -- <command ...>")
    parser.add_argument("--step", required=True,
                        help="artifact step folder (current, frf, verify, ma600, ...)")
    parser.add_argument("--target", "-t", type=int, default=None,
                        help="moteus CAN id (1=drum, 2=horn) -- for the folder label only")
    parser.add_argument("--label", default=None,
                        help="optional tag appended to the artifact folder name")
    args = parser.parse_args(wrapper_argv)

    if not command:
        print("error: empty command after '--'.", file=sys.stderr)
        return 2

    # Resolve any command tokens that are real paths relative to the launch
    # directory to absolute, so they survive the working-directory change.
    launch_cwd = os.getcwd()
    resolved = []
    for tok in command:
        candidate = os.path.join(launch_cwd, tok)
        resolved.append(os.path.abspath(candidate) if os.path.exists(candidate) else tok)

    run = Run(args.step, target=args.target, label=args.label)
    print(f"Running in {run.dir}:\n  {' '.join(command)}\n" + "-" * 60)

    log_path = run.dir / "output.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(resolved, cwd=str(run.dir),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1)
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(line)
        proc.wait()
    rc = proc.returncode

    produced = sorted(p.name for p in run.dir.iterdir()
                      if p.name not in ("output.log", "meta.json"))
    run.set_meta(command=resolved, command_str=" ".join(command),
                 returncode=rc, produced_files=produced)
    run.finish()
    print("-" * 60 + f"\nExit code {rc}.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
