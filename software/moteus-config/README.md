# moteus-config

moteus controller configuration, calibration artifacts, and original data-collection scripts
for the horn (CAN id 2) and drum (CAN id 1) motors.

## Contents

- `scripts/` — **original** scripts that have no upstream equivalent. Run from the `software/`
  uv project, e.g. `uv run python moteus-config/scripts/capture_velocity.py`.
  - `capture_velocity.py` — drive a controller at constant velocity and log position / velocity
    / q-current to CSV.
  - `align_encoders.py` — align MA600 zero position and empirically determine the belt pulley
    ratio for the moteus c1.
  - `measure_belt_slip.py` — drive a controller and measure belt slip (logs to CSV, fits a
    model, plots).
  - `identify_friction.py` — map friction torque vs speed (both directions) and fit a
    Coulomb + viscous + Stribeck model. Feeds the bias torque for `identify_frf.py` and the
    achievable-accel bound.
  - `identify_frf.py` — stepped-sine frequency response `T_m -> motor velocity`. Validates the
    two-mass belt model, reads the reflected inertia `Jbar` from the low-frequency asymptote, and
    locates the belt mode (`w_a`, `w_1`). Open-loop torque — read the SAFETY note in the header.
  - `compute_position_gains.py` — pure calculator: rigid-body pole placement turning `Jbar`/`w_1`
    into Moteus `servo.pid_position` gains, with the `i^2` output-frame rescale.
  - `compensate_encoder_velocity.py` — fork of upstream `compensate_encoder.py` (closed-loop
    velocity drive) that **saves the plot, data dumps, and `meta.json`** into `analysis/data/`,
    routed by encoder channel (`-c 0` onboard → `current/`, `-c 1` MA600 → `ma600/`).
  - `compensate_cogging_capture.py` — fork of upstream `compensate_cogging.py` that saves the
    source/averaged plot, the cogging data JSON, and `meta.json` into `analysis/data/current/`.
  - `capture_run.py` — thin wrapper that runs an upstream tool that has no fork (mainly
    `moteus_tool --calibrate`, also `measure_inertia.py`, `measure_velocity_performance.py`)
    inside an `analysis/data/<step>/` folder, teeing its console output to `output.log` and
    capturing any files it drops. The command goes after `--`.

### Data capture

`identify_friction.py`, `identify_frf.py`, `measure_belt_slip.py`, and
`compute_position_gains.py` save every run (CSV + plot + `meta.json`) into a
timestamped folder under `analysis/data/<step>/` via the shared `_artifacts.py`
helper. Pass `--label <tag>` to annotate a run and `--no-show` to save without
opening a plot window. Upstream tools that don't use the logger are captured the
same way through `capture_run.py`. See `analysis/data/README.md` for the layout
and the mapping to the implementation-chapter figures.

### Tuning order

The scripts map onto the tuning methodology (thesis §Control System Design and Tuning Methodology).
Run everything from `software/`; wrap upstream tools with `capture_run.py` so their output is
logged alongside the original scripts.

1. **Current loop + encoder (belt off):**
   ```
   uv run python moteus-config/scripts/capture_run.py --step current -t 1 -- \
       python third_party/moteus/utils/moteus_tool.py -t 1 --calibrate
   uv run python moteus-config/scripts/compensate_encoder_velocity.py -t 1 -c 0
   ```
2. **Plant ID (belt on):** `identify_friction.py`, then `identify_frf.py` → `Jbar`, `w_1`;
   cross-check `Jbar`:
   ```
   uv run python moteus-config/scripts/capture_run.py --step frf -t 1 --label inertia -- \
       python third_party/moteus/utils/measure_inertia.py -t 1
   ```
3. **Position loop:** `compute_position_gains.py` (motor frame) → apply. Then cogging + verify:
   ```
   uv run python moteus-config/scripts/compensate_cogging_capture.py -t 1 --store
   uv run python moteus-config/scripts/capture_run.py --step verify -t 1 -- \
       python third_party/moteus/utils/measure_velocity_performance.py -t 1 --measure-error
   ```
4. **Output frame:** `measure_belt_slip.py` → `i`, `s`; re-run
   `compute_position_gains.py --ratio i` for the output-frame gains.
   The MA600 itself is compensated with `compensate_encoder_velocity.py -t 1 -c 1` (→ `ma600/`).

Planned (to be migrated from the separate `masteroppgave/` working folder):

- `*.cfg` — saved controller configs (horn / drum backups)
- `moteus-cal-*.log` — calibration logs

## Stock moteus utilities

The standard moteus utility scripts (`measure_inertia.py`, `compensate_cogging.py`,
`calibrate_encoder.py`, …) are **not** copied here — use the pinned upstream copies in the
`software/third_party/moteus` submodule:

```bash
cd software
uv run python third_party/moteus/utils/<script>.py ...
```
