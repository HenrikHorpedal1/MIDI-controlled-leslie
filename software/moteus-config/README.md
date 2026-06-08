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
