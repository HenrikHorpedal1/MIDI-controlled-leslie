# Measurement data

Captured automatically by the identification/tuning scripts in
`moteus-config/scripts/` (via `_artifacts.py`). Each run creates one self-contained,
timestamped folder so the data for the implementation chapter stays labelled and
reproducible as bring-up proceeds.

## Layout

```
data/<step>/<motor>_<timestamp>[_<label>]/
    data.csv / <name>.csv    raw measurement rows
    <name>.png               the figure for the thesis
    meta.json                target, all CLI args, fitted results, git commit
```

`<motor>` is `drum` (CAN id 1) or `horn` (id 2). Pass `--label <tag>` to any script
to append a note to the folder name (e.g. `--label belt-off`, `--label retest`).

## Steps → scripts → thesis figures

Rows marked *(via `capture_run.py`)* are upstream tools that don't use the logger
themselves; wrap them so their `output.log` and dropped files land in the folder.

| `<step>`          | Script                       | Produces                                  | Implementation-chapter figure |
|-------------------|------------------------------|-------------------------------------------|-------------------------------|
| `current/`        | `moteus_tool --calibrate` *(via `capture_run.py`)*; `compensate_encoder_velocity.py -c 0`; `compensate_cogging_capture.py` | R, L, Kt; encoder/cogging plots + data | current-loop calibration |
| `friction/`       | `identify_friction.py`       | `friction_map.png`, T_C, b, Stribeck      | friction map                  |
| `frf/`            | `identify_frf.py`; `measure_inertia.py` *(via `capture_run.py`)* | `frf_bode.png`, Jbar, w_a, w_1 | measured Bode vs belt model |
| `position_gains/` | `compute_position_gains.py`  | `meta.json` (applied kp/ki/kd, w_n)       | gain table                    |
| `belt_slip/`      | `measure_belt_slip.py`       | `belt_slip.png`, i0, i, slip stats        | belt-slip / linearity         |
| `ma600/`          | `measure_ma600_bct_velocity.py`; `compensate_encoder_velocity.py -c 1` | `encoder_compensation.png`, residual eccentricity | MA600 residual error |
| `verify/`         | `measure_velocity_performance.py` *(via `capture_run.py`)* | low-speed tracking | tracking validation |

## Using a figure in the thesis

Copy the chosen `.png` into the thesis `fig/implementation/` and cite the run's
`meta.json` for the exact parameters. The `meta.json` is the provenance record:
it pins the git commit, the moteus target, and every CLI argument used, so any
plot in the chapter can be traced back to the run that produced it.

Keep these folders under version control — they are the experimental record for
the thesis.
