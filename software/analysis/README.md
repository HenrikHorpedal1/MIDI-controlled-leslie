# Analysis

Signal-processing and measurement analysis for the Leslie drive (belt slip, encoder behaviour).

## Contents

- `analyze_belt_slip.py` — belt-slip model fit / plots (reads measurement CSVs, no moteus
  connection needed).

Run scripts from the `software/` uv project, e.g.:

```bash
cd software
uv run python analysis/analyze_belt_slip.py ...
```

Planned (to be migrated from the separate `masteroppgave/` working folder):

- `slip_measurements*.csv` — measurement data
- generated figures
