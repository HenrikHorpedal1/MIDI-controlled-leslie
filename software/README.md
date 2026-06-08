# Software

Python tooling for the MIDI-controlled Leslie: moteus configuration / data collection and
signal analysis. Managed with [uv](https://docs.astral.sh/uv/).

## Setup

```bash
cd software
uv sync                       # creates .venv from uv.lock
```

Python is pinned to 3.13 (`.python-version`). Dependencies (`moteus`, `moteus-gui`, numpy,
matplotlib, scipy) are declared in `pyproject.toml` and locked in `uv.lock`.

## Running things

Everything runs through `uv run`:

```bash
uv run moteus_tool --help     # calibration / config (from the moteus pip package)
uv run tview                  # live telemetry GUI (from moteus-gui)

# stock moteus utility scripts come from the pinned submodule:
uv run python third_party/moteus/utils/measure_inertia.py --help
uv run python third_party/moteus/utils/compensate_cogging.py --help

# original scripts that have no upstream equivalent:
uv run python moteus-config/scripts/capture_velocity.py
```

## Layout

- `pyproject.toml`, `uv.lock`, `.python-version` — the uv project (one env for everything here).
- `moteus-config/` — moteus configs, calibration logs, and original config/data-collection
  scripts (`scripts/`).
- `analysis/` — belt-slip / encoder analysis scripts and data.
- `third_party/moteus/` — **git submodule**, upstream `mjbots/moteus` pinned to a specific
  commit, used for its stock `utils/` scripts. Run `git submodule update --init` after a fresh
  clone (or clone with `--recurse-submodules`).

The submodule provides the *stock* moteus utilities. Anything under `moteus-config/scripts/` is
original work that does not exist upstream.
