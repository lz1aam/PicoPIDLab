# Quick Start

Version target: **v1.2.8** (2026-03-14)

This guide covers both supported workflows:
- **Thonny workflow** (direct firmware terminal)
- **Lab workflow** (`runner/lab.py`, recipes + plots + metrics + run folders)

Use this document as the operational entry point for students and instructors.

## 1. Prerequisites

### Hardware

- RP2040 board (Pico-class)
- PicoPID thermal plant connected to the board
- USB cable

### PC software

- Python 3.10+
- Thonny (recommended for firmware-side interactive work): https://thonny.org
- Python packages:

```bash
pip install pyserial pyyaml matplotlib numpy
```

### Firmware files

Upload all files from:
- `firmware/`

to the board root filesystem.

---

## 2. One-time Board Setup in Thonny

1. In Thonny, select interpreter: `MicroPython (Raspberry Pi Pico)`.
2. Select the correct serial port.
3. Upload all firmware files from `firmware/`.
4. Run `firmware/main.py`.
5. Wait for firmware `# READY:` prompt.

At this point the board can be used either from Thonny or from `runner/lab.py`.

---

## 3. Workflow A: Thonny (direct firmware terminal)

Use this when you want direct command-line interaction with firmware and no recipe automation.

Typical command sequence:

1. `check`
2. `status`
3. `params` / `params pid` / `params model` / `params tune`
4. run command:
   - `control` (closed-loop control)
   - `tune` (parameter tuning from selected `TUNING_RULE`)
   - `model` (FOPDT identification)
   - `monitor` (passive monitoring, heater OFF)
5. during active run: `stop`

Parameter update syntax:
- `<PARAM> <VALUE>` (primary)
- `<PARAM>=<VALUE>` (also accepted)

Examples:

```text
SETPOINT_C 45
TUNING_RULE CC_PID
PID_ALGORITHM PARALLEL
```

Firmware ownership:
- Firmware status/report tokens always start with `# ...`
- Telemetry lines are non-prefixed:
  - `PV:... SP:... OP:...`
  - MPC mode adds `YH` and `YP`

---

## 4. Workflow B: Lab Runner (`runner/lab.py`)

Use this for repeatable exercises, run folders, CSV logs, live plot, and live metrics.

Start from repository root:

```bash
python3 runner/lab.py
```

(`python3 lab.py` also works if current directory is `runner/`.)

After connect/sync the host enters `lab>` directly.

### Host commands (`lab>`)

- `h` help
- `c` catalog
- `e <id|exp_id>` run experiment
- `s` stop active run
- `k` firmware `check`
- `u` host status
- `x` reconnect serial
- `b` terminal home/refresh
- `q` quit

Input not recognized as host command is forwarded to firmware.

### Active run controls

During active long runs (`control`, `tune`, `model`, `monitor`):
- `s` stops the run
- `h` prints run controls

### Output ownership

- Host lines are prefixed with `LAB:`
- Firmware lines keep `# ...` and telemetry formats

### Run artifacts

Each run is stored under:
- `runner/runs/<timestamp>__<EXPERIMENT>__<shortname>/`

Typical contents:
- CSV telemetry
- log text
- metrics JSON
- plots

---

## 5. Recipe System (`runner/lab.yaml`)

`runner/lab.yaml` is the host-side source of truth for exercises.
`firmware/config.py` is fallback for parameters not overridden by recipe.

Recipe kinds:
- `standard` fixed-duration run
- `fopdt` open-loop model identification
- `tuning` tuning procedure
- `sweep` cartesian parameter sweep

Completion token used by host synchronization:
- `# FINISH: done`

---

## 6. Recommended Student Flow

1. `model` to identify/update FOPDT model (`MODEL_*` in RAM)
2. `tune` with selected `TUNING_RULE` (model-based or relay-based)
3. `control` with updated gains
4. Compare PV/SP/OP trends and saved metrics

Repeat with one parameter change at a time.

---

## 7. Safety and Validation

Before any run:

1. `check`
2. Confirm:
   - `# RESULT: check passed (config is valid)`
3. Verify safety settings in `config.py`:
   - `TEMP_CUTOFF_C`

On cutoff or fatal condition:
- heater is forced OFF
- firmware prints warning/error tokens

---

## 8. Troubleshooting

### No serial connection

- Reconnect USB
- Confirm correct interpreter/port in Thonny
- Run `x` in `lab>` to reconnect session

### Host can connect but run does not start

- Run `k` (firmware check) from `lab>`
- Run `params` and verify required settings for selected mode

### Plot/metrics window does not stay open

- Ensure an active run is started (`control`, `model`, `tune`, `monitor`, or `e <id>`)
- Do not close windows before run starts

### Unexpected command behavior

- Use `h` in `lab>`
- Use firmware `help` command for device-side command list

---

## 9. Related Docs

- `README.md`
- `docs/en/SETTINGS_DEPENDENCIES.md`
- `docs/en/REPORTING_STYLE.md`
- `docs/en/CONTROL_FORMULAS.md`
- `docs/en/CONFIG_TUTORIAL.md`
