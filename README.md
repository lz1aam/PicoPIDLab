# PicoPID Lab (MicroPython, RP2040)

Current version: **v1.2.10** (2026-03-16)

PicoPID Lab is an open-source educational platform for university control-systems courses.  
Students experiment with ON/OFF, PID, Fuzzy, and MPC algorithms on real hardware (RP2040-based thermal plant), then compare controller performance through telemetry, metrics, and plots.

Primary runtime is **MicroPython on RP2040** (RP2040-Zero class boards).

![PicoPID Lab hardware](hardware/PicoPIDLab-picture.jpg)

## Interfaces

### Thonny firmware workflow

![Thonny interface](docs/examples/thonny.png)

Thonny can be used as the direct firmware-side classroom interface for interactive command entry, parameter changes, and live teaching demos.

### Host lab workflow (`runner/lab.py`)

![runner/lab.py interface](docs/examples/lab.py.png)

`runner/lab.py` provides the experiment-driven host workflow with recipe execution, live plots, run folders, metrics, and promoted report-grade artifacts.

## What This Shows

The platform is built around one simple lab workflow:
- identify the thermal plant
- tune a controller
- run closed-loop experiments
- compare plots and metrics

Core observed variables:
- `PV` measured temperature [°C]
- `SP` setpoint temperature [°C]
- `OP` heater output [%]
- `YH` model estimate [°C] in MPC mode
- `YP` model prediction [°C] in MPC mode

Typical plot meaning:
- `PV` vs `SP` shows how well the controller tracks the target temperature
- `OP` shows how aggressively the heater is driven
- FOPDT identification plots show the open-loop thermal step used to estimate `K`, `tau`, and `theta`
- tuning and standard runs produce metrics and comparison plots for repeatable lab reports

## Example Results

### FOPDT identification

![FOPDT identification plot](docs/examples/fopdt-identification.png)

Open-loop heater step used to estimate the thermal model parameters `K`, `tau`, and `theta`.

### Relay tuning run (Ziegler-Nichols 2 PID)

![PID relay tuning run plot](docs/examples/pid-relay-zn2-run.png)

Relay cycling around the target temperature to extract oscillation metrics and tuned gains.
The oscillation experiment follows the Astrom-Hagglund relay-feedback idea for estimating `Ku` and `Pu`, then applies the Ziegler-Nichols 2 PID tuning rule.

### Relay-method Bode margins (Ziegler-Nichols 2 PID)

![ZN2 PID Bode plot](docs/examples/pid-relay-zn2-bode.png)

Oscillation-based tuning can push the loop close to the stability boundary. The Bode plot shows that directly through weak or negative classical margins.

### Model-method Bode margins (Cohen-Coon PID)

![CC PID Bode plot](docs/examples/pid-model-cc-bode.png)

Model-based tuning gives a more conservative loop. Positive gain and phase margins provide a clearer robustness reserve than the relay-tuned case.

### Controller capability examples

#### ON/OFF control

![ON/OFF step tracking plot](docs/examples/onoff-step.png)

Simple bang-bang control is easy to understand and visibly cycles around the target. It is a useful baseline for comparing more advanced regulators.

#### P-only control with disturbance rejection

![P-only step tracking plot](docs/examples/pid-p-step.png)

Pure proportional control reaches the target quickly but leaves a visible offset and weaker disturbance rejection than PI or PID.

#### PI control with disturbance rejection

![PI step tracking plot](docs/examples/pid-pi-step.png)

PI control removes the steady-state offset and gives a cleaner thermal response than P-only control while staying simpler than full PID.

#### Parallel PID control with disturbance rejection

![Parallel PID step tracking plot](docs/examples/pid-parallel-step.png)

Parallel PID reduces oscillation and recovers after a disturbance with smoother heater action than ON/OFF control.

#### Ideal PID control with ramp setpoint

![Ideal PID ramp tracking plot](docs/examples/pid-ideal-ramp.png)

Ideal-form PID also supports ramp commands, which makes it useful for demonstrations where setpoint shaping matters as much as final tracking accuracy.

#### 2DOF PID control

![2DOF PID step tracking plot](docs/examples/pid-2dof-step.png)

Two-degree-of-freedom PID separates setpoint weighting from disturbance rejection, which helps demonstrate how reference shaping changes the transient response.

#### PID with feedforward

![PID feedforward tracking plot](docs/examples/pid-feedforward-tracking.png)

Feedforward PID combines the feedback loop with a compensating term and shows smooth setpoint tracking with reduced control effort.

#### Fuzzy control

![Fuzzy step tracking plot](docs/examples/fuzzy-step.png)

The fuzzy controller reaches the target with a different output profile, showing that the platform is not limited to classical linear control laws.

#### Smith-predictor PI

![Smith PI step tracking plot](docs/examples/smith-pi-step.png)

Smith-predictor PI uses the identified model to compensate dead time and improve response on delayed plants.

#### Gain-scheduled PID

![Gain-scheduled PID step tracking plot](docs/examples/pid-gain-sched-step.png)

Gain scheduling demonstrates that the device can vary controller aggressiveness across the operating region instead of relying on one fixed PID setting.

#### MPC

![MPC tracking plot](docs/examples/mpc-tracking.png)

MPC uses the internal process model to optimize future heater moves under constraints while exposing `YH` and `YP` for prediction-aware teaching demos.

Students primarily edit one file:
- `firmware/config.py`

## Project Layout

- `firmware/` firmware modules on device
- `runner/lab.py` host lab terminal + experiment runner
- `runner/lab.yaml` host experiment/config source of truth

Documentation language folders:
- `docs/en/` (English)
- `docs/bg/` (Bulgarian)
- `docs/de/` (German)
- `docs/es/` (Spanish)
- `docs/uk/` (Ukrainian)
- `docs/it/` (Italian)
- `docs/fr/` (French)

## Quick Start

1. Flash MicroPython to RP2040 (once).
2. Upload files from `firmware/` to board root.
3. Install host dependencies:
   - `pip install pyserial pyyaml matplotlib numpy`
4. Edit experiments in `runner/lab.yaml`.
5. Start host interface:
   - `python3 runner/lab.py`

After connect, the runner enters `lab>` directly.

## Host Terminal Commands (`lab>`)

- `h` help
- `c` catalog
- `e <id|exp_id>` run experiment
- `s` stop active firmware run
- `k` firmware `check`
- `u` host status
- `x` reconnect serial
- `b` terminal home/refresh
- `q` quit

Notes:
- Host uses `LAB:` messages.
- Firmware keeps `# ...` messages/tokens.
- In terminal mode, text telemetry is hidden; live plot and live metrics are always on.
- During active long runs (`control`, `tune`, `model`, `monitor`), one-letter host commands are intercepted locally; `s` maps to firmware `stop`.

## Firmware Terminal Commands (device side)

At firmware prompt (`lab>`), typical commands:
- `status`
- `params`, `params <group>`, `params all`
- `check`
- `pid`
- `<PARAM> <VALUE>` or `<PARAM>=<VALUE>`
- `control`, `tune`, `model`, `monitor`
- during active run: `stop`, `restart`, `help`

## Supported Control Modes

Set in `firmware/config.py`:

- `CONTROL_MODE = "ONOFF"`
- `CONTROL_MODE = "PID"`
- `CONTROL_MODE = "FUZZY"`
- `CONTROL_MODE = "MPC"`

For PID family:
- `PID_VARIANT`: `PID`, `2DOF`, `FF_PID`, `GAIN_SCHED`, `SMITH_PI`
- `PID_ALGORITHM`: `PARALLEL`, `IDEAL`, `SERIES`
- `TUNING_METHOD`:
  - model-based: `ZN1_P`, `ZN1_PI`, `ZN1_PID`, `CC_P`, `CC_PI`, `CC_PID`
  - relay-based: `ZN2_P`, `ZN2_PI`, `ZN2_PID`, `TL_P`, `TL_PI`, `TL_PID`

## Two Workflows

### A) Thonny-only workflow

- Run `firmware/main.py` from Thonny.
- Use firmware commands directly in Thonny shell.
- Best for interactive teaching/demo sessions.

### B) Host lab workflow (`runner/lab.py`)

- Experiment-driven runs from `runner/lab.yaml`
- Automatic run folders with telemetry CSV/log/metrics
- Best for report-grade repeatable experiments

Run artifacts:
- `runner/runs/<timestamp>__<EXPERIMENT>__<shortname>/`

## Experiment Kinds (`runner/lab.yaml`)

- `standard` fixed-duration tracking run
- `fopdt` open-loop model identification
- `tuning` tuning procedure
- `sweep` cartesian parameter sweep

Completion token in firmware:
- `# FINISH: done`

## Telemetry

Firmware modes (`TELEMETRY_MODE`):
- `INFO` status/report lines only
- `NORMAL` `PV/SP/OP`
- `MPC` `PV/SP/OP` + `YH/YP` in MPC mode

Canonical runtime line:
```text
PV:25.3 SP:30.0 OP:45.0
```

MPC extended line:
```text
PV:25.3 SP:30.0 OP:45.0 YH:25.1 YP:26.4
```

## Safety

Configured in `firmware/config.py`:
- `TEMP_CUTOFF_C`

At cutoff:
- heater forced OFF
- active run aborted immediately

## Reporting Style

- `docs/en/REPORTING_STYLE.md`

## Community

- Code of Conduct: `codeofconduct.md`
- Contributing guide: `CONTRIBUTING.md`
