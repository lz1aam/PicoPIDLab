# Settings Dependency Guide

This guide explains which settings depend on each other and when each group is active.

For abbreviation definitions (CC, FOPDT, TL, ZN, SK, etc.), see:
- `docs/en/GLOSSARY.md`

Notation convention used in this guide:
- Setting identifiers are shown exactly as used in firmware (`KP`, `TI_S`, `PID_ALGORITHM`, ...).
- Control-theory symbols in explanations use academic notation (`Kp`, `Ki`, `Kd`, `Kc`, `Ti`, `Td`, `Pb`).

## 1) Always-active settings

These apply in all modes:

- `CONTROL_MODE`
- `SETPOINT_TYPE`, `SETPOINT_C`, `SETPOINT_RAMP_RATE`
- `EXPERIMENT_RUN_S`
- `TS_S`
- `TELEMETRY_MODE` (`INFO` | `NORMAL` | `MPC`)
- `DIST_ENABLE`, `DIST_MODE`, `DIST_MAG_PCT`, `DIST_START_S`, `DIST_DURATION_S`
- `TEMP_CUTOFF_C`
- `INDICATOR_SP_NEAR_ZERO_TOL_C`, `INDICATOR_BAND_C`, `INDICATOR_TIGHT_BAND_C`

## 2) Mode switch map

- `CONTROL_MODE="ONOFF"` -> ON/OFF settings are used
- `CONTROL_MODE="PID"` -> PID family settings are used
- `CONTROL_MODE="FUZZY"` -> Fuzzy settings are used
- `CONTROL_MODE="MPC"` -> MPC settings are used

## 3) PID dependency map

### 3.1 Core selectors

- `PID_VARIANT` selects controller family:
  - `PID`, `2DOF`, `FF_PID`, `GAIN_SCHED`, `SMITH_PI`
- For `PID_VARIANT="PID"`, anti-windup policy is selected by:
  - `PID_AW_TYPE`: `NONE`, `CLAMP`, `BACKCALC`
- `PID_ALGORITHM` selects internal block form:
  - `IDEAL`, `PARALLEL`, `SERIES`
- Active gain form is selected automatically by `PID_ALGORITHM`:
  - `PARALLEL` -> active settings: `KP`, `KI`, `KD`
  - `IDEAL` or `SERIES` -> active settings: `KC`, `TI_S`, `TD_S`
  - `PB` is a reported equivalent view for comparison (`pid`)

### 3.2 Shared PID shaping

- `DERIVATIVE_FILTER_ALPHA` is used by PID-family feedback controllers
- `PID_INTEGRAL_LIMIT` caps integral contribution when enabled

### 3.3 Variant-specific settings

- `PID_VARIANT="PID"` and `PID_AW_TYPE="BACKCALC"` -> uses `PID_AW_TRACKING_TIME_S`
- `PID_VARIANT="2DOF"` -> uses `PID_BETA`
  - requires `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="FF_PID"` -> uses:
  - `FF_MODE`
  - `FF_GAIN_PCT_PER_C`, `FF_BIAS_PCT`
  - `FF_AMBIENT_C`
  - requires `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="GAIN_SCHED"` -> uses:
  - `GS_VARIABLE`
  - `GS_TABLE`
- `PID_VARIANT="SMITH_PI"` -> requires:
  - active FOPDT model values (`MODEL_*`)
  - active PI gains (`Kp!=0`, `Ki>0`)

## 4) PID tuning method dependency map

Only active when `CONTROL_MODE="PID"` and `tune` command is executed.

- `TUNING_METHOD` in relay family (`ZN2_*`, `TL_*`):
  - `tune` command runs relay tuning instead of control tracking
  - performs relay test (`Ku`, `Pu`)
  - valid `TUNING_METHOD` values:
    - `ZN2_P`, `ZN2_PI`, `ZN2_PID`
    - `TL_P`, `TL_PI`, `TL_PID`
  - selected gains are written to active runtime values (`KP/KI/KD` and derived `KC/TI_S/TD_S`)
- `TUNING_METHOD` in model family (`ZN1_*`, `CC_*`):
  - `tune` command runs model-based tuning using active FOPDT model values (`MODEL_*`)
  - valid rule namespace:
    - `ZN1_P`, `ZN1_PI`, `ZN1_PID`
    - `CC_P`, `CC_PI`, `CC_PID`

Relay tuning behavior settings:

- `TUNING_TARGET_C`
- relay output is fixed ON/OFF (`u_high=100%`, `u_low=0%`)
- `TUNING_BAND_C` (hysteresis half-band)
- `TUNING_CYCLES`
- global steady detector:
  - `STEADY_WINDOW_S`
  - `STEADY_BAND_C`

## 5) MPC dependency map

- MPC uses active FOPDT model values (`MODEL_*`).
- Active model must contain: `K`, `tau_s`, `theta_s`, `u0_pct`, `y0`.
- Operating point `(u0, y0)` is taken from the active model.

MPC optimizer settings (always used in MPC mode):

- `MPC_HORIZON_STEPS`
- `MPC_GRID_STEP_PCT`
- `MPC_DU_MAX_PCT`
- `MPC_LAMBDA_MOVE`
- `MPC_MAX_CANDIDATES`
- `MPC_OBSERVER_GAIN` (state observer correction gain `L`, range `0..1`)

MPC execution policy:
- horizon and candidate settings are used exactly as configured
- no automatic factory expansion/reduction of horizon or candidate count

## 6) Disturbance injection dependency map

Active only in `control` runs (not in `monitor`, `model`, or `tune` commands):

- `DIST_ENABLE` (`False` disables all disturbance behavior)
- `DIST_MODE` (`STEP` | `PULSE`)
- `DIST_MAG_PCT` (additive output disturbance, signed percent)
- `DIST_START_S` (activation time from control-run start)
- `DIST_DURATION_S` (used only for `PULSE`)

Behavior:
- applied output is `OP_applied = clamp(OP_controller + DIST_MAG_PCT, 0, 100)` while disturbance is active
- `STEP`: active from `DIST_START_S` to end of run
- `PULSE`: active in `[DIST_START_S, DIST_START_S + DIST_DURATION_S)`

## 7) FOPDT dependency map

Active when `model` command is executed:

- step definition:
  - `FOPDT_U1_PERCENT`
- estimator:
  - baseline policy:
  - firmware always runs presoak-to-steady at `OP=0%`
  - steady detector uses `STEADY_WINDOW_S`, `STEADY_BAND_C`

## 8) Runtime commands to inspect/verify

- `params` (active groups)
- `params all` (full catalog)
- `params pid` (PID-only view)
- `check` (validate current runtime config)
- `pid` (teaching report with algorithm/input/mode info)
