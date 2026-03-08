# Profile Tutorial

`firmware/profile.py` is intentionally short: only knobs plus brief effect comments.
This page contains the longer guidance that was removed from the profile file.

## Conventions

- `PV`: measured process temperature [degC]
- `SP`: setpoint/target temperature [degC]
- `OP`: controller output to heater [%], clamped to 0..100
- Settings in `profile.py` are uppercase (`KP`, `TI_S`, `SETPOINT_C`)
- Reports/logs use symbols (`Kp`, `Ki`, `Kd`, `Kc`, `Ti`, `Td`, `Pb`)

## Abbreviations

- `FOPDT`: First-Order Plus Dead Time
- `CC`: Cohen-Coon reaction-curve tuning
- `MPC`: Model Predictive Control
- `ZN`: Ziegler-Nichols
- `TL`: Tyreus-Luyben
- `SK`: Sundaresan-Krishnaswamy
- `2DOF`: Two-Degree-of-Freedom PID

## Suggested Lab Flow

1. Set `CONTROL_MODE`.
2. Set `SETPOINT_C` and optional `EXPERIMENT_RUN_S`.
3. Run one experiment.
4. Change one parameter.
5. Re-run and compare PV/SP/OP trends.

## PID Combination Map

- `PID_VARIANT="PID"`: supports `PID_AW_TYPE = NONE/CLAMP/BACKCALC`
- `PID_VARIANT="2DOF"`: requires `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="FF_PID"`: requires `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="SMITH_PI"`: requires active FOPDT model values (`MODEL_*`) and active PI gains (`Kp!=0`, `Ki>0`)
- `TUNING_RULE` in model family (`ZN_1_*`, `CC_*`): `tune` executes model-based tuning
- `TUNING_RULE` in relay family (`ZN_2_*`, `TL_*`): `tune` executes relay tuning and applies gains

## Quick Exercises

- Windup demo: `PID_VARIANT="PID"`, compare `PID_AW_TYPE="NONE"` vs `"CLAMP"`.
- Relay auto-tuning demo: `TUNING_RULE="ZN_2_PID"`.
- Model-tuning demo: `TUNING_RULE="CC_PID"` (recommended) or `ZN_1_PID`.
- Algorithm-form demo: switch `PID_ALGORITHM` and compare `pid report` output.
- ON/OFF demo: hold `SETPOINT_C`, sweep `ONOFF_HYST_C`, observe switch frequency vs oscillation amplitude.
- FOPDT demo: keep presoak at `OP=0%` (fixed), choose moderate `FOPDT_U1_PERCENT`, run `model` long enough to see settling.

## Notes by Mode

### Disturbance injection

- Disturbance is a control-path OP injection, active only in `control` runs.
- Knobs:
  - `DIST_ENABLE`
  - `DIST_MODE` (`STEP` or `PULSE`)
  - `DIST_MAG_PCT` (signed additive OP percent)
  - `DIST_START_S`
  - `DIST_DURATION_S` (pulse width for `PULSE`)

### Relay tuning

- Relay output is fixed two-position ON/OFF (`u_high=100%`, `u_low=0%`).
- `TUNING_BAND_C` is the relay hysteresis half-band around target.
- More `TUNING_CYCLES` improves confidence but takes longer.

### MPC

Use FOPDT first and save a model before MPC labs. CPU cost is roughly proportional to:

`MPC_HORIZON_STEPS * MPC_MAX_CANDIDATES`

### Indicator bands

If absolute bands are configured, they take precedence over percent-of-SP style logic.

## Related Docs

- `docs/en/SETTINGS_DEPENDENCIES.md`
- `docs/en/QUICKSTART.md`
- `runner/lab.yaml`
