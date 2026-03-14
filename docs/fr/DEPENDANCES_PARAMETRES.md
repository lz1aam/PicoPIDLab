# Dépendances des paramètres (FR)

## CONTROL_MODE

- `ONOFF`, `PID`, `FUZZY`, `MPC`

## PID

### PID_VARIANT

- `PID`, `2DOF`, `FF_PID`, `GAIN_SCHED`, `SMITH_PI`

### PID_ALGORITHM

- `PARALLEL` -> utilise `KP`, `KI`, `KD`
- `IDEAL`/`SERIES` -> utilise `KC`, `TI_S`, `TD_S`

### TUNING_RULE

Basé modèle:
- `ZN1_*`, `CC_*`

Basé relay:
- `ZN2_*`, `TL_*`

## TELEMETRY_MODE

- `INFO`, `NORMAL`, `MPC`
