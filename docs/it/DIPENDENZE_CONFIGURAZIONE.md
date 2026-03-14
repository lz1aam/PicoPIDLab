# Dipendenze delle impostazioni (IT)

## CONTROL_MODE

- `ONOFF`, `PID`, `FUZZY`, `MPC`

## PID

### PID_VARIANT

- `PID`, `2DOF`, `FF_PID`, `GAIN_SCHED`, `SMITH_PI`

### PID_ALGORITHM

- `PARALLEL` -> usa `KP`, `KI`, `KD`
- `IDEAL`/`SERIES` -> usa `KC`, `TI_S`, `TD_S`

### TUNING_RULE

Modello:
- `ZN1_*`, `CC_*`

Relay:
- `ZN2_*`, `TL_*`

## TELEMETRY_MODE

- `INFO`, `NORMAL`, `MPC`
