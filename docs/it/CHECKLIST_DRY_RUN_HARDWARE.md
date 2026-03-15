# Checklist dry-run hardware (FOPDT + TUNING)

1. Caricare tutti i file da `firmware/` sulla scheda.
2. Aprire terminale seriale.
3. Attendere `# READY:`.
4. Sorvegliare fisicamente il riscaldatore.

## Preflight sicurezza

- `check`
- verificare `# RESULT: check passed (config is valid)`
- impostare: `TEMP_CUTOFF_C 80`
- `check` di nuovo

## Dry-run FOPDT

- configurare `CONTROL_MODE PID`, `TELEMETRY_MODE NORMAL`, `TS_S 0.25`, `FOPDT_U1_PERCENT 50`, `STEADY_WINDOW_S 30`, `STEADY_BAND_C 0.1`
- `check`
- `model`

## Dry-run TUNING relay

- configurare `TUNING_METHOD ZN2_PID`, `PID_VARIANT PID`, `PID_AW_TYPE CLAMP`, `PID_ALGORITHM PARALLEL`, `TUNING_TARGET_C 50`, `TUNING_BAND_C 1`, `TUNING_CYCLES 5`, `EXPERIMENT_RUN_S 480`
- `check`
- `tune`

## Test arresto

Durante `model` e `tune` inviare `stop`.
