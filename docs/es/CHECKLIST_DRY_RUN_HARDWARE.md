# Checklist de dry-run en hardware (FOPDT + TUNING)

Use esta checklist en hardware real después de cargar el firmware.

## Precondiciones

1. Cargar todos los archivos de `firmware/` en la placa.
2. Abrir terminal serie (Thonny o runner).
3. Esperar `# READY:`.
4. Supervisar físicamente el calentador.

## Preflight de seguridad

1. Ejecutar `check`.
2. Confirmar salida: `# RESULT: check passed (config is valid)`.
3. Configurar:
   - `TEMP_CUTOFF_C 80`
4. Ejecutar `check` nuevamente.

## Dry-run de FOPDT

1. Configurar:
   - `CONTROL_MODE PID`
   - `TELEMETRY_MODE NORMAL`
   - `TS_S 0.25`
   - `FOPDT_U1_PERCENT 50`
   - `STEADY_WINDOW_S 30`
   - `STEADY_BAND_C 0.1`
2. `check`
3. `model`

## Dry-run de TUNING (relay)

1. Configurar:
   - `CONTROL_MODE PID`
   - `TUNING_METHOD ZN2_PID`
   - `PID_VARIANT PID`
   - `PID_AW_TYPE CLAMP`
   - `PID_ALGORITHM PARALLEL`
   - `TUNING_TARGET_C 50`
   - `TUNING_BAND_C 1`
   - `TUNING_CYCLES 5`
   - `EXPERIMENT_RUN_S 480`
2. `check`
3. `tune`

## Verificación de abortado

1. Durante FOPDT enviar `stop`.
2. Durante TUNING enviar `stop`.

Criterio de aprobación:
- Fin seguro, calentador OFF y retorno a `# READY:`.
