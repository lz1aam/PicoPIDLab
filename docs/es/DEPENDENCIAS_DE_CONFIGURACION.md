# Dependencias de configuración (ES)

Este documento describe combinaciones válidas de parámetros en `firmware/config.py`.

Para terminología:
- `docs/es/GLOSARIO.md`

## Idea principal

- Los parámetros de ejecución se aplican directamente en firmware.
- La ruta de control activa se determina por `CONTROL_MODE` y (en PID) por `PID_VARIANT`.
- `TUNING_RULE` selecciona la familia de sintonía (modelo o relay).

## CONTROL_MODE

- `ONOFF` -> controlador todo/nada
- `PID` -> familia PID (`PID_VARIANT` activo)
- `FUZZY` -> control difuso
- `MPC` -> control MPC

## Familia PID

### PID_VARIANT

- `PID`
- `2DOF`
- `FF_PID`
- `GAIN_SCHED`
- `SMITH_PI`

### PID_ALGORITHM

- `PARALLEL` usa principalmente `KP`, `KI`, `KD`
- `IDEAL`/`SERIES` usan principalmente `KC`, `TI_S`, `TD_S`

### TUNING_RULE

Basadas en modelo:
- `ZN1_P`, `ZN1_PI`, `ZN1_PID`
- `CC_P`, `CC_PI`, `CC_PID`

Basadas en relay:
- `ZN2_P`, `ZN2_PI`, `ZN2_PID`
- `TL_P`, `TL_PI`, `TL_PID`

## Lógica de validación clave

- `TS_S > 0`
- `SETPOINT_C` dentro del rango permitido
- límites de salida/seguridad consistentes
- en `SMITH_PI`: valores válidos del modelo FOPDT y parámetros PI activos

## Modos de telemetría

- `INFO` -> solo mensajes `# ...`
- `NORMAL` -> `PV/SP/OP`
- `MPC` -> `PV/SP/OP` + `YH/YP` en modo MPC

## Prioridad receta/perfil

- Los parámetros de receta en `runner/lab.yaml` sobrescriben el perfil.
- Parámetros no definidos en receta se toman de `firmware/config.py`.

## Recomendación práctica

- Cambiar solo 1–2 parámetros por experimento.
- Ejecutar `check` después de cada cambio de configuración.
