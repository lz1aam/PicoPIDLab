# Guía de profile.py

`firmware/profile.py` es intencionalmente compacto: solo parámetros con comentarios Effect breves.
Este documento contiene la guía extendida.

## Convenciones

- `PV`: temperatura medida del proceso [°C]
- `SP`: valor de referencia [°C]
- `OP`: salida del controlador al calentador [%], limitada a 0..100
- Los parámetros de `profile.py` se escriben en mayúsculas (`KP`, `TI_S`, `SETPOINT_C`)
- Reportes/logs usan símbolos (`Kp`, `Ki`, `Kd`, `Kc`, `Ti`, `Td`, `Pb`)

## Abreviaturas

- `FOPDT`, `CC`, `MPC`, `ZN`, `TL`, `SK`, `2DOF`

## Flujo recomendado de laboratorio

1. Definir `CONTROL_MODE`.
2. Definir `SETPOINT_C` y opcionalmente `EXPERIMENT_RUN_S`.
3. Ejecutar un experimento.
4. Cambiar un parámetro.
5. Repetir y comparar tendencias PV/SP/OP.

## Mapa de combinaciones PID

- `PID_VARIANT="PID"`: `PID_AW_TYPE = NONE/CLAMP/BACKCALC`
- `PID_VARIANT="2DOF"`: requiere `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="FF_PID"`: requiere `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="SMITH_PI"`: requiere `MODEL_*` activos y ganancias PI (`Kp!=0`, `Ki>0`)
- Reglas de modelo (`ZN_1_*`, `CC_*`) -> `tune` basado en modelo
- Reglas relay (`ZN_2_*`, `TL_*`) -> `tune` por relay

## Ejercicios rápidos

- Demo de windup: `PID_AW_TYPE="NONE"` vs `"CLAMP"`
- Sintonía relay: `TUNING_RULE="ZN_2_PID"`
- Sintonía por modelo: `TUNING_RULE="CC_PID"` o `ZN_1_PID`
- Forma algorítmica: cambiar `PID_ALGORITHM` y comparar `pid report`

## Notas por modo

### Inyección de perturbación

- Activa solo en la ruta `control`.
- Parámetros: `DIST_ENABLE`, `DIST_MODE`, `DIST_MAG_PCT`, `DIST_START_S`, `DIST_DURATION_S`.

### Sintonía relay

- Salida fija ON/OFF (`u_high=100%`, `u_low=0%`).
- `TUNING_BAND_C` es la semibanda de histéresis.

### MPC

Costo computacional aproximado:

`MPC_HORIZON_STEPS * MPC_MAX_CANDIDATES`

## Documentos relacionados

- `docs/en/SETTINGS_DEPENDENCIES.md`
- `docs/en/QUICKSTART.md`
- `runner/lab.yaml`
