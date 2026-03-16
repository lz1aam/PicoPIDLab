# Inicio rápido

Versión objetivo: **v1.2.10**

Este documento cubre los dos flujos soportados:
- **Flujo Thonny** (terminal de firmware directo)
- **Flujo Lab** (`runner/lab.py`, recetas, gráficas, métricas y carpetas de ejecución)

## 1. Requisitos

### Hardware

- Placa RP2040 (clase Pico)
- Planta térmica PicoPID
- Cable USB

### Software

- Python 3.10+
- Thonny
- Paquetes Python:

```bash
pip install pyserial pyyaml matplotlib numpy
```

### Archivos de firmware

- Copiar todos los archivos de `firmware/` a la placa.

## 2. Puesta en marcha inicial (Thonny)

1. Intérprete: `MicroPython (Raspberry Pi Pico)`
2. Seleccionar puerto serie correcto.
3. Ejecutar `firmware/main.py`.
4. Esperar `# READY:`.

## 3. Flujo A: Thonny

Comandos típicos:

1. `check`
2. `status`
3. `params`, `params pid`, `params model`, `params tune`
4. `control` / `tune` / `model` / `monitor`
5. durante ejecución: `stop`

Asignación de parámetros:
- `<PARAM> <VALUE>`
- `<PARAM>=<VALUE>`

Ejemplos:

```text
SETPOINT_C 45
TUNING_METHOD CC_PID
PID_ALGORITHM PARALLEL
```

## 4. Flujo B: runner (`runner/lab.py`)

Inicio (desde la raíz del repositorio):

```bash
python3 runner/lab.py
```

### Comandos host (`lab>`)

- `h` ayuda
- `c` catálogo
- `e <id|exp_id>` ejecutar experimento
- `s` detener ejecución activa
- `k` `check` de firmware
- `u` estado host
- `x` reconexión serie
- `b` home de terminal
- `q` salir

Entradas no host se reenvían a firmware.

### Artefactos de ejecución

- `runner/runs/<timestamp>__<EXPERIMENT>__<shortname>/`

## 5. Sistema de recetas (`runner/lab.yaml`)

- Fuente de recetas: `runner/lab.yaml`
- Parámetros de respaldo: `firmware/config.py`

Tipos:
- `standard`
- `fopdt`
- `tuning`
- `sweep`

Token de finalización:
- `# FINISH: done`

## 6. Flujo recomendado para estudiantes

1. `model`
2. `tune`
3. `control`
4. comparar PV/SP/OP y métricas

## 7. Seguridad

Antes de cada ejecución:

1. `check`
2. Confirmar: `# RESULT: check passed (config is valid)`
3. Verificar parámetros de seguridad:
   - `TEMP_CUTOFF_C`

## 8. Solución de problemas

### Sin conexión serie

- Revisar USB
- Revisar puerto/intérprete en Thonny
- usar `x` en `lab>`

### La ejecución no inicia

- ejecutar `k`
- revisar `params`

### La ventana de gráficas/métricas se cierra

- asegurar que existe una ejecución activa

## 9. Documentos relacionados

- `README.md`
- `docs/en/SETTINGS_DEPENDENCIES.md`
- `docs/en/REPORTING_STYLE.md`
- `docs/en/CONTROL_FORMULAS.md`
- `docs/en/CONFIG_TUTORIAL.md`
