# Estilo de reportes (ES)

Alcance:
- `firmware/*.py`
- `runner/lab.py`

## Clases de mensajes en ejecución

- `# PHASE:` inicio de fase claramente nombrada
- `# INFO:` información neutral
- `# WARNING:` condición no fatal
- `# ERROR:` condición fatal, calentador apagado
- `# RESULT:` resultado calculado
- `# FINISH:` marcador de sincronización con host
- `# READY:` sistema en reposo

## Formato de bloque

- Solo se permite este separador:
  - `# ========================================`
- No usar separadores alternativos (`-`, `_`, longitudes mixtas)

## Propiedad de salida

- Los tokens de firmware mantienen formato `# ...`.
- Los mensajes host usan prefijo `LAB:`.
- Al cambiar de origen (host/firmware), usar exactamente una línea en blanco.

## Formato de telemetría

Estándar:

```text
PV:xx.x SP:xx.x OP:xx.x
```

MPC:

```text
PV:xx.x SP:xx.x OP:xx.x YH:xx.x YP:xx.x
```
