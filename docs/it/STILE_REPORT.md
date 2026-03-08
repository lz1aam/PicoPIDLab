# Stile di reporting (IT)

Ambito:
- `firmware/*.py`
- `runner/lab.py`

## Classi di messaggio

- `# PHASE:` inizio fase
- `# INFO:` informazione neutra
- `# WARNING:` condizione non fatale
- `# ERROR:` condizione fatale, riscaldatore OFF
- `# RESULT:` risultato calcolato
- `# FINISH:` marker di sincronizzazione host
- `# READY:` sistema in attesa

## Separatore di blocco

Usare solo:
- `# ========================================`

## Telemetria

```text
PV:xx.x SP:xx.x OP:xx.x
```

MPC:

```text
PV:xx.x SP:xx.x OP:xx.x YH:xx.x YP:xx.x
```
