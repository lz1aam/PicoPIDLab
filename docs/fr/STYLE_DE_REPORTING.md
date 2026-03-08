# Style de reporting (FR)

Périmètre:
- `firmware/*.py`
- `runner/lab.py`

## Classes de messages

- `# PHASE:` début d'une phase
- `# INFO:` information neutre
- `# WARNING:` condition non fatale
- `# ERROR:` condition fatale, chauffage OFF
- `# RESULT:` résultat calculé
- `# FINISH:` marqueur de synchronisation hôte
- `# READY:` système en attente

## Séparateur de bloc

Utiliser uniquement:
- `# ========================================`

## Télémétrie

```text
PV:xx.x SP:xx.x OP:xx.x
```

MPC:

```text
PV:xx.x SP:xx.x OP:xx.x YH:xx.x YP:xx.x
```
