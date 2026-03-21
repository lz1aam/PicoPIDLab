# Avvio rapido

Versione target: **v1.2.11**

Workflow supportati:
- Thonny (terminale firmware diretto)
- runner (`runner/lab.py`) per ricette e logging

## Requisiti

```bash
pip install pyserial pyyaml matplotlib numpy
```

Caricare `firmware/*` sulla scheda.

## Thonny

- avviare `firmware/main.py`
- attendere `# READY:`
- usare `check`, `status`, `params`, `control|tune|model|monitor`, `stop`

## Runner

```bash
python3 runner/lab.py
```

Comandi host: `h`, `c`, `e <id>`, `s`, `k`, `u`, `x`, `b`, `q`.

Output run:
- `runner/runs/<timestamp>__<EXPERIMENT>__<shortname>/`
