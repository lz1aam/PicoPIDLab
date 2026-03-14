# Démarrage rapide

Version cible: **v1.2.8**

Workflows:
- Thonny (terminal firmware direct)
- runner (`runner/lab.py`) pour recettes et journalisation

## Pré-requis

```bash
pip install pyserial pyyaml matplotlib numpy
```

Copier `firmware/*` sur la carte.

## Thonny

- lancer `firmware/main.py`
- attendre `# READY:`
- utiliser `check`, `status`, `params`, `control|tune|model|monitor`, `stop`

## Runner

```bash
python3 runner/lab.py
```

Commandes hôte: `h`, `c`, `e <id>`, `s`, `k`, `u`, `x`, `b`, `q`.

Sorties run:
- `runner/runs/<timestamp>__<EXPERIMENT>__<shortname>/`
