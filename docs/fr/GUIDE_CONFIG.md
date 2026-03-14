# Guide de config.py

`firmware/config.py` contient uniquement les paramètres pédagogiques modifiables.

## Flux recommandé

1. Définir `CONTROL_MODE`.
2. Définir `SETPOINT_C` et éventuellement `EXPERIMENT_RUN_S`.
3. Exécuter une expérience.
4. Modifier un seul paramètre.
5. Relancer et comparer PV/SP/OP.

## PID

- `PID_VARIANT="PID"` -> `PID_AW_TYPE = NONE/CLAMP/BACKCALC`
- `2DOF`/`FF_PID` -> nécessite `PID_ALGORITHM="PARALLEL"`
- `SMITH_PI` -> nécessite `MODEL_*` valides et PI actif (`Kp!=0`, `Ki>0`)

## Documents liés

- `docs/fr/DEPENDANCES_PARAMETRES.md`
- `docs/fr/DEMARRAGE_RAPIDE.md`
