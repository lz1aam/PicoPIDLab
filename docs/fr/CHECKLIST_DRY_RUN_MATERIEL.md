# Checklist dry-run matériel (FOPDT + TUNING)

1. Charger tous les fichiers de `firmware/` sur la carte.
2. Ouvrir un terminal série.
3. Attendre `# READY:`.
4. Surveiller physiquement le chauffage.

## Pré-vol sécurité

- exécuter `check`
- vérifier `# RESULT: check passed (config is valid)`
- régler `TEMP_CUTOFF_C 80`
- exécuter `check` à nouveau

## Dry-run FOPDT

- configurer `CONTROL_MODE PID`, `TELEMETRY_MODE NORMAL`, `TS_S 0.25`, `FOPDT_U1_PERCENT 50`, `STEADY_WINDOW_S 30`, `STEADY_BAND_C 0.1`
- `check`
- `model`

## Dry-run TUNING relay

- configurer `TUNING_METHOD ZN2_PID`, `PID_VARIANT PID`, `PID_AW_TYPE CLAMP`, `PID_ALGORITHM PARALLEL`, `TUNING_TARGET_C 50`, `TUNING_BAND_C 1`, `TUNING_CYCLES 5`, `EXPERIMENT_RUN_S 480`
- `check`
- `tune`

## Test d'arrêt

Pendant `model` et `tune`, envoyer `stop`.
