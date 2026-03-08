# Anleitung zu profile.py

`firmware/profile.py` ist bewusst kompakt: nur Stellgrößen mit kurzen Effect-Kommentaren.
Dieses Dokument enthält die ausführlichen Hinweise.

## Konventionen

- `PV`: gemessene Prozesstemperatur [°C]
- `SP`: Sollwert [°C]
- `OP`: Reglerausgang zum Heizer [%], begrenzt auf 0..100
- Einstellungen in `profile.py` sind UPPERCASE (`KP`, `TI_S`, `SETPOINT_C`)
- Berichte/Logs verwenden Symbole (`Kp`, `Ki`, `Kd`, `Kc`, `Ti`, `Td`, `Pb`)

## Abkürzungen

- `FOPDT`, `CC`, `MPC`, `ZN`, `TL`, `SK`, `2DOF`

## Empfohlener Laborablauf

1. `CONTROL_MODE` setzen.
2. `SETPOINT_C` und optional `EXPERIMENT_RUN_S` setzen.
3. Experiment ausführen.
4. Einen Parameter ändern.
5. Neu ausführen und PV/SP/OP vergleichen.

## PID-Kombinationsübersicht

- `PID_VARIANT="PID"`: `PID_AW_TYPE = NONE/CLAMP/BACKCALC`
- `PID_VARIANT="2DOF"`: benötigt `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="FF_PID"`: benötigt `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="SMITH_PI"`: benötigt aktive `MODEL_*` und PI-Gains (`Kp!=0`, `Ki>0`)
- Modellregeln (`ZN_1_*`, `CC_*`) -> `tune` modellbasiert
- Relayregeln (`ZN_2_*`, `TL_*`) -> `tune` relaybasiert

## Schnellübungen

- Windup-Demo: `PID_AW_TYPE="NONE"` vs `"CLAMP"`
- Relay-Tuning: `TUNING_RULE="ZN_2_PID"`
- Modell-Tuning: `TUNING_RULE="CC_PID"` oder `ZN_1_PID`
- Algorithmusform: `PID_ALGORITHM` wechseln und `pid report` vergleichen

## Modusspezifische Hinweise

### Störgrößen-Injektion

- Aktiv nur im `control`-Pfad.
- Wichtige Parameter: `DIST_ENABLE`, `DIST_MODE`, `DIST_MAG_PCT`, `DIST_START_S`, `DIST_DURATION_S`.

### Relay-Tuning

- Fester Zweipunktbetrieb (`u_high=100%`, `u_low=0%`).
- `TUNING_BAND_C` ist die halbe Hysteresebreite.

### MPC

Rechenaufwand ist näherungsweise proportional zu:

`MPC_HORIZON_STEPS * MPC_MAX_CANDIDATES`

## Verwandte Dokumente

- `docs/en/SETTINGS_DEPENDENCIES.md`
- `docs/en/QUICKSTART.md`
- `runner/lab.yaml`
