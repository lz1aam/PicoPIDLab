# Abhängigkeiten der Einstellungen (DE)

Dieses Dokument beschreibt gültige Kombinationen von Parametern in `firmware/profile.py`.

Für Begriffe siehe:
- `docs/de/GLOSSAR.md`

## Kerngedanke

- Laufzeitparameter werden direkt in Firmware angewendet.
- Der aktive Regelpfad wird durch `CONTROL_MODE` und (bei PID) durch `PID_VARIANT` bestimmt.
- `TUNING_RULE` wählt die Tuning-Familie (modellbasiert oder relaybasiert).

## CONTROL_MODE

- `ONOFF` -> Zweipunktregler
- `PID` -> PID-Familie (`PID_VARIANT` aktiv)
- `FUZZY` -> Fuzzy-Regelung
- `MPC` -> MPC-Regelung

## PID-Familie

### PID_VARIANT

- `PID`
- `2DOF`
- `FF_PID`
- `GAIN_SCHED`
- `SMITH_PI`

### PID_ALGORITHM

- `PARALLEL` nutzt primär `KP`, `KI`, `KD`
- `IDEAL`/`SERIES` nutzen primär `KC`, `TI_S`, `TD_S`

### TUNING_RULE

Modellbasiert:
- `ZN_1_P`, `ZN_1_PI`, `ZN_1_PID`
- `CC_P`, `CC_PI`, `CC_PID`

Relaybasiert:
- `ZN_2_P`, `ZN_2_PI`, `ZN_2_PID`
- `TL_P`, `TL_PI`, `TL_PID`

## Wichtige Validierungslogik

- `TS_S > 0`
- `SETPOINT_C` innerhalb zulässigem Bereich
- Ausgangs-/Sicherheitsgrenzen konsistent
- Bei `SMITH_PI`: gültige FOPDT-Modellwerte und aktive PI-Parameter

## Telemetrie-Modi

- `INFO` -> nur `# ...` Meldungen
- `NORMAL` -> `PV/SP/OP`
- `MPC` -> `PV/SP/OP` + `YH/YP` im MPC-Modus

## Rezept- und Profil-Priorität

- In `runner/lab.yaml` gesetzte Rezeptparameter überschreiben Profilwerte.
- Nicht gesetzte Parameter werden aus `firmware/profile.py` übernommen.

## Praxisempfehlung

- Immer nur 1–2 Parameter pro Versuch ändern.
- Nach jeder Konfigurationsänderung `check` ausführen.
