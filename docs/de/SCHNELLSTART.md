# Schnellstart

Zielversion: **v1.2.6**

Dieses Dokument deckt beide unterstützten Workflows ab:
- **Thonny-Workflow** (direktes Firmware-Terminal)
- **Lab-Workflow** (`runner/lab.py`, Rezepte, Plots, Metriken, Run-Ordner)

## 1. Voraussetzungen

### Hardware

- RP2040-Board (Pico-Klasse)
- Thermischer PicoPID-Aufbau
- USB-Kabel

### Software

- Python 3.10+
- Thonny
- Python-Pakete:

```bash
pip install pyserial pyyaml matplotlib numpy
```

### Firmware-Dateien

- Alle Dateien aus `firmware/` auf das Board kopieren.

## 2. Einmalige Inbetriebnahme (Thonny)

1. Interpreter: `MicroPython (Raspberry Pi Pico)`
2. Richtigen seriellen Port wählen.
3. `firmware/main.py` starten.
4. Auf `# READY:` warten.

## 3. Workflow A: Thonny

Typische Befehle:

1. `check`
2. `status`
3. `params`, `params pid`, `params model`, `params tuning`
4. `control` / `tune` / `model` / `monitor`
5. während Lauf: `stop`

Parametereingabe:
- `<PARAM> <VALUE>`
- `<PARAM>=<VALUE>`

Beispiele:

```text
SETPOINT_C 45
TUNING_RULE CC_PID
PID_ALGORITHM PARALLEL
```

## 4. Workflow B: Lab-Runner (`runner/lab.py`)

Start (im Repo-Root):

```bash
python3 runner/lab.py
```

### Host-Befehle (`lab>`)

- `h` Hilfe
- `c` Katalog
- `r <id|exp_id>` Rezept starten
- `s` laufenden Versuch stoppen
- `k` Firmware-`check`
- `u` Host-Status
- `x` Serial-Reconnect
- `b` Terminal-Home
- `q` Beenden

Nicht-Host-Eingaben werden an Firmware weitergeleitet.

### Run-Artefakte

- `runner/runs/<timestamp>__<EXPERIMENT>__<shortname>/`

## 5. Rezeptsystem (`runner/lab.yaml`)

- Quelle der Übungsrezepte: `runner/lab.yaml`
- Fallback-Parameter: `firmware/profile.py`

Arten:
- `standard`
- `fopdt`
- `tuning`
- `sweep`

Synchronisationsmarker:
- `# FINISH: done`

## 6. Empfohlener Studierenden-Flow

1. `model`
2. `tune`
3. `control`
4. PV/SP/OP und Metriken vergleichen

## 7. Sicherheit

Vor jedem Versuch:

1. `check`
2. Bestätigung: `# RESULT: check passed (profile is valid)`
3. Sicherheitsparameter prüfen:
   - `TEMP_CUTOFF_C`
   - `SAFETY_HOLD_S`
   - `SAFETY_HYST_C`

## 8. Fehlersuche

### Keine serielle Verbindung

- USB prüfen
- Port/Interpreter in Thonny prüfen
- `x` im `lab>` verwenden

### Lauf startet nicht

- `k` ausführen
- `params` prüfen

### Plot/Metrik-Fenster schließt sofort

- Sicherstellen, dass ein aktiver Lauf gestartet wurde

## 9. Verwandte Dokumente

- `README.md`
- `docs/en/SETTINGS_DEPENDENCIES.md`
- `docs/en/REPORTING_STYLE.md`
- `docs/en/CONTROL_FORMULAS.md`
- `docs/en/PROFILE_TUTORIAL.md`
