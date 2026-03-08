# Hardware-Dry-Run-Checkliste (FOPDT + TUNING)

Diese Checkliste wird auf realer Hardware nach dem Flashen verwendet.

## Voraussetzungen

1. Alle Dateien aus `firmware/` auf das Board laden.
2. Serielle Konsole öffnen (Thonny oder Runner).
3. Auf `# READY:` warten.
4. Heizer stets physisch überwachen.

## Sicherheits-Vorprüfung

1. `check` ausführen.
2. Ausgabe muss enthalten: `# RESULT: check passed (profile is valid)`.
3. Setzen:
   - `TEMP_CUTOFF_C 80`
   - `SAFETY_HOLD_S 5`
   - `SAFETY_HYST_C 2`
4. `check` erneut ausführen.

## FOPDT-Dry-Run

1. Konfigurieren:
   - `CONTROL_MODE PID`
   - `TELEMETRY_MODE NORMAL`
   - `TS_S 0.25`
   - `FOPDT_U1_PERCENT 50`
   - `STEADY_WINDOW_S 30`
   - `STEADY_BAND_C 0.1`
2. `check`
3. `model`

## TUNING-Dry-Run (Relay)

1. Konfigurieren:
   - `CONTROL_MODE PID`
   - `TUNING_RULE ZN_2_PID`
   - `PID_VARIANT PID`
   - `PID_AW_TYPE CLAMP`
   - `PID_ALGORITHM PARALLEL`
   - `TUNING_TARGET_C 50`
   - `TUNING_BAND_C 1`
   - `TUNING_CYCLES 5`
   - `EXPERIMENT_RUN_S 480`
2. `check`
3. `tune`

## Abbruchprüfung

1. Während FOPDT `stop` senden.
2. Während TUNING `stop` senden.

Kriterium:
- Sicheres Ende, Heizer OFF, Rückkehr zu `# READY:`.
