# Hardware Dry-Run Checklist (FOPDT + TUNING)

Use this on real hardware after flashing current firmware files.

## Preconditions

1. Upload all files from `firmware/` to the board.
2. Open serial terminal (Thonny shell or runner).
3. Wait for `# READY:` prompt.
4. Keep heater physically supervised.

## Global safety preflight

1. Run `check`.
2. Confirm output contains `# RESULT: check passed (profile is valid)`.
3. Run:
   - `TEMP_CUTOFF_C 80`
   - `SAFETY_HOLD_S 5`
   - `SAFETY_HYST_C 2`
4. Run `check` again.

## FOPDT path dry-run

1. Configure:
   - `CONTROL_MODE PID`
   - `TELEMETRY_MODE NORMAL`
   - `TS_S 0.25`
   - `FOPDT_U1_PERCENT 50`
   - `STEADY_WINDOW_S 30`
   - `STEADY_BAND_C 0.1`
2. Run `check`.
3. Run `model`.

## TUNING path dry-run (relay)

1. Configure:
   - `CONTROL_MODE PID`
   - `TUNING_RULE ZN_2_PID`
   - `PID_VARIANT PID`
   - `PID_AW_TYPE CLAMP`
   - `PID_ALGORITHM PARALLEL`
   - `TUNING_TARGET_C 50`
   - `TUNING_BAND_C 1`
   - `TUNING_CYCLES 5`
   - `EXPERIMENT_RUN_S 480`
2. Run `check`.
3. Run `tune`.

Expected phase sequence:
1. `# PHASE: TUNING run starting`
2. `# PHASE: TUNING soak`
3. `# PHASE: TUNING relay`
4. `# RESULT: TUNING completed`
5. `# RESULT: Applied set: ...`
6. `# RESULT: TUNING: runtime gains updated -> Kp=... Ki=... Kd=...`
7. `# FINISH: done`
8. `# READY: stopped`

## Abort behavior check

1. During FOPDT run, send `stop`.
2. During TUNING run, send `stop`.

Pass criteria:
- FOPDT prints stop info and exits safely.
- TUNING prints stop info and exits safely.
- Heater is forced OFF and prompt returns to `# READY:`.
