# Чекліст dry-run на апаратурі (FOPDT + TUNING)

## Передумови

1. Завантажити всі файли з `firmware/` на плату.
2. Відкрити serial-термінал (Thonny або runner).
3. Дочекатися `# READY:`.
4. Забезпечити фізичний нагляд за нагрівачем.

## Перевірка безпеки

1. Виконати `check`.
2. Переконатися, що є `# RESULT: check passed (config is valid)`.
3. Встановити:
   - `TEMP_CUTOFF_C 80`
4. Повторно виконати `check`.

## Dry-run FOPDT

1. Налаштувати: `CONTROL_MODE PID`, `TELEMETRY_MODE NORMAL`, `TS_S 0.25`, `FOPDT_U1_PERCENT 50`, `STEADY_WINDOW_S 30`, `STEADY_BAND_C 0.1`.
2. `check`
3. `model`

## Dry-run TUNING (relay)

1. Налаштувати: `TUNING_METHOD ZN2_PID`, `PID_VARIANT PID`, `PID_AW_TYPE CLAMP`, `PID_ALGORITHM PARALLEL`, `TUNING_TARGET_C 50`, `TUNING_BAND_C 1`, `TUNING_CYCLES 5`, `EXPERIMENT_RUN_S 480`.
2. `check`
3. `tune`

## Перевірка зупинки

Під час `model` і `tune` надіслати `stop`.
Критерій: безпечне завершення, `heater OFF`, повернення до `# READY:`.
