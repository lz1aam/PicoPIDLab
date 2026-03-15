# Контролен списък за dry-run на хардуер (FOPDT + TUNING)

Използвайте този списък на реален хардуер след качване на текущите firmware файлове.

## Предварителни условия

1. Качете всички файлове от `firmware/` на платката.
2. Отворете сериен терминал (Thonny shell или runner).
3. Изчакайте подканата `# READY:`.
4. Осигурете физически контрол и наблюдение на нагревателя.

## Глобална проверка за безопасност

1. Изпълнете `check`.
2. Потвърдете, че изходът съдържа `# RESULT: check passed (config is valid)`.
3. Задайте:
   - `TEMP_CUTOFF_C 80`
4. Изпълнете `check` отново.

## Dry-run на FOPDT път

1. Конфигурирайте:
   - `CONTROL_MODE PID`
   - `TELEMETRY_MODE NORMAL`
   - `TS_S 0.25`
   - `FOPDT_U1_PERCENT 50`
   - `STEADY_WINDOW_S 30`
   - `STEADY_BAND_C 0.1`
2. Изпълнете `check`.
3. Изпълнете `model`.

## Dry-run на TUNING път (relay)

1. Конфигурирайте:
   - `CONTROL_MODE PID`
   - `TUNING_METHOD ZN2_PID`
   - `PID_VARIANT PID`
   - `PID_AW_TYPE CLAMP`
   - `PID_ALGORITHM PARALLEL`
   - `TUNING_TARGET_C 50`
   - `TUNING_BAND_C 1`
   - `TUNING_CYCLES 5`
   - `EXPERIMENT_RUN_S 480`
2. Изпълнете `check`.
3. Изпълнете `tune`.

Очаквана последователност на фазите:
1. `# PHASE: TUNING run starting`
2. `# PHASE: TUNING soak`
3. `# PHASE: TUNING relay`
4. `# RESULT: TUNING completed`
5. `# tuning method: ...`
6. `# RESULT: TUNING: runtime gains updated -> Kp=... Ki=... Kd=...`
7. `# FINISH: done`
8. `# READY: stopped`

## Проверка на прекъсване (abort)

1. По време на FOPDT изпълнение изпратете `stop`.
2. По време на TUNING изпълнение изпратете `stop`.

Критерии за успешно преминаване:
- FOPDT отпечатва съобщение за спиране и приключва безопасно.
- TUNING отпечатва съобщение за спиране и приключва безопасно.
- Нагревателят е принудително изключен и подканата се връща към `# READY:`.
