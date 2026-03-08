# Посібник з profile.py

`firmware/profile.py` містить лише параметри, які змінює студент.

## Конвенції

- `PV` — виміряна температура [°C]
- `SP` — задане значення [°C]
- `OP` — вихід регулятора [%], обмежений 0..100

## Рекомендований хід лабораторної

1. Встановити `CONTROL_MODE`.
2. Встановити `SETPOINT_C` і за потреби `EXPERIMENT_RUN_S`.
3. Запустити експеримент.
4. Змінити один параметр.
5. Повторити запуск і порівняти PV/SP/OP.

## PID комбінації

- `PID_VARIANT="PID"` -> `PID_AW_TYPE = NONE/CLAMP/BACKCALC`
- `PID_VARIANT="2DOF"` або `"FF_PID"` -> потрібен `PID_ALGORITHM="PARALLEL"`
- `PID_VARIANT="SMITH_PI"` -> потрібні валідні `MODEL_*` і активний PI (`Kp!=0`, `Ki>0`)

## Пов'язані документи

- `docs/uk/ЗАЛЕЖНОСТІ_НАЛАШТУВАНЬ.md`
- `docs/uk/ШВИДКИЙ_СТАРТ.md`
- `runner/lab.yaml`
