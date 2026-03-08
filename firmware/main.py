# main.py
"""main.py - PicoPID Lab (MicroPython, RP2040)

This file coordinates:
- sensor acquisition (thermistor -> °C)
- controller orchestration
- heater output (% -> PWM)
- WS2812 status LED

Students typically only edit: profile.py
"""

import time
import gc
import math

from sensor import NTCLG100E2103JB
from actuator import Heater
from indicator import StatusRGB
import profile as P
from cli import wait_for_run_command, poll_command_nonblocking
from builder import (
    build_controller,
    pid_descriptor_from_profile,
    pid_selection_from_profile,
    pid_forms_from_gains,
)
from model import (
    run_test as run_model_test,
    set_model_values,
    load_effective_model,
)
from tuning import run_relay_tuning, run_model_tuning, tuning_rule_label

_FIRMWARE_VERSION = "1.2.6"

try:
    from machine import idle as _machine_idle
except Exception:
    _machine_idle = None


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _yield_cpu() -> None:
    if _machine_idle is not None:
        try:
            _machine_idle()
        except Exception:
            pass


def _advance_deadline(next_ms: int, period_ms: int, now_ms: int, max_lag_periods: int = 3) -> int:
    """Advance a periodic deadline without accumulating an unbounded backlog."""
    lag_ms = time.ticks_diff(now_ms, next_ms)
    max_lag_ms = int(max(1, max_lag_periods) * int(period_ms))
    if lag_ms > max_lag_ms:
        return time.ticks_add(now_ms, int(period_ms))
    return time.ticks_add(next_ms, int(period_ms))


def _telemetry_mode() -> str:
    mode = str(P.TELEMETRY_MODE).upper()
    return mode if mode in ("INFO", "NORMAL", "MPC") else "NORMAL"


def _emit_telemetry_line(pv_c: float, sp_c: float, op_pct: float, y_hat=None, y_pred=None) -> None:
    mode = _telemetry_mode()
    if mode == "INFO":
        return
    if (mode == "MPC") and (str(P.CONTROL_MODE).upper() == "MPC") and (y_hat is not None) and (y_pred is not None):
        print("PV:%.1f SP:%.1f OP:%.1f YH:%.1f YP:%.1f" % (float(pv_c), float(sp_c), float(op_pct), float(y_hat), float(y_pred)))
        return
    print("PV:%.1f SP:%.1f OP:%.1f" % (float(pv_c), float(sp_c), float(op_pct)))


def _update_setpoint(setpoint_c: float, sp_current: float, is_ramp: bool, ramp_rate: float, dt_s: float) -> float:
    if not is_ramp:
        return float(setpoint_c)
    max_step = float(ramp_rate) * float(dt_s)
    diff = float(setpoint_c) - float(sp_current)
    if abs(diff) <= max_step:
        return float(setpoint_c)
    return float(sp_current + (max_step if diff > 0.0 else -max_step))


def _emit_telemetry_line_timed(pv_c: float, sp_c: float, op_pct: float, last_emit_ms: int, emit_period_ms: int) -> int:
    if _telemetry_mode() == "INFO":
        return int(last_emit_ms)
    pv_val = float(pv_c)
    if not _isfinite(pv_val):
        return int(last_emit_ms)
    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, last_emit_ms) < int(emit_period_ms):
        return int(last_emit_ms)
    print("PV:%.1f SP:%.1f OP:%.1f" % (pv_val, float(sp_c), float(op_pct)))
    return int(now_ms)


def _compute_dt_s(now_ms: int, last_exec_ms: int, default_dt_s: float, dt_max_s=None) -> float:
    dt_s = time.ticks_diff(now_ms, last_exec_ms) / 1000.0
    if (dt_s <= 0.0) or (not _isfinite(dt_s)):
        return float(default_dt_s)
    if (dt_max_s is not None) and (dt_s > float(dt_max_s)):
        return float(dt_max_s)
    return float(dt_s)


def _disturbance_is_active(elapsed_s: float, enabled: bool, mode: str, start_s: float, duration_s: float) -> bool:
    if not enabled:
        return False
    if float(elapsed_s) < float(start_s):
        return False
    if mode == "STEP":
        return True
    return float(elapsed_s) < (float(start_s) + float(duration_s))


def _yield_until_next(next_ui_ms: int, next_ctrl_ms: int, now_ms: int) -> None:
    dt_ui = time.ticks_diff(next_ui_ms, now_ms)
    dt_ctrl = time.ticks_diff(next_ctrl_ms, now_ms)
    if (dt_ui if dt_ui < dt_ctrl else dt_ctrl) > 0:
        _yield_cpu()


def _handle_control_command(cmd: str):
    if cmd == "stop":
        print("# INFO: stop command received -> control loop stopping")
        return True, False
    if cmd == "restart":
        print("# INFO: restart command received -> control loop reinitializing")
        return True, True
    if cmd in ("help", "?"):
        print("# INFO: run commands: stop, restart, help")
        return False, False
    if cmd in ("control", "tune", "model"):
        print("# INFO: already running (use 'stop' or 'restart')")
        return False, False
    print("# WARNING: unknown command '%s' (type 'help')" % cmd)
    return False, False


def _drain_queued_commands(max_cmds: int = 12) -> None:
    """Drop stale serial commands before presenting READY prompt."""
    dropped = 0
    while dropped < int(max_cmds):
        cmd = poll_command_nonblocking()
        if cmd is None:
            break
        dropped += 1
    if dropped > 0:
        print("# INFO: discarded %d queued command(s) before READY" % int(dropped))


def _make_run_abort_cb(run_label: str):
    """Main-owned command handler for blocking algorithm runs."""
    def _cb():
        cmd = poll_command_nonblocking()
        if cmd is None:
            return False
        if cmd in ("stop", "restart"):
            print("# INFO: stop command received -> %s aborted" % str(run_label))
            try:
                P._RUN_ABORTED = True
            except Exception:
                pass
            return True
        if cmd in ("help", "?"):
            print("# INFO: %s commands: stop, restart, help" % str(run_label))
            return False
        return False

    return _cb


def _make_overtemp_cb(cutoff_c: float):
    """Main-owned overtemperature guard shared by long blocking runs."""
    cutoff = float(cutoff_c)

    def _cb(temp_c: float, phase_label: str):
        _ = phase_label
        return float(temp_c) >= cutoff

    return _cb


def _activate_safety(now_ms: int, safety_hold_ms: int, controller_reset, heater_off) -> int:
    controller_reset()
    heater_off()
    return time.ticks_add(now_ms, int(safety_hold_ms))


def _safety_can_clear(now_ms: int, safety_until_ms: int, pv_valid: bool, temp_raw: float, cool_threshold_c: float) -> bool:
    if not pv_valid:
        return False
    if time.ticks_diff(now_ms, safety_until_ms) < 0:
        return False
    return float(temp_raw) <= float(cool_threshold_c)


def _shutdown_outputs(heater, indicator, done: bool = False) -> None:
    heater.off()
    indicator.off()
    if done:
        print("# FINISH: done")


def _print_info_block(lines) -> None:
    print("# INFO:")
    print("# ========================================")
    for line in lines:
        print("# %s" % str(line))
    print("# ========================================")


def _collect_banner_lines():
    control_mode = str(P.CONTROL_MODE).upper()
    setpoint_type = str(P.SETPOINT_TYPE).upper()
    lines = [
        "PicoPID Lab v%s" % _FIRMWARE_VERSION,
        "control mode: %s" % control_mode,
        "setpoint mode: %s" % setpoint_type,
    ]
    exp_run_s = P.EXPERIMENT_RUN_S
    exp_timer_enabled = (exp_run_s is not None) and (float(exp_run_s) > 0.0)
    run_s_txt = ("%.1f" % float(exp_run_s)) if exp_timer_enabled else "none"
    lines.append(
        "runtime context: telemetry=%s Ts=%.3fs run_s=%s"
        % (
            _telemetry_mode(),
            float(P.TS_S),
            run_s_txt,
        )
    )

    if control_mode == "PID":
        rule = str(P.TUNING_RULE).upper()
        lines.append("tuning rule: %s" % tuning_rule_label(rule))
        try:
            pid_sel = pid_selection_from_profile(P)
            v = str(pid_sel.get("variant", "PID")).upper()
            aw = str(pid_sel.get("aw_type", "CLAMP")).upper()
            if v == "PID":
                lines.append("PID variant: PID (AW=%s)" % aw)
            else:
                lines.append("PID variant: %s" % v)
        except Exception:
            lines.append("PID variant: %s" % str(P.PID_VARIANT).upper())
        lines.append("PID algorithm: %s" % str(P.PID_ALGORITHM).upper())
        try:
            pid_desc = pid_descriptor_from_profile(P)
            algorithm = str(pid_desc.get("algorithm", "PARALLEL")).upper()
            configured = pid_desc.get("configured", {})
            if algorithm == "PARALLEL":
                lines.append("PID configured params: Kp=%.4g Ki=%.4g Kd=%.4g"
                             % (float(configured.get("KP", 0.0)),
                                float(configured.get("KI", 0.0)),
                                float(configured.get("KD", 0.0))))
            else:
                lines.append("PID configured params: Kc=%.4g Ti=%.4gs Td=%.4gs"
                             % (float(configured.get("KC", 0.0)),
                                float(configured.get("TI_S", 0.0)),
                                float(configured.get("TD_S", 0.0))))
        except Exception:
            # Keep banner resilient even if validation catches a bad profile later.
            pass

    if control_mode == "FUZZY":
        lines.append("fuzzy inputs: e, de/dt (Sugeno)")
        lines.append("fuzzy scales: E=%.3f °C  dE=%.3f °C/s"
                     % (P.FUZZY_E_SCALE_C, P.FUZZY_DE_SCALE_C_PER_S))
        lines.append("fuzzy output shaping: du_rate_max=%.2f%%/s  de_alpha=%.2f"
                     % (P.FUZZY_DU_RATE_MAX, P.FUZZY_DE_FILTER_ALPHA))

    if control_mode == "MPC":
        lines.append("MPC config: horizon=%d  grid_step=%.1f%%  du_max=%.1f%%  lambda=%.3f"
                     % (int(P.MPC_HORIZON_STEPS),
                        float(P.MPC_GRID_STEP_PCT),
                        float(P.MPC_DU_MAX_PCT),
                        float(P.MPC_LAMBDA_MOVE)))
        if _telemetry_mode() == "MPC":
            lines.append("MPC telemetry fields: YH=one-step estimate, YP=horizon-end prediction")
    return lines


def _print_session_header():
    control_mode = str(P.CONTROL_MODE).upper()
    setpoint_type = str(P.SETPOINT_TYPE).upper()
    lines = _collect_banner_lines()
    lines.append("setpoint target: %s °C (%s)" % (str(P.SETPOINT_C), setpoint_type))
    if setpoint_type == "RAMP":
        lines.append("ramp rate: %.2f °C/s" % P.SETPOINT_RAMP_RATE)
    tmode = _telemetry_mode()
    if tmode == "INFO":
        lines.append("telemetry fields: status lines only")
    elif (tmode == "MPC") and (control_mode == "MPC"):
        lines.append("telemetry fields: PV[°C], SP[°C], OP[%], YH[°C], YP[°C]")
    else:
        lines.append("telemetry fields: PV[°C], SP[°C], OP[%]")
    _print_info_block(lines)


def _run_monitor_loop(sensor, heater, indicator):
    """Passive telemetry monitor (no control action, heater forced OFF)."""
    ts_s = max(0.05, float(P.TS_S))
    ctrl_period_ms = int(ts_s * 1000)
    ui_period_ms = 50
    gc_period_ms = 3000
    setpoint_c = float(P.SETPOINT_C)
    is_ramp = (str(P.SETPOINT_TYPE).upper() == "RAMP")
    ramp_rate = float(P.SETPOINT_RAMP_RATE)

    sensor_read = sensor.read_c
    heater_off = heater.off
    indicator_update = indicator.update

    now_ms = time.ticks_ms()
    next_ctrl_ms = now_ms
    next_ui_ms = now_ms
    next_gc_ms = now_ms
    last_temp_c = sensor_read()
    if not _isfinite(last_temp_c):
        print("# ERROR: sensor reading invalid at monitor start -> monitor aborted (heater OFF)")
        _shutdown_outputs(heater, indicator)
        return
    sp_current = setpoint_c if not is_ramp else float(last_temp_c)
    last_ctrl_exec_ms = now_ms

    heater_off()
    print("# PHASE: monitor loop starting (heater forced OFF)")
    print("# INFO: monitor commands: stop, help")
    try:
        while True:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, next_gc_ms) >= 0:
                gc.collect()
                next_gc_ms = _advance_deadline(next_gc_ms, gc_period_ms, now_ms)

            if time.ticks_diff(now_ms, next_ui_ms) >= 0:
                cmd = poll_command_nonblocking()
                if cmd is not None:
                    if cmd == "stop":
                        print("# INFO: stop command received -> monitor loop stopping")
                        break
                    if cmd in ("help", "?"):
                        print("# INFO: monitor commands: stop, help")
                indicator_update(last_temp_c, sp_current)
                next_ui_ms = _advance_deadline(next_ui_ms, ui_period_ms, now_ms)

            if time.ticks_diff(now_ms, next_ctrl_ms) >= 0:
                dt_s = _compute_dt_s(now_ms, last_ctrl_exec_ms, ts_s)
                last_ctrl_exec_ms = now_ms
                sp_current = _update_setpoint(setpoint_c, sp_current, is_ramp, ramp_rate, dt_s)

                temp_c = sensor_read()
                if _isfinite(temp_c):
                    last_temp_c = temp_c
                else:
                    temp_c = last_temp_c
                    print("# WARNING: sensor reading invalid in monitor loop (using last valid PV)")
                heater_off()
                _emit_telemetry_line(temp_c, sp_current, 0.0)
                next_ctrl_ms = _advance_deadline(next_ctrl_ms, ctrl_period_ms, now_ms)

            _yield_until_next(next_ui_ms, next_ctrl_ms, now_ms)
    except KeyboardInterrupt:
        print("\n# INFO: keyboard interrupt -> monitor loop stopped")
    _shutdown_outputs(heater, indicator)


def _run_fopdt_experiment(sensor, heater, indicator):
    abort_cb = _make_run_abort_cb("FOPDT run")
    overtemp_cb = _make_overtemp_cb(P.TEMP_CUTOFF_C)
    setattr(P, "_RUN_OVERTEMP_CB", overtemp_cb)
    model = run_model_test(sensor, heater, indicator, P, poll_cmd=abort_cb)
    gc.collect()
    try:
        delattr(P, "_RUN_OVERTEMP_CB")
    except Exception:
        pass
    if model is not None:
        if set_model_values(P, model) is None:
            print("# WARNING: FOPDT model update failed")
    _shutdown_outputs(heater, indicator, done=True)


def _run_pid_relay_tuning(sensor, heater, indicator):
    ts_s = float(P.TS_S)
    emit_period_ms = int(max(100, round(max(0.1, ts_s) * 1000.0)))
    try:
        P._RUN_ABORTED = False
    except Exception:
        pass
    last_emit_ms = time.ticks_add(time.ticks_ms(), -emit_period_ms)

    def _telemetry_cb(pv_c, sp_c, op_pct):
        nonlocal last_emit_ms
        last_emit_ms = _emit_telemetry_line_timed(pv_c, sp_c, op_pct, last_emit_ms, emit_period_ms)

    abort_cb = _make_run_abort_cb("TUNING")
    overtemp_cb = _make_overtemp_cb(P.TEMP_CUTOFF_C)
    setattr(P, "_RUN_OVERTEMP_CB", overtemp_cb)
    print("# PHASE: TUNING run starting")
    KP, KI, KD, _safety = run_relay_tuning(
        sensor,
        heater,
        indicator,
        P,
        float(P.TUNING_TARGET_C),
        poll_cmd=abort_cb,
        telemetry_cb=_telemetry_cb,
    )
    try:
        delattr(P, "_RUN_OVERTEMP_CB")
    except Exception:
        pass
    if hasattr(P, "_RUN_ABORTED") and bool(P._RUN_ABORTED):
        _shutdown_outputs(heater, indicator, done=True)
        return

    P.KP = float(KP)
    P.KI = float(KI)
    P.KD = float(KD)
    ideal_auto = pid_forms_from_gains(KP, KI, KD, float(P.SPAN))["IDEAL"]
    P.KC = float(ideal_auto["KC"])
    P.TI_S = 0.0 if ideal_auto["TI_S"] is None else float(ideal_auto["TI_S"])
    P.TD_S = 0.0 if ideal_auto["TD_S"] is None else float(ideal_auto["TD_S"])

    _shutdown_outputs(heater, indicator, done=True)


def _run_pid_model_tuning(heater, indicator):
    print("# PHASE: TUNING model")
    model = load_effective_model(P)
    if model is None:
        print("# ERROR: TUNING model requires MODEL_* values")
        _shutdown_outputs(heater, indicator, done=True)
        return
    try:
        KP, KI, KD = run_model_tuning(P, model)
    except Exception as ex:
        print("# ERROR: TUNING model failed: %s" % ex)
        _shutdown_outputs(heater, indicator, done=True)
        return

    P.KP = float(KP)
    P.KI = float(KI)
    P.KD = float(KD)
    ideal_auto = pid_forms_from_gains(KP, KI, KD, float(P.SPAN))["IDEAL"]
    P.KC = float(ideal_auto["KC"])
    P.TI_S = 0.0 if ideal_auto["TI_S"] is None else float(ideal_auto["TI_S"])
    P.TD_S = 0.0 if ideal_auto["TD_S"] is None else float(ideal_auto["TD_S"])
    _shutdown_outputs(heater, indicator, done=True)


def _run_control_session(sensor, heater, indicator, controller, tuning_safety: bool, exp_timer_enabled: bool, exp_run_s):
    CONTROL_PERIOD_MS = int(P.TS_S * 1000)
    UI_PERIOD_MS = 50
    GC_PERIOD_MS = 3000
    UI_LOG_GUARD_MS = 40
    TS_S = float(P.TS_S)
    DT_MAX_S = 5.0 * TS_S
    R_TARGET_C = float(P.SETPOINT_C)
    setpoint_type = str(P.SETPOINT_TYPE).upper()
    IS_RAMP = (setpoint_type == "RAMP")
    SETPOINT_RAMP_RATE = float(P.SETPOINT_RAMP_RATE)
    TEMP_CUTOFF_C = float(P.TEMP_CUTOFF_C)
    COOL_THRESHOLD_C = TEMP_CUTOFF_C - float(P.SAFETY_HYST_C)
    SAFETY_HOLD_MS = int(P.SAFETY_HOLD_S * 1000)
    DIST_ENABLE = bool(P.DIST_ENABLE)
    DIST_MODE = str(P.DIST_MODE).upper()
    DIST_MAG_PCT = float(P.DIST_MAG_PCT)
    DIST_START_S = float(P.DIST_START_S)
    DIST_DURATION_S = float(P.DIST_DURATION_S)

    sensor_read = sensor.read_c
    heater_off = heater.off
    heater_set_percent = heater.set_percent
    indicator_update = indicator.update
    controller_update = controller.update
    is_mpc_mode = (str(P.CONTROL_MODE).upper() == "MPC")
    controller_warm_start = getattr(controller, "warm_start", None) if is_mpc_mode else None
    controller_get_estimates = getattr(controller, "get_estimates", None) if is_mpc_mode else None

    now_ms = time.ticks_ms()
    next_ctrl_ms = now_ms
    next_ui_ms = now_ms
    next_gc_ms = now_ms
    mem_free_fn = getattr(gc, "mem_free", None)
    gc_low_mem_bytes = 24 * 1024 if callable(mem_free_fn) else None
    last_emergency_gc_ms = now_ms

    y_last = sensor_read()
    sensor_fault = not _isfinite(y_last)
    if sensor_fault:
        print("# ERROR: sensor reading invalid at control start -> heater forced OFF")
        y_last = 0.0

    r_current = float(P.SETPOINT_C) if setpoint_type == "STEP" else y_last

    in_safety = bool(tuning_safety or sensor_fault)
    safety_until_ms = 0
    last_safety_msg_ms = 0

    if in_safety:
        safety_until_ms = time.ticks_add(time.ticks_ms(), SAFETY_HOLD_MS)
        print("# PHASE: safety hold active at startup")

    heater_off()
    if callable(controller_warm_start):
        try:
            u_now = 0.0
            if hasattr(heater, "current_percent"):
                try:
                    u_now = float(heater.current_percent())
                except Exception:
                    u_now = 0.0
            controller_warm_start(y_last, u_now)
        except Exception:
            pass
    print("# PHASE: control loop starting")

    exp_start_ms = time.ticks_ms()
    restart_requested = False
    ended_by_timer = False

    try:
        last_ctrl_exec_ms = time.ticks_ms()
        u = 0.0
        dist_prev_active = False
        dist_msg = 0  # 0=none, 1=start, 2=end
        safety_msg_due = False
        safety_msg_sensor = False
        safety_msg_temp = 0.0

        while True:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, next_gc_ms) >= 0:
                gc.collect()
                next_gc_ms = _advance_deadline(next_gc_ms, GC_PERIOD_MS, now_ms)
            elif gc_low_mem_bytes is not None:
                try:
                    if (mem_free_fn() < gc_low_mem_bytes) and (time.ticks_diff(now_ms, last_emergency_gc_ms) >= 500):
                        gc.collect()
                        last_emergency_gc_ms = now_ms
                except Exception:
                    gc_low_mem_bytes = None

            if time.ticks_diff(now_ms, next_ui_ms) >= 0:
                cmd = poll_command_nonblocking()
                if cmd is not None:
                    stop_run, restart_now = _handle_control_command(cmd)
                    if stop_run:
                        restart_requested = restart_now
                        break
                indicator_update(y_last, r_current)
                now_ui_ms = time.ticks_ms()
                if time.ticks_diff(next_ctrl_ms, now_ui_ms) >= UI_LOG_GUARD_MS:
                    if dist_msg == 1:
                        print("# INFO: disturbance active (%s, dOP=%+.1f%%)" % (DIST_MODE, DIST_MAG_PCT))
                        dist_msg = 0
                    elif dist_msg == 2:
                        print("# INFO: disturbance ended")
                        dist_msg = 0
                    elif safety_msg_due:
                        if safety_msg_sensor:
                            print("# WARNING: safety hold active (sensor invalid; waiting for recovery)")
                        else:
                            print("# WARNING: safety hold active (temp=%.1f °C; waiting to cool below %.1f °C)"
                                  % (safety_msg_temp, COOL_THRESHOLD_C))
                        safety_msg_due = False
                next_ui_ms = _advance_deadline(next_ui_ms, UI_PERIOD_MS, now_ms)

            if time.ticks_diff(now_ms, next_ctrl_ms) >= 0:
                dt_s = _compute_dt_s(now_ms, last_ctrl_exec_ms, TS_S, DT_MAX_S)
                last_ctrl_exec_ms = now_ms

                if exp_timer_enabled:
                    elapsed_s = time.ticks_diff(now_ms, exp_start_ms) / 1000.0
                    if elapsed_s >= float(exp_run_s):
                        try:
                            controller.reset()
                        except Exception:
                            pass
                        heater_off()
                        print("# RESULT: experiment timer reached %.1f s -> heater OFF" % float(exp_run_s))
                        ended_by_timer = True
                        break

                y_raw = sensor_read()
                pv_valid = _isfinite(y_raw)
                if pv_valid:
                    y = y_raw
                    y_last = y_raw
                else:
                    y = y_last
                    if not in_safety:
                        in_safety = True
                        safety_until_ms = _activate_safety(now_ms, SAFETY_HOLD_MS, controller.reset, heater_off)
                    heater_off()

                r_current = _update_setpoint(R_TARGET_C, r_current, IS_RAMP, SETPOINT_RAMP_RATE, dt_s)

                if (not in_safety) and (y >= TEMP_CUTOFF_C):
                    in_safety = True
                    safety_until_ms = _activate_safety(now_ms, SAFETY_HOLD_MS, controller.reset, heater_off)

                if in_safety and _safety_can_clear(now_ms, safety_until_ms, pv_valid, y_raw, COOL_THRESHOLD_C):
                    in_safety = False

                if in_safety:
                    u = 0.0
                    heater_off()
                    # Latch safety status (max 1 line/s); print later in UI branch.
                    if time.ticks_diff(now_ms, last_safety_msg_ms) >= 1000:
                        last_safety_msg_ms = now_ms
                        safety_msg_due = True
                        safety_msg_sensor = (not pv_valid)
                        safety_msg_temp = float(y_raw)
                else:
                    u_ctrl = controller_update(r_current, y, dt_s)
                    elapsed_s = time.ticks_diff(now_ms, exp_start_ms) / 1000.0
                    dist_active = _disturbance_is_active(
                        elapsed_s, DIST_ENABLE, DIST_MODE, DIST_START_S, DIST_DURATION_S
                    )
                    if dist_active and (not dist_prev_active):
                        dist_msg = 1
                    if (not dist_active) and dist_prev_active and (DIST_MODE == "PULSE"):
                        dist_msg = 2
                    dist_prev_active = dist_active
                    u = u_ctrl + DIST_MAG_PCT if dist_active else u_ctrl
                    if u < 0.0:
                        u = 0.0
                    elif u > 100.0:
                        u = 100.0
                    heater_set_percent(u)

                if callable(controller_get_estimates):
                    try:
                        y_hat, y_pred = controller_get_estimates()
                        _emit_telemetry_line(y, r_current, u, y_hat=y_hat, y_pred=y_pred)
                    except Exception:
                        _emit_telemetry_line(y, r_current, u)
                else:
                    _emit_telemetry_line(y, r_current, u)
                next_ctrl_ms = _advance_deadline(next_ctrl_ms, CONTROL_PERIOD_MS, now_ms)

            _yield_until_next(next_ui_ms, next_ctrl_ms, now_ms)

    except KeyboardInterrupt:
        print("\n# INFO: keyboard interrupt -> control loop stopped")
        restart_requested = False

    _shutdown_outputs(heater, indicator)
    return restart_requested, ended_by_timer


P.validate()

sensor = NTCLG100E2103JB()
heater = Heater()
indicator = StatusRGB(
    band_percent=5.0,
    tight_band_percent=0.5,
    blink_period_ms=800,
)

_print_session_header()
while True:
    _drain_queued_commands()
    gc.collect()
    action = wait_for_run_command(P)
    if action == "exit":
        break
    if action == "status":
        _print_session_header()
        continue
    if action == "monitor":
        _run_monitor_loop(sensor, heater, indicator)
        continue
    if action == "model":
        try:
            P.validate()
        except Exception as ex:
            print("# ERROR: model validation failed: %s" % ex)
            continue
        _run_fopdt_experiment(sensor, heater, indicator)
        continue
    if action == "tune":
        try:
            if str(P.CONTROL_MODE).upper() != "PID":
                raise ValueError("tune command requires CONTROL_MODE='PID'")
            P.validate()
        except Exception as ex:
            print("# ERROR: tune validation failed: %s" % ex)
            continue
        rule = str(P.TUNING_RULE).upper()
        if rule in P.RELAY_TUNING_RULES:
            _run_pid_relay_tuning(sensor, heater, indicator)
        elif rule in P.MODEL_TUNING_RULES:
            _run_pid_model_tuning(heater, indicator)
        else:
            print("# ERROR: tune validation failed: TUNING_RULE must be one of %s"
                  % (P.ALLOWED_TUNING_RULES,))
        continue
    if action != "control":
        print("# ERROR: unknown run action: %s" % str(action))
        continue

    try:
        P.validate()
    except Exception as ex:
        print("# ERROR: control validation failed: %s" % ex)
        continue
    # Re-read timer settings after command-phase parameter edits.
    exp_run_s = P.EXPERIMENT_RUN_S
    exp_timer_enabled = (exp_run_s is not None) and (float(exp_run_s) > 0.0)

    try:
        controller, tuning_safety = build_controller(P, sensor, heater, indicator, poll_cmd=poll_command_nonblocking)
    except Exception as ex:
        _shutdown_outputs(heater, indicator)
        print("# ERROR: controller build failed: %s" % ex)
        continue
    if hasattr(P, "_RUN_ABORTED") and bool(P._RUN_ABORTED):
        _shutdown_outputs(heater, indicator)
        continue
    try:
        controller.reset()
    except Exception:
        pass

    restart_requested, ended_by_timer = _run_control_session(
        sensor, heater, indicator, controller, tuning_safety, exp_timer_enabled, exp_run_s
    )

    if restart_requested:
        print("# INFO: restart complete")
        continue
    if ended_by_timer:
        # Protocol token used by host tools to mark run completion.
        print("# FINISH: done")

print("# INFO: heater OFF")
