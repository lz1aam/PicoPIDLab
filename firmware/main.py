# main.py
"""main.py - PicoPID Lab (MicroPython, RP2040)

This file coordinates:
- sensor acquisition (thermistor -> °C)
- controller orchestration
- heater output (% -> PWM)
- WS2812 status LED

Students typically only edit: config.py
"""

import time
import gc
import math

from hardware import NTCLG100E2103JB, Heater, StatusRGB, yield_cpu, advance_deadline
import config as P
from cli import wait_for_run_command, poll_command_nonblocking

_FIRMWARE_VERSION = "1.2.11"

def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _tuning_method_label(code: str) -> str:
    method = str(code).upper()
    if method.startswith("ZN1_"):
        family = "Ziegler-Nichols 1"
    elif method.startswith("ZN2_"):
        family = "Ziegler-Nichols 2"
    elif method.startswith("CC_"):
        family = "Cohen-Coon"
    elif method.startswith("TL_"):
        family = "Tyreus-Luyben"
    else:
        return str(code)
    mode = method.split("_", 1)[-1]
    return "%s (%s)" % (family, mode)


def _telemetry_mode(profile_mod=P) -> str:
    mode = str(profile_mod.TELEMETRY_MODE).upper()
    return mode if mode in ("INFO", "NORMAL", "MPC") else "NORMAL"


def _snapshot_header_config(profile_mod):
    control_mode = str(profile_mod.CONTROL_MODE).upper()
    setpoint_type = str(profile_mod.SETPOINT_TYPE).upper()
    telemetry_mode = _telemetry_mode(profile_mod)
    exp_run_s = profile_mod.EXPERIMENT_RUN_S
    return {
        "control_mode": control_mode,
        "setpoint_type": setpoint_type,
        "telemetry_mode": telemetry_mode,
        "ts_s": float(profile_mod.TS_S),
        "exp_run_s": exp_run_s,
        "exp_timer_enabled": (exp_run_s is not None) and (float(exp_run_s) > 0.0),
        "setpoint_c_text": str(profile_mod.SETPOINT_C),
        "ramp_rate": float(profile_mod.SETPOINT_RAMP_RATE),
        "pid_method": str(profile_mod.TUNING_METHOD).upper(),
        "pid_variant": str(profile_mod.PID_VARIANT).upper(),
        "pid_aw_type": str(profile_mod.PID_AW_TYPE).upper(),
        "pid_algorithm": str(profile_mod.PID_ALGORITHM).upper(),
        "kp": float(profile_mod.KP),
        "ki": float(profile_mod.KI),
        "kd": float(profile_mod.KD),
        "kc": float(profile_mod.KC),
        "ti_s": float(profile_mod.TI_S),
        "td_s": float(profile_mod.TD_S),
        "fuzzy_e_scale_c": float(profile_mod.FUZZY_E_SCALE_C),
        "fuzzy_de_scale_c_per_s": float(profile_mod.FUZZY_DE_SCALE_C_PER_S),
        "fuzzy_du_rate_max": float(profile_mod.FUZZY_DU_RATE_MAX),
        "fuzzy_de_filter_alpha": float(profile_mod.FUZZY_DE_FILTER_ALPHA),
        "mpc_horizon_steps": int(profile_mod.MPC_HORIZON_STEPS),
        "mpc_grid_step_pct": float(profile_mod.MPC_GRID_STEP_PCT),
        "mpc_du_max_pct": float(profile_mod.MPC_DU_MAX_PCT),
        "mpc_lambda_move": float(profile_mod.MPC_LAMBDA_MOVE),
    }


def _emit_telemetry_line(
    pv_c: float,
    sp_c: float,
    op_pct: float,
    telemetry_mode: str,
    control_mode: str,
    y_hat=None,
    y_pred=None,
) -> None:
    mode = str(telemetry_mode).upper()
    if mode == "INFO":
        return
    if (mode == "MPC") and (str(control_mode).upper() == "MPC") and (y_hat is not None) and (y_pred is not None):
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
        yield_cpu()


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


def _make_run_abort_cb(run_label: str, run_state: dict):
    """Main-owned command handler for blocking algorithm runs."""
    def _cb():
        cmd = poll_command_nonblocking()
        if cmd is None:
            return False
        if cmd in ("stop", "restart"):
            print("# INFO: stop command received -> %s aborted" % str(run_label))
            run_state["aborted"] = True
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


def _shutdown_outputs(heater, indicator, done: bool = False) -> None:
    heater.off()
    indicator.off()
    if done:
        print("# FINISH: done")


def _new_run_state() -> dict:
    return {"aborted": False}


def _print_info_block(lines) -> None:
    print("# INFO:")
    print("# ========================================")
    for line in lines:
        print("# %s" % str(line))
    print("# ========================================")


def _collect_banner_lines(cfg):
    control_mode = cfg["control_mode"]
    telemetry_mode = cfg["telemetry_mode"]
    lines = [
        "PicoPID Lab v%s" % _FIRMWARE_VERSION,
        "control mode: %s" % control_mode,
        "setpoint mode: %s" % cfg["setpoint_type"],
    ]
    exp_run_s = cfg["exp_run_s"]
    exp_timer_enabled = bool(cfg["exp_timer_enabled"])
    run_s_txt = ("%.1f" % float(exp_run_s)) if exp_timer_enabled else "none"
    lines.append(
        "runtime context: telemetry=%s Ts=%.3fs run_s=%s"
        % (
            telemetry_mode,
            float(cfg["ts_s"]),
            run_s_txt,
        )
    )

    if control_mode == "PID":
        lines.append("tuning method: %s" % _tuning_method_label(cfg["pid_method"]))
    lines.extend(_mode_banner_lines_from_snapshot(cfg, telemetry_mode))
    return lines


def _print_session_header():
    cfg = _snapshot_header_config(P)
    control_mode = cfg["control_mode"]
    setpoint_type = cfg["setpoint_type"]
    telemetry_mode = cfg["telemetry_mode"]
    lines = _collect_banner_lines(cfg)
    lines.append("setpoint target: %s °C (%s)" % (cfg["setpoint_c_text"], setpoint_type))
    if setpoint_type == "RAMP":
        lines.append("ramp rate: %.2f °C/s" % cfg["ramp_rate"])
    if telemetry_mode == "INFO":
        lines.append("telemetry fields: status lines only")
    elif (telemetry_mode == "MPC") and (control_mode == "MPC"):
        lines.append("telemetry fields: PV[°C], SP[°C], OP[%], YH[°C], YP[°C]")
    else:
        lines.append("telemetry fields: PV[°C], SP[°C], OP[%]")
    _print_info_block(lines)


def _mode_banner_lines_from_snapshot(cfg, telemetry_mode: str):
    control_mode = str(cfg["control_mode"]).upper()
    if control_mode == "PID":
        lines = []
        variant = str(cfg["pid_variant"]).upper()
        aw_type = str(cfg["pid_aw_type"]).upper()
        algorithm = str(cfg["pid_algorithm"]).upper()

        if variant == "PID":
            lines.append("PID variant: PID (AW=%s)" % aw_type)
        else:
            lines.append("PID variant: %s" % variant)
        lines.append("PID algorithm: %s" % algorithm)

        if algorithm == "PARALLEL":
            lines.append(
                "configured PID gains: Kp=%.4g Ki=%.4g Kd=%.4g"
                % (
                    float(cfg["kp"]),
                    float(cfg["ki"]),
                    float(cfg["kd"]),
                )
            )
        else:
            lines.append(
                "configured PID gains: Kc=%.4g Ti=%.4gs Td=%.4gs"
                % (
                    float(cfg["kc"]),
                    float(cfg["ti_s"]),
                    float(cfg["td_s"]),
                )
            )
        return lines

    lines = []
    if control_mode == "FUZZY":
        lines.append("fuzzy inputs: e, de/dt (Sugeno)")
        lines.append(
            "fuzzy scales: E=%.3f °C  dE=%.3f °C/s"
            % (float(cfg["fuzzy_e_scale_c"]), float(cfg["fuzzy_de_scale_c_per_s"]))
        )
        lines.append(
            "fuzzy output shaping: du_rate_max=%.2f%%/s  de_alpha=%.2f"
            % (float(cfg["fuzzy_du_rate_max"]), float(cfg["fuzzy_de_filter_alpha"]))
        )
        return lines

    if control_mode == "MPC":
        lines.append(
            "MPC configuration: horizon=%d  grid_step=%.1f%%  du_max=%.1f%%  lambda=%.3f"
            % (
                int(cfg["mpc_horizon_steps"]),
                float(cfg["mpc_grid_step_pct"]),
                float(cfg["mpc_du_max_pct"]),
                float(cfg["mpc_lambda_move"]),
            )
        )
        if str(telemetry_mode).upper() == "MPC":
            lines.append("MPC telemetry fields: YH=one-step estimate, YP=horizon-end prediction")
    return lines


def _apply_tuned_parallel_gains(profile_mod, kp: float, ki: float, kd: float) -> None:
    from control import parallel_to_ideal_terms, series_configured_from_ideal_terms

    profile_mod.KP = float(kp)
    profile_mod.KI = float(ki)
    profile_mod.KD = float(kd)
    kc_eff, ti_eff, td_eff = parallel_to_ideal_terms(kp, ki, kd)
    algorithm = str(profile_mod.PID_ALGORITHM).upper()
    if algorithm == "SERIES":
        kc_cfg, ti_cfg, td_cfg = series_configured_from_ideal_terms(kc_eff, ti_eff, td_eff)
        profile_mod.KC = float(kc_cfg)
        profile_mod.TI_S = float(ti_cfg)
        profile_mod.TD_S = float(td_cfg)
        return
    profile_mod.KC = float(kc_eff)
    profile_mod.TI_S = 0.0 if ti_eff is None else float(ti_eff)
    profile_mod.TD_S = 0.0 if td_eff is None else float(td_eff)


def _validate_run_action(action: str, profile_mod) -> None:
    if action == "tune" and str(profile_mod.CONTROL_MODE).upper() != "PID":
        raise ValueError("tune command requires CONTROL_MODE='PID'")
    profile_mod.validate()


def _snapshot_monitor_config():
    return {
        "control_mode": str(P.CONTROL_MODE).upper(),
        "telemetry_mode": _telemetry_mode(P),
        "ts_s": max(0.05, float(P.TS_S)),
        "setpoint_c": float(P.SETPOINT_C),
        "is_ramp": (str(P.SETPOINT_TYPE).upper() == "RAMP"),
        "ramp_rate": float(P.SETPOINT_RAMP_RATE),
    }


def _snapshot_model_config():
    return {"cutoff_c": float(P.TEMP_CUTOFF_C)}


def _snapshot_relay_tuning_config():
    ts_s = float(P.TS_S)
    return {
        "telemetry_mode": _telemetry_mode(P),
        "ts_s": ts_s,
        "cutoff_c": float(P.TEMP_CUTOFF_C),
        "target_c": float(P.TUNING_TARGET_C),
    }


def _snapshot_control_config(exp_run_s):
    ts_s = float(P.TS_S)
    setpoint_type = str(P.SETPOINT_TYPE).upper()
    return {
        "control_mode": str(P.CONTROL_MODE).upper(),
        "telemetry_mode": _telemetry_mode(P),
        "control_period_ms": int(ts_s * 1000),
        "ui_period_ms": 50,
        "gc_period_ms": 3000,
        "ui_log_guard_ms": 40,
        "ts_s": ts_s,
        "dt_max_s": 5.0 * ts_s,
        "target_c": float(P.SETPOINT_C),
        "setpoint_type": setpoint_type,
        "is_ramp": (setpoint_type == "RAMP"),
        "ramp_rate": float(P.SETPOINT_RAMP_RATE),
        "cutoff_c": float(P.TEMP_CUTOFF_C),
        "dist_enable": bool(P.DIST_ENABLE),
        "dist_mode": str(P.DIST_MODE).upper(),
        "dist_mag_pct": float(P.DIST_MAG_PCT),
        "dist_start_s": float(P.DIST_START_S),
        "dist_duration_s": float(P.DIST_DURATION_S),
        "exp_run_s": exp_run_s,
        "exp_timer_enabled": (exp_run_s is not None) and (float(exp_run_s) > 0.0),
    }


def _run_monitor_loop(sensor, heater, indicator, cfg):
    """Passive telemetry monitor (no control action, heater forced OFF)."""
    ts_s = float(cfg["ts_s"])
    ctrl_period_ms = int(ts_s * 1000)
    ui_period_ms = 50
    gc_period_ms = 3000
    setpoint_c = float(cfg["setpoint_c"])
    is_ramp = bool(cfg["is_ramp"])
    ramp_rate = float(cfg["ramp_rate"])
    control_mode = str(cfg["control_mode"]).upper()
    telemetry_mode = str(cfg["telemetry_mode"]).upper()

    sensor_read = sensor.read_c
    heater_off = heater.off
    indicator_update = indicator.update

    now_ms = time.ticks_ms()
    next_ctrl_ms = now_ms
    next_ui_ms = now_ms
    next_gc_ms = now_ms
    last_temp_c = sensor_read()
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
                next_gc_ms = advance_deadline(next_gc_ms, gc_period_ms, now_ms)

            if time.ticks_diff(now_ms, next_ui_ms) >= 0:
                cmd = poll_command_nonblocking()
                if cmd is not None:
                    if cmd == "stop":
                        print("# INFO: stop command received -> monitor loop stopping")
                        break
                    if cmd in ("help", "?"):
                        print("# INFO: monitor commands: stop, help")
                indicator_update(last_temp_c, sp_current)
                next_ui_ms = advance_deadline(next_ui_ms, ui_period_ms, now_ms)

            if time.ticks_diff(now_ms, next_ctrl_ms) >= 0:
                dt_s = _compute_dt_s(now_ms, last_ctrl_exec_ms, ts_s)
                last_ctrl_exec_ms = now_ms
                sp_current = _update_setpoint(setpoint_c, sp_current, is_ramp, ramp_rate, dt_s)
                temp_c = sensor_read()
                last_temp_c = temp_c
                heater_off()
                _emit_telemetry_line(temp_c, sp_current, 0.0, telemetry_mode, control_mode)
                next_ctrl_ms = advance_deadline(next_ctrl_ms, ctrl_period_ms, now_ms)

            _yield_until_next(next_ui_ms, next_ctrl_ms, now_ms)
    except KeyboardInterrupt:
        print("\n# INFO: keyboard interrupt -> monitor loop stopped")
    _shutdown_outputs(heater, indicator)


def _run_fopdt_experiment(sensor, heater, indicator, cfg, run_state):
    from identify import run_test as run_model_test, set_model_values

    abort_cb = _make_run_abort_cb("FOPDT run", run_state)
    overtemp_cb = _make_overtemp_cb(cfg["cutoff_c"])
    telemetry_mode = _telemetry_mode(P)

    def _telemetry_cb(pv_c, sp_c, op_pct):
        _emit_telemetry_line(pv_c, sp_c, op_pct, telemetry_mode, "MODEL")

    success = False
    model = run_model_test(
        sensor,
        heater,
        indicator,
        P,
        poll_cmd=abort_cb,
        overtemp_cb=overtemp_cb,
        telemetry_cb=_telemetry_cb,
    )
    gc.collect()
    if model is not None:
        if set_model_values(P, model) is None:
            print("# WARNING: FOPDT model update failed")
        else:
            success = True
    _shutdown_outputs(heater, indicator, done=success)
    return success


def _run_pid_relay_tuning(sensor, heater, indicator, cfg, run_state):
    from identify import run_relay_tuning

    telemetry_mode = str(cfg["telemetry_mode"]).upper()

    def _telemetry_cb(pv_c, sp_c, op_pct):
        _emit_telemetry_line(pv_c, sp_c, op_pct, telemetry_mode, "PID")

    abort_cb = _make_run_abort_cb("tuning run", run_state)
    overtemp_cb = _make_overtemp_cb(cfg["cutoff_c"])
    success = False
    print("# PHASE: tuning run starting")
    gains = run_relay_tuning(
        sensor,
        heater,
        indicator,
        P,
        float(cfg["target_c"]),
        poll_cmd=abort_cb,
        telemetry_cb=_telemetry_cb,
        overtemp_cb=overtemp_cb,
    )
    if gains is None:
        _shutdown_outputs(heater, indicator)
        return False
    gc.collect()
    KP, KI, KD = gains
    if bool(run_state.get("aborted")):
        _shutdown_outputs(heater, indicator)
        return False

    _apply_tuned_parallel_gains(P, KP, KI, KD)
    success = True
    _shutdown_outputs(heater, indicator, done=success)
    return success


def _run_pid_model_tuning(heater, indicator):
    from identify import load_effective_model, run_model_tuning

    print("# PHASE: tuning model")
    model = load_effective_model(P)
    if model is None:
        print("# ERROR: tuning model requires MODEL_* values")
        _shutdown_outputs(heater, indicator)
        return False
    try:
        KP, KI, KD = run_model_tuning(P, model)
    except Exception as ex:
        print("# ERROR: tuning model failed: %s" % ex)
        _shutdown_outputs(heater, indicator)
        return False

    gc.collect()
    _apply_tuned_parallel_gains(P, KP, KI, KD)
    _shutdown_outputs(heater, indicator, done=True)
    return True


def _run_control_session(sensor, heater, indicator, controller, cfg):
    control_period_ms = int(cfg["control_period_ms"])
    ui_period_ms = int(cfg["ui_period_ms"])
    gc_period_ms = int(cfg["gc_period_ms"])
    ui_log_guard_ms = int(cfg["ui_log_guard_ms"])
    ts_s = float(cfg["ts_s"])
    dt_max_s = float(cfg["dt_max_s"])
    target_c = float(cfg["target_c"])
    is_ramp = bool(cfg["is_ramp"])
    ramp_rate = float(cfg["ramp_rate"])
    cutoff_c = float(cfg["cutoff_c"])
    dist_enable = bool(cfg["dist_enable"])
    dist_mode = str(cfg["dist_mode"]).upper()
    dist_mag_pct = float(cfg["dist_mag_pct"])
    dist_start_s = float(cfg["dist_start_s"])
    dist_duration_s = float(cfg["dist_duration_s"])
    exp_run_s = cfg["exp_run_s"]
    exp_timer_enabled = bool(cfg["exp_timer_enabled"])
    control_mode = str(cfg["control_mode"]).upper()
    telemetry_mode = str(cfg["telemetry_mode"]).upper()

    sensor_read = sensor.read_c
    heater_off = heater.off
    heater_set_percent = heater.set_percent
    indicator_update = indicator.update
    controller_update = controller.update
    is_mpc_mode = (control_mode == "MPC")
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
    if y_last >= cutoff_c:
        print("# ERROR: overtemperature at control start (PV=%.1f °C >= cutoff %.1f °C) -> run aborted"
              % (float(y_last), cutoff_c))
        _shutdown_outputs(heater, indicator)
        return False, False

    r_current = target_c if cfg["setpoint_type"] == "STEP" else y_last

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

        while True:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, next_gc_ms) >= 0:
                gc.collect()
                next_gc_ms = advance_deadline(next_gc_ms, gc_period_ms, now_ms)
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
                if time.ticks_diff(next_ctrl_ms, now_ui_ms) >= ui_log_guard_ms:
                    if dist_msg == 1:
                        print("# INFO: disturbance active (%s, dOP=%+.1f%%)" % (dist_mode, dist_mag_pct))
                        dist_msg = 0
                    elif dist_msg == 2:
                        print("# INFO: disturbance ended")
                        dist_msg = 0
                next_ui_ms = advance_deadline(next_ui_ms, ui_period_ms, now_ms)

            if time.ticks_diff(now_ms, next_ctrl_ms) >= 0:
                dt_s = _compute_dt_s(now_ms, last_ctrl_exec_ms, ts_s, dt_max_s)
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

                y = sensor_read()
                y_last = y

                r_current = _update_setpoint(target_c, r_current, is_ramp, ramp_rate, dt_s)

                if y >= cutoff_c:
                    try:
                        controller.reset()
                    except Exception:
                        pass
                    heater_off()
                    print("# ERROR: overtemperature cutoff reached (PV=%.1f °C >= cutoff %.1f °C) -> control run aborted"
                          % (float(y), cutoff_c))
                    break

                u_ctrl = controller_update(r_current, y, dt_s)
                elapsed_s = time.ticks_diff(now_ms, exp_start_ms) / 1000.0
                dist_active = _disturbance_is_active(
                    elapsed_s, dist_enable, dist_mode, dist_start_s, dist_duration_s
                )
                if dist_active and (not dist_prev_active):
                    dist_msg = 1
                if (not dist_active) and dist_prev_active and (dist_mode == "PULSE"):
                    dist_msg = 2
                dist_prev_active = dist_active
                u = u_ctrl + dist_mag_pct if dist_active else u_ctrl
                if u < 0.0:
                    u = 0.0
                elif u > 100.0:
                    u = 100.0
                heater_set_percent(u)

                if callable(controller_get_estimates):
                    try:
                        y_hat, y_pred = controller_get_estimates()
                        _emit_telemetry_line(
                            y, r_current, u, telemetry_mode, control_mode, y_hat=y_hat, y_pred=y_pred
                        )
                    except Exception:
                        _emit_telemetry_line(y, r_current, u, telemetry_mode, control_mode)
                else:
                    _emit_telemetry_line(y, r_current, u, telemetry_mode, control_mode)
                next_ctrl_ms = advance_deadline(next_ctrl_ms, control_period_ms, now_ms)

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
        try:
            _run_monitor_loop(sensor, heater, indicator, _snapshot_monitor_config())
        except Exception as ex:
            _shutdown_outputs(heater, indicator)
            print("# ERROR: monitor run failed: %s" % ex)
        continue
    if action == "model":
        try:
            _validate_run_action("model", P)
        except Exception as ex:
            print("# ERROR: model validation failed: %s" % ex)
            continue
        run_state = _new_run_state()
        try:
            _run_fopdt_experiment(sensor, heater, indicator, _snapshot_model_config(), run_state)
        except Exception as ex:
            _shutdown_outputs(heater, indicator)
            print("# ERROR: model run failed: %s" % ex)
        continue
    if action == "tune":
        try:
            _validate_run_action("tune", P)
        except Exception as ex:
            print("# ERROR: tune validation failed: %s" % ex)
            continue
        run_state = _new_run_state()
        rule = str(P.TUNING_METHOD).upper()
        if rule in P.RELAY_TUNING_METHODS:
            try:
                _run_pid_relay_tuning(sensor, heater, indicator, _snapshot_relay_tuning_config(), run_state)
            except Exception as ex:
                _shutdown_outputs(heater, indicator)
                print("# ERROR: tune run failed: %s" % ex)
        elif rule in P.MODEL_TUNING_METHODS:
            try:
                _run_pid_model_tuning(heater, indicator)
            except Exception as ex:
                _shutdown_outputs(heater, indicator)
                print("# ERROR: tune run failed: %s" % ex)
        else:
            print("# ERROR: internal tune dispatch error (unexpected TUNING_METHOD=%s)" % rule)
        continue
    if action != "control":
        print("# ERROR: unknown run action: %s" % str(action))
        continue

    try:
        _validate_run_action("control", P)
    except Exception as ex:
        print("# ERROR: control validation failed: %s" % ex)
        continue
    # Re-read timer settings after command-phase parameter edits.
    exp_run_s = P.EXPERIMENT_RUN_S
    control_cfg = _snapshot_control_config(exp_run_s)
    try:
        from control import build_controller

        controller = build_controller(P, emit_info=print)
    except Exception as ex:
        _shutdown_outputs(heater, indicator)
        print("# ERROR: controller build failed: %s" % ex)
        continue
    try:
        controller.reset()
    except Exception:
        pass

    try:
        restart_requested, ended_by_timer = _run_control_session(
            sensor, heater, indicator, controller, control_cfg
        )
    except Exception as ex:
        _shutdown_outputs(heater, indicator)
        print("# ERROR: control run failed: %s" % ex)
        continue

    if restart_requested:
        print("# INFO: restart complete")
        continue
    if ended_by_timer:
        # Protocol token used by host tools to mark run completion.
        print("# FINISH: done")

print("# INFO: heater OFF")
