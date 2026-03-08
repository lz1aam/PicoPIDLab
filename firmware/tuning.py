"""PID tuning routines.

Internal notation:
- r: temperature reference [degC]
- y: measured temperature [degC]
- u: controller output [%]
External telemetry/CLI stays PV/SP/OP.
"""

import math
import time

try:
    from machine import idle as _machine_idle
except Exception:
    _machine_idle = None


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _yield_cpu() -> None:
    if _machine_idle is not None:
        try:
            _machine_idle()
        except Exception:
            pass


_RULE_LABEL = {
    "ZN_1_P": "Ziegler-Nichols 1 (P)",
    "ZN_1_PI": "Ziegler-Nichols 1 (PI)",
    "ZN_1_PID": "Ziegler-Nichols 1 (PID)",
    "CC_P": "Cohen-Coon (P)",
    "CC_PI": "Cohen-Coon (PI)",
    "CC_PID": "Cohen-Coon (PID)",
    "ZN_2_P": "Ziegler-Nichols 2 (P)",
    "ZN_2_PI": "Ziegler-Nichols 2 (PI)",
    "ZN_2_PID": "Ziegler-Nichols 2 (PID)",
    "TL_P": "Tyreus-Luyben (P)",
    "TL_PI": "Tyreus-Luyben (PI)",
    "TL_PID": "Tyreus-Luyben (PID)",
}


def tuning_rule_label(code: str) -> str:
    return _RULE_LABEL.get(str(code).upper(), str(code))


def _pid_views(kp: float, ki: float, kd: float, span: float):
    kp = float(kp)
    ki = float(ki)
    kd = float(kd)

    ti_s = None
    if ki > 0.0 and abs(kp) > 1e-12:
        ti_s = kp / ki
    td_s = None
    if abs(kp) > 1e-12:
        td_s = kd / kp
    elif kd == 0.0:
        td_s = 0.0
    span_val = float(span)
    if span_val <= 0.0:
        raise ValueError("SPAN must be > 0")
    pb_pct = None if kp <= 0.0 else (10000.0 / (kp * span_val))
    return {"KP": kp, "KI": ki, "KD": kd, "TI_S": ti_s, "TD_S": td_s, "PB": pb_pct, "SPAN": span_val}


def _fmt_opt(v, unit: str = ""):
    if v is None:
        return "disabled"
    return "%.6g%s" % (float(v), unit)


def _require_fopdt(model):
    if not isinstance(model, dict):
        raise ValueError("MODEL tuning requires a valid runtime FOPDT model")
    try:
        K = float(model["K"])
        tau = float(model["tau_s"])
        theta = float(model["theta_s"])
    except Exception:
        raise ValueError("MODEL tuning requires model fields: K, tau_s, theta_s")
    if abs(K) <= 1e-12:
        raise ValueError("MODEL tuning requires |K| > 0")
    if tau <= 0.0:
        raise ValueError("MODEL tuning requires tau_s > 0")
    if theta < 0.0:
        raise ValueError("MODEL tuning requires theta_s >= 0")
    return K, tau, theta


def _model_rule_set(rule: str, K: float, tau: float, theta: float):
    r = str(rule).upper()
    # Ziegler-Nichols reaction-curve (open-loop) tuning for FOPDT model.
    if r == "ZN_1_P":
        if theta <= 0.0:
            raise ValueError("ZN_1_* model tuning requires theta_s > 0")
        kc = tau / (K * theta)
        return {"KP": kc, "KI": 0.0, "KD": 0.0}
    if r == "ZN_1_PI":
        if theta <= 0.0:
            raise ValueError("ZN_1_* model tuning requires theta_s > 0")
        kc = 0.9 * tau / (K * theta)
        ti = 3.33 * theta
        return {"KP": kc, "KI": (kc / ti), "KD": 0.0}
    if r == "ZN_1_PID":
        if theta <= 0.0:
            raise ValueError("ZN_1_* model tuning requires theta_s > 0")
        kc = 1.2 * tau / (K * theta)
        ti = 2.0 * theta
        td = 0.5 * theta
        return {"KP": kc, "KI": (kc / ti), "KD": (kc * td)}

    # Cohen-Coon reaction-curve tuning (open-loop FOPDT).
    if r in ("CC_P", "CC_PI", "CC_PID"):
        if theta <= 0.0:
            raise ValueError("CC_* model tuning requires theta_s > 0")
        rho = theta / tau
        base = (1.0 / K) * (tau / theta)
        if r == "CC_P":
            kc = base * (1.0 + (rho / 3.0))
            return {"KP": kc, "KI": 0.0, "KD": 0.0}
        if r == "CC_PI":
            kc = base * (0.9 + (rho / 12.0))
            ti = theta * ((30.0 + (3.0 * rho)) / (9.0 + (20.0 * rho)))
            return {"KP": kc, "KI": (kc / ti), "KD": 0.0}
        kc = base * ((4.0 / 3.0) + (rho / 4.0))
        ti = theta * ((32.0 + (6.0 * rho)) / (13.0 + (8.0 * rho)))
        td = theta * (4.0 / (11.0 + (2.0 * rho)))
        return {"KP": kc, "KI": (kc / ti), "KD": (kc * td)}
    return None


def run_model_tuning(cfg, model):
    """Model-based tuning from FOPDT model using selected MODEL rule."""
    K, tau, theta = _require_fopdt(model)
    selected_rule = str(cfg.TUNING_RULE).upper()
    base_set = _model_rule_set(selected_rule, K, tau, theta)
    if base_set is None:
        raise ValueError("TUNING model requires TUNING_RULE in {ZN_1_*, CC_*} (current: %s)" % selected_rule)

    KP = float(base_set["KP"])
    KI = float(base_set["KI"])
    KD = float(base_set["KD"])
    if (not math.isfinite(KP)) or (not math.isfinite(KI)) or (not math.isfinite(KD)):
        raise ValueError("TUNING model produced non-finite gains")
    pb_span = float(cfg.SPAN)
    applied = _pid_views(KP, KI, KD, pb_span)

    print("# RESULT:")
    print("# ========================================")
    print("# TUNING completed")
    print("# FOPDT model used: K=%.6f °C/%% tau=%.2f s theta=%.2f s" % (K, tau, theta))
    print("# Applied set: %s" % tuning_rule_label(selected_rule))
    print("# Applied PARALLEL form: Kp=%s Ki=%s Kd=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["KI"]), _fmt_opt(applied["KD"])))
    print("# Applied IDEAL form: Kc=%s Ti=%s Td=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["TI_S"], "s"), _fmt_opt(applied["TD_S"], "s")))
    print("# ========================================")
    return KP, KI, KD


def _rule_terms(base_set: dict, mode: str):
    m = str(mode).upper()
    kp = float(base_set["KP"])
    ki = float(base_set["KI"])
    kd = float(base_set["KD"])
    if m == "P":
        return {"KP": kp, "KI": 0.0, "KD": 0.0}
    if m == "PI":
        return {"KP": kp, "KI": ki, "KD": 0.0}
    if m == "PID":
        return {"KP": kp, "KI": ki, "KD": kd}
    return None


def _ring_init(capacity: int):
    n = max(1, int(capacity))
    return {"buf": [0.0] * n, "head": 0, "count": 0}


def _ring_push(ring, value: float):
    buf = ring["buf"]
    n = len(buf)
    head = int(ring["head"])
    buf[head] = value
    head += 1
    if head >= n:
        head = 0
    ring["head"] = head
    c = int(ring["count"])
    ring["count"] = c + 1 if c < n else n


def _ring_values(ring):
    buf = ring["buf"]
    n = len(buf)
    c = int(ring["count"])
    if c <= 0:
        return []
    if c < n:
        return buf[:c]
    head = int(ring["head"])
    out = [0.0] * n
    for i in range(n):
        out[i] = float(buf[(head + i) % n])
    return out


def run_relay_tuning(sensor, heater, indicator, cfg, target_c: float, poll_cmd=None, telemetry_cb=None):
    """Relay-feedback tuning with strict two-position ON/OFF relay."""
    cycles_n = max(1, int(cfg.TUNING_CYCLES))
    r_target = float(target_c)
    band = abs(float(cfg.TUNING_BAND_C))
    lower_c = r_target - band
    upper_c = r_target + band
    u_high = 100.0
    u_low = 0.0
    d_eff = 0.5 * (u_high - u_low)

    run_timeout_s = cfg.EXPERIMENT_RUN_S
    use_timeout = (run_timeout_s is not None) and (float(run_timeout_s) > 0.0)
    timeout_ms = int(float(run_timeout_s) * 1000.0) if use_timeout else None
    current_out = 0.0

    def _set_output(out_pct: float):
        nonlocal current_out
        current_out = float(_clamp(float(out_pct), 0.0, 100.0))
        heater.set_percent(current_out)

    def _heater_off():
        nonlocal current_out
        current_out = 0.0
        heater.off()

    def _abort_tuning(msg: str, safety: bool = False):
        print("# ERROR: %s" % msg)
        try:
            cfg._RUN_ABORTED = True
        except Exception:
            pass
        _heater_off()
        return (cfg.KP, cfg.KI, cfg.KD, bool(safety))

    _set_output(u_high)
    sample_ms = max(1, int(round(float(cfg.TS_S) * 1000.0)))
    soak_start_ms = time.ticks_ms()
    next_sample_ms = soak_start_ms
    soak_peak_c = None
    soak_peak_ms = None
    overtemp_cb = getattr(cfg, "_RUN_OVERTEMP_CB", None)

    while True:
        now_ms = time.ticks_ms()
        if (timeout_ms is not None) and (time.ticks_diff(now_ms, soak_start_ms) > timeout_ms):
            return _abort_tuning("TUNING timeout during soak")
        if time.ticks_diff(now_ms, next_sample_ms) < 0:
            _yield_cpu()
            continue
        next_sample_ms = time.ticks_add(next_sample_ms, sample_ms)

        if callable(poll_cmd) and bool(poll_cmd()):
            return (cfg.KP, cfg.KI, cfg.KD, False)

        y_c = sensor.read_c()
        if callable(overtemp_cb) and bool(overtemp_cb(float(y_c), "TUNING soak")):
            return _abort_tuning("TUNING soak cutoff reached (%.2f °C)" % y_c, safety=True)
        indicator.update(y_c, r_target)
        if callable(telemetry_cb):
            try:
                telemetry_cb(float(y_c), float(r_target), float(current_out))
            except Exception:
                pass
        if y_c >= upper_c:
            soak_peak_c = float(y_c)
            soak_peak_ms = int(now_ms)
            break

    print("# PHASE: TUNING relay (target=%.2f °C band=±%.2f °C mode=ON/OFF u_high=100%% u_low=0%%)"
          % (r_target, band))

    y_c = sensor.read_c()
    is_high = y_c < r_target
    _set_output(u_high if is_high else u_low)

    keep_n = int(cycles_n + 2)
    peaks_ring = _ring_init(keep_n)
    troughs_ring = _ring_init(keep_n)
    peak_times_ring = _ring_init(keep_n)
    periods_ring = _ring_init(keep_n)
    if (soak_peak_c is not None) and (soak_peak_ms is not None):
        _ring_push(peaks_ring, float(soak_peak_c))
        _ring_push(peak_times_ring, int(soak_peak_ms))
    phase_max = float(y_c)
    phase_min = float(y_c)
    total_needed = int(cycles_n)
    start_ms = time.ticks_ms()
    sample_ms = max(1, int(round(float(cfg.TS_S) * 1000.0)))
    next_sample_ms = time.ticks_ms()
    safety_triggered = False

    while True:
        now_ms = time.ticks_ms()
        if (timeout_ms is not None) and (time.ticks_diff(now_ms, start_ms) > timeout_ms):
            return _abort_tuning("TUNING timeout")
        if time.ticks_diff(now_ms, next_sample_ms) < 0:
            _yield_cpu()
            continue
        next_sample_ms = time.ticks_add(next_sample_ms, sample_ms)

        if callable(poll_cmd) and bool(poll_cmd()):
            return (cfg.KP, cfg.KI, cfg.KD, False)

        y_c = sensor.read_c()
        if callable(overtemp_cb) and bool(overtemp_cb(float(y_c), "TUNING relay")):
            safety_triggered = True
            break
        indicator.update(y_c, r_target)

        if y_c > phase_max:
            phase_max = float(y_c)
        if y_c < phase_min:
            phase_min = float(y_c)

        next_is_high = is_high
        if y_c < lower_c:
            next_is_high = True
        elif y_c > upper_c:
            next_is_high = False

        if is_high and (not next_is_high):
            _ring_push(peaks_ring, float(phase_max))
            prev_peak_count = int(peak_times_ring["count"])
            prev_peak_time = None
            if prev_peak_count > 0:
                prev_idx = (int(peak_times_ring["head"]) - 1) % len(peak_times_ring["buf"])
                prev_peak_time = int(peak_times_ring["buf"][prev_idx])
            _ring_push(peak_times_ring, int(now_ms))
            if prev_peak_time is not None:
                T = time.ticks_diff(now_ms, prev_peak_time) / 1000.0
                _ring_push(periods_ring, float(T))
                print("# PHASE: TUNING cycle %d: period=%.1f s peak=%.2f °C"
                      % (int(periods_ring["count"]), T, phase_max))
            phase_min = float(y_c)
        elif (not is_high) and next_is_high:
            _ring_push(troughs_ring, float(phase_min))
            phase_max = float(y_c)

        is_high = next_is_high
        _set_output(u_high if is_high else u_low)

        if callable(telemetry_cb):
            try:
                telemetry_cb(float(y_c), float(r_target), float(current_out))
            except Exception:
                pass

        if int(periods_ring["count"]) >= total_needed:
            break

    _heater_off()

    peaks_c = _ring_values(peaks_ring)
    troughs_c = _ring_values(troughs_ring)
    periods_s = _ring_values(periods_ring)
    n_total = min(len(periods_s), len(troughs_c), max(0, len(peaks_c) - 1))
    if n_total < 1:
        return _abort_tuning("TUNING failed (not enough valid relay cycles)", safety=safety_triggered)
    n_keep = min(cycles_n, n_total)
    periods_keep = periods_s[:n_keep]
    troughs_keep = troughs_c[:n_keep]
    peaks_keep = peaks_c[1:1 + n_keep]
    if (len(periods_keep) < 1) or (len(troughs_keep) < 1) or (len(peaks_keep) < 1):
        return _abort_tuning("TUNING failed (cycle slicing produced empty kept set)", safety=safety_triggered)

    Pu = sum(periods_keep) / len(periods_keep)
    peak_avg = sum(peaks_keep) / len(peaks_keep)
    trough_avg = sum(troughs_keep) / len(troughs_keep)
    pv_pp = peak_avg - trough_avg
    if pv_pp <= 0.0:
        return _abort_tuning("TUNING failed (non-positive PV oscillation amplitude)", safety=safety_triggered)
    A = 0.5 * pv_pp
    pv_pp_required = 2.0 * band
    if pv_pp < pv_pp_required:
        return _abort_tuning(
            "insufficient oscillation (PV_pp=%.3f °C < %.3f °C); reduce TUNING_BAND_C or tune TUNING_TARGET_C"
            % (pv_pp, pv_pp_required),
            safety=safety_triggered,
        )
    Ku = (4.0 * d_eff) / (math.pi * A)

    Kp_zn = 0.6 * Ku
    Ti_zn = 0.5 * Pu
    Td_zn = 0.125 * Pu
    Ki_zn = (Kp_zn / Ti_zn) if Ti_zn > 0.0 else 0.0
    Kd_zn = Kp_zn * Td_zn

    Kp_tl = Ku / 2.2
    Ti_tl = 2.2 * Pu
    Td_tl = Pu / 6.3
    Ki_tl = (Kp_tl / Ti_tl) if Ti_tl > 0.0 else 0.0
    Kd_tl = Kp_tl * Td_tl

    zn_base = {"KP": Kp_zn, "KI": Ki_zn, "KD": Kd_zn}
    tl_base = {"KP": Kp_tl, "KI": Ki_tl, "KD": Kd_tl}
    sets = {
        "ZN_2_P": _rule_terms(zn_base, "P"),
        "ZN_2_PI": _rule_terms(zn_base, "PI"),
        "ZN_2_PID": _rule_terms(zn_base, "PID"),
        "TL_P": _rule_terms(tl_base, "P"),
        "TL_PI": _rule_terms(tl_base, "PI"),
        "TL_PID": _rule_terms(tl_base, "PID"),
    }
    requested_rule = str(cfg.TUNING_RULE).upper()
    selected_rule = requested_rule
    if selected_rule not in sets:
        return _abort_tuning("TUNING relay requires TUNING_RULE in {ZN_2_*, TL_*} (current: %s)" % requested_rule)

    pb_span = float(cfg.SPAN)
    base_set = sets.get(selected_rule, sets["ZN_2_PID"])
    KP = float(base_set["KP"])
    KI = float(base_set["KI"])
    KD = float(base_set["KD"])

    print("# RESULT:")
    print("# ========================================")
    print("# TUNING completed")
    print("# relay metrics: A=%.3f °C d_eff=%.3f%% Ku=%.3f Pu=%.3f s PV_pp=%.3f °C"
          % (A, d_eff, Ku, Pu, pv_pp))
    print("# relay summary: type=2pos-onoff band=±%.2f °C u_high=%.1f%% u_low=%.1f%% cycles_used=%d"
          % (band, u_high, u_low, len(periods_keep)))
    print("# Applied set: %s" % tuning_rule_label(selected_rule))
    applied = _pid_views(KP, KI, KD, pb_span)
    print("# Applied PARALLEL form: Kp=%s Ki=%s Kd=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["KI"]), _fmt_opt(applied["KD"])))
    print("# Applied IDEAL form: Kc=%s Ti=%s Td=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["TI_S"], "s"), _fmt_opt(applied["TD_S"], "s")))
    print("# ========================================")

    return (KP, KI, KD, safety_triggered)
