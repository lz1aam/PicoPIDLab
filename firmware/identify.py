"""identify.py - FOPDT identification and PID tuning helpers."""

import math
import time
import gc

from hardware import yield_cpu, advance_deadline

try:
    from array import array as _array
except ImportError:  # pragma: no cover
    _array = None

_METHOD_LABEL = {
    "SMITH": "Smith 28.3%/63.2% method",
    "SK": "Sundaresan-Krishnaswamy 35.3%/85.3% method",
    "BROIDA": "Broida 28.3%/40.0% method",
}

_CROSSING_INTERP_EPS = 1e-12
_SATURATION_HIT_EPS = 1e-9
_TS_FLOOR_EPS = 1e-3
_STEP_RESPONSE_EPS = 1e-6
_IDENT_UI_PERIOD_MS = 50
_IDENT_GC_PERIOD_MS = 3000


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _service_gc(now_ms: int, next_gc_ms: int, mem_free_fn, gc_low_mem_bytes, last_emergency_gc_ms: int):
    if time.ticks_diff(now_ms, next_gc_ms) >= 0:
        gc.collect()
        return advance_deadline(next_gc_ms, _IDENT_GC_PERIOD_MS, now_ms), last_emergency_gc_ms
    if gc_low_mem_bytes is None:
        return next_gc_ms, last_emergency_gc_ms
    try:
        if (mem_free_fn() < gc_low_mem_bytes) and (time.ticks_diff(now_ms, last_emergency_gc_ms) >= 500):
            gc.collect()
            return next_gc_ms, now_ms
    except Exception:
        return next_gc_ms, last_emergency_gc_ms
    return next_gc_ms, last_emergency_gc_ms


def tuning_method_label(code: str) -> str:
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


class _RollingSteadyWindow:
    """Bounded rolling window for steady-state checks."""

    __slots__ = (
        "_window_s", "_cap", "_t", "_pv", "_op", "_start", "_count",
        "_sum_pv", "_sum_op",
    )

    def __init__(self, window_s: float, max_samples: int):
        self._window_s = max(0.1, float(window_s))
        self._cap = max(8, int(max_samples))
        self._t = [0.0] * self._cap
        self._pv = [0.0] * self._cap
        self._op = [0.0] * self._cap
        self._start = 0
        self._count = 0
        self._sum_pv = 0.0
        self._sum_op = 0.0

    def append(self, t_s: float, pv_c: float, op_pct: float) -> None:
        t = float(t_s)
        pv = float(pv_c)
        op = float(op_pct)
        if self._count < self._cap:
            idx = (self._start + self._count) % self._cap
            self._count += 1
        else:
            idx = self._start
            self._sum_pv -= self._pv[idx]
            self._sum_op -= self._op[idx]
            self._start = (self._start + 1) % self._cap
        self._t[idx] = t
        self._pv[idx] = pv
        self._op[idx] = op
        self._sum_pv += pv
        self._sum_op += op

        cutoff = t - self._window_s
        while self._count > 1 and self._t[self._start] < cutoff:
            self._sum_pv -= self._pv[self._start]
            self._sum_op -= self._op[self._start]
            self._start = (self._start + 1) % self._cap
            self._count -= 1

    def stats(self):
        n = self._count
        if n < 2:
            return (False, float("nan"), float("nan"), 0.0, float("nan"))
        i0 = self._start
        i1 = (self._start + self._count - 1) % self._cap
        t0 = float(self._t[i0])
        t1 = float(self._t[i1])
        span_s = float(t1 - t0)
        if span_s <= 0.0:
            return (False, float("nan"), float("nan"), 0.0, float("nan"))

        count = self._count
        pv_avg = self._sum_pv / count
        op_avg = self._sum_op / count
        pv_min = float("inf")
        pv_max = float("-inf")
        for j in range(self._count):
            i = (self._start + j) % self._cap
            yv = float(self._pv[i])
            if yv < pv_min:
                pv_min = yv
            if yv > pv_max:
                pv_max = yv
        pv_p2p = float(pv_max - pv_min) if (pv_max >= pv_min) else float("nan")
        return (True, float(pv_avg), float(op_avg), float(span_s), pv_p2p)


def _wait_for_steady(
    sensor,
    heater,
    indicator,
    cfg,
    target_sp,
    u_hold_pct,
    window_s,
    should_abort_cb=None,
    overtemp_cb=None,
    telemetry_cb=None,
    phase_label="steady",
):
    ts_s = max(0.02, float(cfg.TS_S))
    period_ms = max(1, int(round(ts_s * 1000.0)))
    window_s = max(0.1, float(window_s))
    band_c = abs(float(cfg.STEADY_BAND_C))

    hold_u = _clamp(float(u_hold_pct), 0.0, 100.0)
    r = float(target_sp)

    max_samples = int((max(window_s, ts_s) / ts_s) + 8)
    win = _RollingSteadyWindow(window_s=float(window_s), max_samples=max_samples)
    sensor_read = sensor.read_c
    indicator_update = indicator.update
    heater_set_percent = heater.set_percent
    heater_off = heater.off
    ui_period_ms = _IDENT_UI_PERIOD_MS
    mem_free_fn = getattr(gc, "mem_free", None)
    gc_low_mem_bytes = 24 * 1024 if callable(mem_free_fn) else None

    heater_set_percent(hold_u)
    start_ms = time.ticks_ms()
    now_ms = start_ms
    next_sample_ms = start_ms
    next_ui_ms = start_ms
    next_gc_ms = start_ms
    last_emergency_gc_ms = start_ms
    last_pv_c = float("nan")

    while True:
        now_ms = time.ticks_ms()
        next_gc_ms, last_emergency_gc_ms = _service_gc(
            now_ms, next_gc_ms, mem_free_fn, gc_low_mem_bytes, last_emergency_gc_ms
        )

        if time.ticks_diff(now_ms, next_ui_ms) >= 0:
            if _isfinite(last_pv_c):
                indicator_update(last_pv_c, r)
            if callable(should_abort_cb) and bool(should_abort_cb()):
                heater_off()
                elapsed_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
                return (False, float("nan"), float("nan"), float(elapsed_s), "ABORT", float("nan"))
            next_ui_ms = advance_deadline(next_ui_ms, ui_period_ms, now_ms)

        if time.ticks_diff(now_ms, next_sample_ms) < 0:
            yield_cpu()
            continue
        next_sample_ms = advance_deadline(next_sample_ms, period_ms, now_ms)

        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0
        pv_c = sensor_read()
        last_pv_c = float(pv_c)

        if callable(overtemp_cb) and bool(overtemp_cb(float(pv_c), str(phase_label))):
            heater_off()
            return (False, float("nan"), float("nan"), float(elapsed_s), "CUTOFF", float(pv_c))

        win.append(float(elapsed_s), float(pv_c), hold_u)

        if callable(telemetry_cb):
            try:
                telemetry_cb(float(pv_c), r, hold_u)
            except Exception:
                pass

        ok_stats, y_avg, u_avg, span_s, pv_p2p = win.stats()
        window_ready = (
            ok_stats
            and (float(span_s) >= float(window_s))
            and (float(pv_p2p) <= (2.0 * band_c))
        )
        if window_ready:
            return (True, float(y_avg), float(u_avg), float(elapsed_s), "STEADY", float("nan"))


def _normalize_model(raw):
    """Normalize model dict to canonical numeric keys used by controllers."""
    if not isinstance(raw, dict):
        return None

    K = raw.get("K")
    tau_s = raw.get("tau_s")
    theta_s = raw.get("theta_s")
    u0_pct = raw.get("u0_pct")
    y0 = raw.get("y0")
    if (K is None) or (tau_s is None) or (theta_s is None) or (u0_pct is None) or (y0 is None):
        return None

    try:
        out = dict(raw)
        out["K"] = float(K)
        out["tau_s"] = float(tau_s)
        out["theta_s"] = float(theta_s)
        out["u0_pct"] = float(u0_pct)
        out["y0"] = float(y0)
        return out
    except Exception:
        return None


def get_model_values(profile):
    """Return normalized active FOPDT model values from config MODEL_* fields."""
    raw = {
        "K": profile.MODEL_K,
        "tau_s": profile.MODEL_TAU_S,
        "theta_s": profile.MODEL_THETA_S,
        "u0_pct": profile.MODEL_U0_PCT,
        "y0": profile.MODEL_Y0,
        "y1": profile.MODEL_Y1,
        "rmse": profile.MODEL_RMSE,
        "method": profile.MODEL_METHOD,
    }
    return _normalize_model(raw)


def set_model_values(profile, model):
    """Store normalized model into config MODEL_* fields."""
    model_norm = _normalize_model(model)
    if model_norm is None:
        return None
    try:
        setattr(profile, "MODEL_K", float(model_norm["K"]))
        setattr(profile, "MODEL_TAU_S", float(model_norm["tau_s"]))
        setattr(profile, "MODEL_THETA_S", float(model_norm["theta_s"]))
        setattr(profile, "MODEL_U0_PCT", float(model_norm["u0_pct"]))
        setattr(profile, "MODEL_Y0", float(model_norm["y0"]))
        if "y1" in model_norm:
            setattr(profile, "MODEL_Y1", float(model_norm["y1"]))
        if "rmse" in model_norm:
            setattr(profile, "MODEL_RMSE", float(model_norm["rmse"]))
        if "method" in model_norm:
            setattr(profile, "MODEL_METHOD", str(model_norm["method"]))
    except Exception:
        return None
    return dict(model_norm)


def load_effective_model(profile=None):
    """Return active model from config MODEL_* fields."""
    if profile is None:
        return None
    return get_model_values(profile)


def _pid_views(kp: float, ki: float, kd: float, span: float):
    kp = float(kp)
    ki = float(ki)
    kd = float(kd)

    ti_s = None
    if ki > 0.0 and abs(kp) > _CROSSING_INTERP_EPS:
        ti_s = kp / ki
    td_s = None
    if abs(kp) > _CROSSING_INTERP_EPS:
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
    if abs(K) <= _CROSSING_INTERP_EPS:
        raise ValueError("MODEL tuning requires |K| > 0")
    if tau <= 0.0:
        raise ValueError("MODEL tuning requires tau_s > 0")
    if theta < 0.0:
        raise ValueError("MODEL tuning requires theta_s >= 0")
    return K, tau, theta


def _model_rule_set(rule: str, K: float, tau: float, theta: float):
    r = str(rule).upper()
    if r == "ZN1_P":
        if theta <= 0.0:
            raise ValueError("ZN1_* model tuning requires theta_s > 0")
        kc = tau / (K * theta)
        return {"KP": kc, "KI": 0.0, "KD": 0.0}
    if r == "ZN1_PI":
        if theta <= 0.0:
            raise ValueError("ZN1_* model tuning requires theta_s > 0")
        kc = 0.9 * tau / (K * theta)
        ti = 3.33 * theta
        return {"KP": kc, "KI": (kc / ti), "KD": 0.0}
    if r == "ZN1_PID":
        if theta <= 0.0:
            raise ValueError("ZN1_* model tuning requires theta_s > 0")
        kc = 1.2 * tau / (K * theta)
        ti = 2.0 * theta
        td = 0.5 * theta
        return {"KP": kc, "KI": (kc / ti), "KD": (kc * td)}

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
    selected_rule = str(cfg.TUNING_METHOD).upper()
    base_set = _model_rule_set(selected_rule, K, tau, theta)
    if base_set is None:
        raise ValueError("TUNING model requires TUNING_METHOD in {ZN1_*, CC_*} (current: %s)" % selected_rule)

    KP = float(base_set["KP"])
    KI = float(base_set["KI"])
    KD = float(base_set["KD"])
    if (not math.isfinite(KP)) or (not math.isfinite(KI)) or (not math.isfinite(KD)):
        raise ValueError("TUNING model produced non-finite gains")
    pb_span = float(cfg.SPAN)
    applied = _pid_views(KP, KI, KD, pb_span)

    print("# RESULT:")
    print("# ========================================")
    print("# PID tuning")
    print("# model used: K=%.6f °C/%% tau=%.2f s theta=%.2f s" % (K, tau, theta))
    print("# tuning method: %s" % tuning_method_label(selected_rule))
    print("# applied parallel gains: Kp=%s Ki=%s Kd=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["KI"]), _fmt_opt(applied["KD"])))
    print("# applied ideal gains: Kc=%s Ti=%s Td=%s"
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


def _moving_avg_iter(x_list, w: int):
    """Yield the same prefix/rolling average sequence as the old _moving_avg()."""
    if w <= 1:
        for v in x_list:
            yield float(v)
        return

    w = int(w)
    s = 0.0
    q = [0.0] * w
    q_len = 0
    q_head = 0
    for v in x_list:
        v = float(v)
        if q_len < w:
            q[q_len] = v
            q_len += 1
            s += v
        else:
            s -= q[q_head]
            q[q_head] = v
            s += v
            q_head += 1
            if q_head >= w:
                q_head = 0
        yield (s / q_len)


def _find_time_at_fraction_stream(t_s_list, y_raw_list, smooth_n: int, t0_s: float, y_i: float, y_f: float, frac: float) -> float:
    """Interpolated crossing time for a streamed moving-average response."""
    target = y_i + float(frac) * (y_f - y_i)
    prev_t = None
    prev_y = None
    for ti, yi in zip(t_s_list, _moving_avg_iter(y_raw_list, smooth_n)):
        if ti < t0_s:
            prev_t = float(ti)
            prev_y = float(yi)
            continue
        cur_t = float(ti)
        cur_y = float(yi)
        if cur_y >= target:
            if (prev_t is not None) and (prev_y is not None) and (cur_t > prev_t):
                dy = cur_y - prev_y
                if abs(dy) > _CROSSING_INTERP_EPS:
                    a = (target - prev_y) / dy
                    if a < 0.0:
                        a = 0.0
                    elif a > 1.0:
                        a = 1.0
                    return float((prev_t + a * (cur_t - prev_t)) - t0_s)
            return float(cur_t - t0_s)
        prev_t = cur_t
        prev_y = cur_y
    return float("nan")


def _tail_avg_from_smoothed(t_s_list, y_raw_list, smooth_n: int, cutoff_t: float) -> float:
    """Return tail average of streamed moving-average values for t >= cutoff_t."""
    tail_sum = 0.0
    tail_n = 0
    last_vals = _ring_init(10)
    for t_s, y_s in zip(t_s_list, _moving_avg_iter(y_raw_list, smooth_n)):
        _ring_push(last_vals, float(y_s))
        if float(t_s) >= float(cutoff_t):
            tail_sum += float(y_s)
            tail_n += 1
    if tail_n > 0:
        return tail_sum / tail_n
    vals = _ring_values(last_vals)
    if not vals:
        return float("nan")
    return sum(float(v) for v in vals) / float(len(vals))


def _rmse_for_candidate_stream(
    t_s_list,
    y_raw_list,
    smooth_n: int,
    t0_s: float,
    y_i: float,
    dy_step: float,
    tau_i: float,
    theta_i: float,
) -> float:
    """Return candidate RMSE against streamed moving-average values."""
    se = 0.0
    n_err = 0
    for t_s, y_meas in zip(t_s_list, _moving_avg_iter(y_raw_list, smooth_n)):
        tr = float(t_s) - float(t0_s)
        if tr < float(theta_i):
            y_hat = float(y_i)
        else:
            x = (tr - float(theta_i)) / float(tau_i)
            y_hat = float(y_i) + float(dy_step) * (1.0 - math.exp(-x))
        e = float(y_meas) - float(y_hat)
        se += e * e
        n_err += 1
    return math.sqrt(se / max(1, n_err))


def run_test(sensor, heater, indicator, profile, poll_cmd=None, overtemp_cb=None, telemetry_cb=None):
    """Run open-loop step test and return model dict or None."""
    u1_raw = float(profile.FOPDT_U1_PERCENT)
    u0 = 0.0
    u1 = _clamp(u1_raw, 0.0, 100.0)
    saturation_hit = (abs(u1 - u1_raw) > _SATURATION_HIT_EPS)

    baseline_mode = "STEADY"
    baseline_window_s = max(float(profile.STEADY_WINDOW_S), float(profile.TS_S))
    baseline_band_c = abs(float(profile.STEADY_BAND_C))
    smooth_n = max(1, int(profile.FOPDT_SMOOTH_N))

    t_mode = str(profile.TELEMETRY_MODE).upper()
    if t_mode not in ("INFO", "NORMAL", "MPC"):
        t_mode = "NORMAL"

    sensor_read = sensor.read_c
    indicator_update = indicator.update
    heater_set_percent = heater.set_percent
    heater_off = heater.off
    ui_period_ms = _IDENT_UI_PERIOD_MS
    mem_free_fn = getattr(gc, "mem_free", None)
    gc_low_mem_bytes = 24 * 1024 if callable(mem_free_fn) else None

    period_ms = max(1, int(round(float(profile.TS_S) * 1000.0)))
    print("# INFO:")
    print("# ========================================")
    print("# FOPDT mode (open-loop identification)")
    print("# step: %.1f%% -> %.1f%%" % (u0, u1))
    print("# baseline mode=STEADY (window=%.1f s, band=±%.3f °C)"
          % (baseline_window_s, baseline_band_c))
    if t_mode == "INFO":
        print("# telemetry mode: INFO (status lines only)")
    else:
        print("# telemetry fields: PV[°C], SP[°C], OP[%]")
    print("# ========================================")

    t_start_ms = time.ticks_ms()
    baseline_elapsed_s = 0.0
    cutoff_hit = False
    if _array is not None:
        t_list = _array("f")
        y_list = _array("f")
    else:
        t_list = []
        y_list = []

    print("# PHASE: FOPDT initial steady-state capture (OP=0%)")
    ok, y_avg, _u_avg, elapsed_s, steady_reason, steady_pv = _wait_for_steady(
        sensor=sensor,
        heater=heater,
        indicator=indicator,
        cfg=profile,
        target_sp=0.0,
        u_hold_pct=0.0,
        window_s=baseline_window_s,
        should_abort_cb=poll_cmd,
        overtemp_cb=overtemp_cb,
        telemetry_cb=telemetry_cb,
        phase_label="FOPDT initial steady-state capture",
    )
    baseline_elapsed_s = float(elapsed_s)
    if not ok:
        if steady_reason == "CUTOFF":
            print("# WARNING: FOPDT aborted: cutoff reached during baseline capture (PV=%.2f °C); model not updated"
                  % float(steady_pv))
        elif steady_reason == "ABORT":
            print("# INFO: FOPDT aborted during baseline capture")
        else:
            print("# WARNING: FOPDT aborted: baseline steady window not reached; model not updated")
        heater_off()
        return None
    y_i = float(y_avg)
    now_ms = time.ticks_ms()
    next_sample_ms = now_ms
    next_ui_ms = now_ms
    next_gc_ms = now_ms
    last_emergency_gc_ms = now_ms
    last_temp_c = float(y_i)
    heater_set_percent(u1)
    print("# PHASE: FOPDT step (baseline frozen: y0=%.3f °C, u0=0.00%%, elapsed=%.1f s)"
          % (y_i, baseline_elapsed_s))
    t0_s = time.ticks_diff(time.ticks_ms(), t_start_ms) / 1000.0
    step_window = _RollingSteadyWindow(
        window_s=float(baseline_window_s),
        max_samples=int((max(float(baseline_window_s), float(profile.TS_S)) / max(float(profile.TS_S), _TS_FLOOR_EPS)) + 8),
    )
    step_steady_reached = False
    step_steady_y = float("nan")

    while True:
        now_ms = time.ticks_ms()
        next_gc_ms, last_emergency_gc_ms = _service_gc(
            now_ms, next_gc_ms, mem_free_fn, gc_low_mem_bytes, last_emergency_gc_ms
        )

        if time.ticks_diff(now_ms, next_ui_ms) >= 0:
            indicator_update(last_temp_c, 0.0)
            if callable(poll_cmd) and bool(poll_cmd()):
                heater_off()
                return None
            next_ui_ms = advance_deadline(next_ui_ms, ui_period_ms, now_ms)

        if time.ticks_diff(now_ms, next_sample_ms) < 0:
            yield_cpu()
            continue
        next_sample_ms = advance_deadline(next_sample_ms, period_ms, now_ms)

        t_s = time.ticks_diff(now_ms, t_start_ms) / 1000.0
        t_rel = t_s - t0_s

        temp_c = sensor_read()
        last_temp_c = float(temp_c)

        if callable(overtemp_cb) and bool(overtemp_cb(float(temp_c), "FOPDT step")):
            cutoff_hit = True
            heater_off()
            print("# WARNING: FOPDT aborted: cutoff reached during step (PV=%.2f °C); model not updated"
                  % float(temp_c))
            return None

        t_list.append(float(t_s))
        y_list.append(float(temp_c))
        if callable(telemetry_cb):
            try:
                telemetry_cb(float(temp_c), 0.0, float(u1))
            except Exception:
                pass
        step_window.append(float(t_rel), float(temp_c), float(u1))

        ok_stats, y_avg_step, _u_avg_step, span_step, p2p_step = step_window.stats()
        window_ready = (
            ok_stats
            and (float(span_step) >= float(baseline_window_s))
            and (float(p2p_step) <= (2.0 * baseline_band_c))
        )
        moved_from_baseline = ok_stats and (abs(float(y_avg_step) - float(y_i)) >= (2.0 * baseline_band_c))
        if window_ready and moved_from_baseline:
            step_steady_reached = True
            step_steady_y = float(y_avg_step)
            print("# PHASE: FOPDT steady-state reached after step (y1=%.3f °C, p2p=%.3f °C)"
                  % (step_steady_y, float(p2p_step)))
            break


    heater_off()

    if len(y_list) < 10:
        print("# ERROR: FOPDT failed: not enough samples collected")
        return None

    gc.collect()
    if step_steady_reached and math.isfinite(step_steady_y):
        y_f = float(step_steady_y)
    else:
        t_end = t_list[-1]
        cutoff_t = t_end - 10.0
        y_f = _tail_avg_from_smoothed(t_list, y_list, smooth_n, cutoff_t)

    du = float(u1 - u0)
    dy = float(y_f - y_i)
    if dy <= 0.0 or abs(du) < _STEP_RESPONSE_EPS:
        print("# ERROR: FOPDT failed: non-positive step response (dy=%.3f °C, du=%.2f%%)" % (dy, du))
        return None

    K = dy / du

    t28 = _find_time_at_fraction_stream(t_list, y_list, smooth_n, t0_s, y_i, y_f, 0.283)
    t40 = _find_time_at_fraction_stream(t_list, y_list, smooth_n, t0_s, y_i, y_f, 0.400)
    t63 = _find_time_at_fraction_stream(t_list, y_list, smooth_n, t0_s, y_i, y_f, 0.632)
    t35 = _find_time_at_fraction_stream(t_list, y_list, smooth_n, t0_s, y_i, y_f, 0.353)
    t85 = _find_time_at_fraction_stream(t_list, y_list, smooth_n, t0_s, y_i, y_f, 0.853)

    candidates = {}
    broida_theta_clamped = False

    if math.isfinite(t28) and math.isfinite(t63) and (t63 > t28):
        tau_sm = 1.5 * (t63 - t28)
        theta_sm = max(0.0, t63 - tau_sm)
        if tau_sm > 0.0:
            candidates["SMITH"] = {"tau": float(tau_sm), "theta": float(theta_sm)}

    if math.isfinite(t28) and math.isfinite(t40) and (t40 > t28):
        tau_br = 5.5 * (t40 - t28)
        theta_br_raw = t28 - 0.33 * tau_br
        theta_br = max(0.0, theta_br_raw)
        if theta_br_raw < 0.0:
            broida_theta_clamped = True
        if tau_br > 0.0:
            candidates["BROIDA"] = {"tau": float(tau_br), "theta": float(theta_br)}

    if math.isfinite(t35) and math.isfinite(t85) and (t85 > t35):
        tau_sk = 0.67 * (t85 - t35)
        theta_sk = max(0.0, (1.3 * t35 - 0.29 * t85))
        if tau_sk > 0.0:
            candidates["SK"] = {"tau": float(tau_sk), "theta": float(theta_sk)}

    if len(candidates) <= 0:
        print("# ERROR: FOPDT failed: no valid crossing pair found for SMITH/SK/BROIDA")
        return None

    dy_step = K * (u1 - u0)
    best_method = None
    best_err = None
    for method_name in ("SMITH", "SK", "BROIDA"):
        item = candidates.get(method_name, None)
        if item is None:
            continue
        tau_i = float(item["tau"])
        theta_i = float(item["theta"])
        rmse_i = _rmse_for_candidate_stream(
            t_list, y_list, smooth_n, t0_s, y_i, dy_step, tau_i, theta_i
        )
        item["rmse"] = float(rmse_i)
        if (best_err is None) or (rmse_i < best_err):
            best_err = float(rmse_i)
            best_method = str(method_name)

    gc.collect()
    tau = float(candidates[best_method]["tau"])
    theta = float(candidates[best_method]["theta"])
    err = float(candidates[best_method]["rmse"])

    print("# RESULT:")
    print("# ========================================")
    print("# FOPDT identification")
    print("# initial steady temperature: %.3f °C" % y_i)
    print("# final steady temperature: %.3f °C" % y_f)
    print("# input step: %.1f%% -> %.1f%%" % (u0, u1))
    print("# output step: %.3f °C" % dy)
    print("# process gain: %.6f °C/%%" % K)
    print("# candidate models:")
    for method_name in ("SMITH", "SK", "BROIDA"):
        item = candidates.get(method_name, None)
        display_name = "Smith" if method_name == "SMITH" else ("Broida" if method_name == "BROIDA" else method_name)
        if item is None:
            print("#   %s: unavailable (crossings not found)" % display_name)
        else:
            print("#   %s: tau=%.2f s theta=%.2f s rmse=%.3f °C"
                  % (display_name, float(item["tau"]), float(item["theta"]), float(item["rmse"])))
    if broida_theta_clamped:
        print("# note: Broida theta was clamped to 0.0 s (raw estimate < 0)")
    print("# selected method: %s (lowest RMSE)" % ("Smith" if best_method == "SMITH" else ("Broida" if best_method == "BROIDA" else best_method)))
    print("# applied model: K=%.6f °C/%% tau=%.2f s theta=%.2f s rmse=%.3f °C"
          % (K, tau, theta, err))
    print("# ========================================")
    model = {
        "K": float(K),
        "tau_s": float(tau),
        "theta_s": float(theta),
        "rmse": float(err),
        "u0_pct": float(u0),
        "u1_pct": float(u1),
        "y0": float(y_i),
        "y1": float(y_f),
        "method": str(best_method),
        "baseline_mode": str(baseline_mode),
        "baseline_elapsed_s": float(baseline_elapsed_s),
        "tail_stop_used": bool(step_steady_reached),
        "crossings_found": True,
        "candidates": candidates,
        "saturation_hit": bool(saturation_hit),
        "cutoff_hit": bool(cutoff_hit),
    }
    return model


def run_relay_tuning(
    sensor, heater, indicator, cfg, target_c: float, poll_cmd=None, telemetry_cb=None, overtemp_cb=None
):
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
    sensor_read = sensor.read_c
    indicator_update = indicator.update
    heater_set_percent = heater.set_percent
    heater_off = heater.off
    ui_period_ms = _IDENT_UI_PERIOD_MS
    mem_free_fn = getattr(gc, "mem_free", None)
    gc_low_mem_bytes = 24 * 1024 if callable(mem_free_fn) else None

    def _set_output(out_pct: float):
        nonlocal current_out
        current_out = float(_clamp(float(out_pct), 0.0, 100.0))
        heater_set_percent(current_out)

    def _heater_off():
        nonlocal current_out
        current_out = 0.0
        heater_off()

    def _abort_tuning(msg: str):
        print("# ERROR: %s" % msg)
        _heater_off()
        return None

    _set_output(u_high)
    sample_ms = max(1, int(round(float(cfg.TS_S) * 1000.0)))
    soak_start_ms = time.ticks_ms()
    next_sample_ms = soak_start_ms
    next_ui_ms = soak_start_ms
    next_gc_ms = soak_start_ms
    last_emergency_gc_ms = soak_start_ms
    soak_peak_c = None
    soak_peak_ms = None
    last_y_c = float("nan")
    while True:
        now_ms = time.ticks_ms()
        next_gc_ms, last_emergency_gc_ms = _service_gc(
            now_ms, next_gc_ms, mem_free_fn, gc_low_mem_bytes, last_emergency_gc_ms
        )

        if time.ticks_diff(now_ms, next_ui_ms) >= 0:
            if _isfinite(last_y_c):
                indicator_update(last_y_c, r_target)
            if callable(poll_cmd) and bool(poll_cmd()):
                _heater_off()
                return None
            next_ui_ms = advance_deadline(next_ui_ms, ui_period_ms, now_ms)

        if (timeout_ms is not None) and (time.ticks_diff(now_ms, soak_start_ms) > timeout_ms):
            return _abort_tuning("TUNING timeout during soak")
        if time.ticks_diff(now_ms, next_sample_ms) < 0:
            yield_cpu()
            continue
        next_sample_ms = advance_deadline(next_sample_ms, sample_ms, now_ms)

        y_c = sensor_read()
        last_y_c = float(y_c)
        if callable(overtemp_cb) and bool(overtemp_cb(float(y_c), "TUNING soak")):
            return _abort_tuning("TUNING soak cutoff reached (%.2f °C)" % y_c)
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

    y_c = sensor_read()
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
    next_ui_ms = next_sample_ms
    next_gc_ms = next_sample_ms
    last_emergency_gc_ms = next_sample_ms
    safety_triggered = False
    safety_temp_c = float("nan")

    while True:
        now_ms = time.ticks_ms()
        next_gc_ms, last_emergency_gc_ms = _service_gc(
            now_ms, next_gc_ms, mem_free_fn, gc_low_mem_bytes, last_emergency_gc_ms
        )

        if time.ticks_diff(now_ms, next_ui_ms) >= 0:
            indicator_update(y_c, r_target)
            if callable(poll_cmd) and bool(poll_cmd()):
                _heater_off()
                return None
            next_ui_ms = advance_deadline(next_ui_ms, ui_period_ms, now_ms)

        if (timeout_ms is not None) and (time.ticks_diff(now_ms, start_ms) > timeout_ms):
            return _abort_tuning("TUNING timeout")
        if time.ticks_diff(now_ms, next_sample_ms) < 0:
            yield_cpu()
            continue
        next_sample_ms = advance_deadline(next_sample_ms, sample_ms, now_ms)

        y_c = sensor_read()
        if callable(overtemp_cb) and bool(overtemp_cb(float(y_c), "TUNING relay")):
            safety_triggered = True
            safety_temp_c = float(y_c)
            break

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

    if safety_triggered:
        return _abort_tuning("TUNING relay cutoff reached (%.2f °C)" % safety_temp_c)

    peaks_c = _ring_values(peaks_ring)
    troughs_c = _ring_values(troughs_ring)
    periods_s = _ring_values(periods_ring)
    n_total = min(len(periods_s), len(troughs_c), max(0, len(peaks_c) - 1))
    if n_total < 1:
        return _abort_tuning("TUNING failed (not enough valid relay cycles)")
    n_keep = min(cycles_n, n_total)
    periods_keep = periods_s[:n_keep]
    troughs_keep = troughs_c[:n_keep]
    peaks_keep = peaks_c[1:1 + n_keep]
    if (len(periods_keep) < 1) or (len(troughs_keep) < 1) or (len(peaks_keep) < 1):
        return _abort_tuning("TUNING failed (cycle slicing produced empty kept set)")

    Pu = sum(periods_keep) / len(periods_keep)
    peak_avg = sum(peaks_keep) / len(peaks_keep)
    trough_avg = sum(troughs_keep) / len(troughs_keep)
    pv_pp = peak_avg - trough_avg
    if pv_pp <= 0.0:
        return _abort_tuning("TUNING failed (non-positive PV oscillation amplitude)")
    A = 0.5 * pv_pp
    pv_pp_required = 2.0 * band
    if pv_pp < pv_pp_required:
        return _abort_tuning(
            "insufficient oscillation (PV_pp=%.3f °C < %.3f °C); reduce TUNING_BAND_C or tune TUNING_TARGET_C"
            % (pv_pp, pv_pp_required)
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
        "ZN2_P": _rule_terms(zn_base, "P"),
        "ZN2_PI": _rule_terms(zn_base, "PI"),
        "ZN2_PID": _rule_terms(zn_base, "PID"),
        "TL_P": _rule_terms(tl_base, "P"),
        "TL_PI": _rule_terms(tl_base, "PI"),
        "TL_PID": _rule_terms(tl_base, "PID"),
    }
    requested_rule = str(cfg.TUNING_METHOD).upper()
    selected_rule = requested_rule
    if selected_rule not in sets:
        return _abort_tuning("TUNING relay requires TUNING_METHOD in {ZN2_*, TL_*} (current: %s)" % requested_rule)

    pb_span = float(cfg.SPAN)
    base_set = sets.get(selected_rule, sets["ZN2_PID"])
    KP = float(base_set["KP"])
    KI = float(base_set["KI"])
    KD = float(base_set["KD"])

    print("# RESULT:")
    print("# ========================================")
    print("# PID tuning")
    print("# relay metrics: A=%.3f °C d_eff=%.3f%% Ku=%.3f Pu=%.3f s PV_pp=%.3f °C"
          % (A, d_eff, Ku, Pu, pv_pp))
    print("# relay summary: type=2pos-onoff band=±%.2f °C u_high=%.1f%% u_low=%.1f%% cycles used=%d"
          % (band, u_high, u_low, len(periods_keep)))
    print("# tuning method: %s" % tuning_method_label(selected_rule))
    applied = _pid_views(KP, KI, KD, pb_span)
    print("# applied parallel gains: Kp=%s Ki=%s Kd=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["KI"]), _fmt_opt(applied["KD"])))
    print("# applied ideal gains: Kc=%s Ti=%s Td=%s"
          % (_fmt_opt(applied["KP"]), _fmt_opt(applied["TI_S"], "s"), _fmt_opt(applied["TD_S"], "s")))
    print("# ========================================")

    return (KP, KI, KD)
