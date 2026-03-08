"""FOPDT identification and runtime/profile model helpers."""

import math
import time
import gc
try:
    from machine import idle as _machine_idle
except Exception:
    _machine_idle = None

try:
    from array import array as _array
except ImportError:  # pragma: no cover
    _array = None

_METHOD_LABEL = {
    "SMITH": "Smith 28.3%/63.2% method",
    "SK": "Sundaresan-Krishnaswamy 35.3%/85.3% method",
    "BROIDA": "Broida 28.3%/40.0% method",
}


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _yield_cpu() -> None:
    if _machine_idle is not None:
        try:
            _machine_idle()
        except Exception:
            pass


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

    heater.set_percent(hold_u)
    start_ms = time.ticks_ms()
    last_ms = start_ms

    while True:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_ms) < period_ms:
            _yield_cpu()
            continue
        last_ms = now_ms

        if callable(should_abort_cb) and bool(should_abort_cb()):
            heater.off()
            elapsed_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            return (False, float("nan"), float("nan"), float(elapsed_s), "ABORT", float("nan"))

        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0
        pv_c = sensor.read_c()

        indicator.update(float(pv_c), r)
        if callable(overtemp_cb) and bool(overtemp_cb(float(pv_c), str(phase_label))):
            heater.off()
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
    """Return normalized active FOPDT model values from profile MODEL_* fields."""
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
    """Store normalized model into profile MODEL_* fields."""
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
    """Return active model from profile MODEL_* fields."""
    if profile is None:
        return None
    return get_model_values(profile)


def _moving_avg(x_list, w: int):
    if w <= 1:
        return list(x_list)
    w = int(w)
    out = []
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
        out.append(s / q_len)
    return out


def _find_time_at_fraction(t_s_list, y_list, t0_s: float, y_i: float, y_f: float, frac: float) -> float:
    """Interpolated crossing time for step fraction target."""
    target = y_i + float(frac) * (y_f - y_i)
    prev_t = None
    prev_y = None
    for ti, yi in zip(t_s_list, y_list):
        if ti < t0_s:
            prev_t = float(ti)
            prev_y = float(yi)
            continue
        cur_t = float(ti)
        cur_y = float(yi)
        if cur_y >= target:
            if (prev_t is not None) and (prev_y is not None) and (cur_t > prev_t):
                dy = cur_y - prev_y
                if abs(dy) > 1e-12:
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


def _simulate_fopdt_step(t_rel_s_list, y_i: float, u0: float, u1: float, K: float, tau: float, theta: float):
    dy = K * (u1 - u0)
    y_hat = []
    for tr in t_rel_s_list:
        tr = float(tr)
        if tr < theta:
            y_hat.append(y_i)
        else:
            x = (tr - theta) / tau
            y_hat.append(y_i + dy * (1.0 - math.exp(-x)))
    return y_hat


def _rmse(y_list, y_hat_list) -> float:
    n = min(len(y_list), len(y_hat_list))
    if n <= 0:
        return float("nan")
    s = 0.0
    for i in range(n):
        e = float(y_list[i]) - float(y_hat_list[i])
        s += e * e
    return math.sqrt(s / n)


def run_test(sensor, heater, indicator, profile, poll_cmd=None):
    """Run open-loop step test and return model dict or None."""
    u1_raw = float(profile.FOPDT_U1_PERCENT)
    u0 = 0.0
    u1 = _clamp(u1_raw, 0.0, 100.0)
    saturation_hit = (abs(u1 - u1_raw) > 1e-9)

    baseline_mode = "STEADY"
    baseline_window_s = max(float(profile.STEADY_WINDOW_S), float(profile.TS_S))
    baseline_band_c = abs(float(profile.STEADY_BAND_C))
    smooth_n = max(1, int(profile.FOPDT_SMOOTH_N))

    t_mode = str(profile.TELEMETRY_MODE).upper()
    if t_mode not in ("INFO", "NORMAL", "MPC"):
        t_mode = "NORMAL"

    def _emit_telemetry(pv_c: float, op_pct: float):
        if t_mode == "INFO":
            return
        print("PV:%.1f SP:%.1f OP:%.1f" % (float(pv_c), 0.0, float(op_pct)))

    period_ms = int(profile.TS_S * 1000)
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
    overtemp_cb = getattr(profile, "_RUN_OVERTEMP_CB", None)
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
        telemetry_cb=lambda pv, sp, op: _emit_telemetry(pv, op),
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
        heater.off()
        return None
    y_i = float(y_avg)
    last_ms = time.ticks_ms()
    heater.set_percent(u1)
    print("# PHASE: FOPDT step (baseline frozen: y0=%.3f °C, u0=0.00%%, elapsed=%.1f s)"
          % (y_i, baseline_elapsed_s))
    t0_s = time.ticks_diff(time.ticks_ms(), t_start_ms) / 1000.0
    step_window = _RollingSteadyWindow(
        window_s=float(baseline_window_s),
        max_samples=int((max(float(baseline_window_s), float(profile.TS_S)) / max(float(profile.TS_S), 1e-3)) + 8),
    )
    step_steady_reached = False
    step_steady_y = float("nan")

    step_idx = 0
    while True:
        if callable(poll_cmd) and bool(poll_cmd()):
            heater.off()
            return None
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_ms) < period_ms:
            _yield_cpu()
            continue
        last_ms = now_ms

        t_s = time.ticks_diff(now_ms, t_start_ms) / 1000.0
        t_rel = t_s - t0_s

        temp_c = sensor.read_c()
        indicator.update(temp_c, 0.0)

        if callable(overtemp_cb) and bool(overtemp_cb(float(temp_c), "FOPDT step")):
            cutoff_hit = True
            heater.off()
            print("# WARNING: FOPDT aborted: cutoff reached during step (PV=%.2f °C); model not updated"
                  % float(temp_c))
            return None

        t_list.append(float(t_s))
        y_list.append(float(temp_c))
        if (step_idx & 0x7F) == 0:
            gc.collect()
        step_idx += 1
        _emit_telemetry(temp_c, u1)
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


    heater.off()

    if len(y_list) < 10:
        print("# ERROR: FOPDT failed: not enough samples collected")
        return None

    gc.collect()
    y_s = _moving_avg(y_list, smooth_n)
    if step_steady_reached and math.isfinite(step_steady_y):
        y_f = float(step_steady_y)
    else:
        t_end = t_list[-1]
        cutoff_t = t_end - 10.0
        tail_sum = 0.0
        tail_n = 0
        for t, y in zip(t_list, y_s):
            if t >= cutoff_t:
                tail_sum += y
                tail_n += 1
        if tail_n <= 0:
            start = len(y_s) - 10
            if start < 0:
                start = 0
            for i in range(start, len(y_s)):
                tail_sum += y_s[i]
                tail_n += 1
        y_f = tail_sum / max(1, tail_n)

    du = float(u1 - u0)
    dy = float(y_f - y_i)
    if dy <= 0.0 or abs(du) < 1e-6:
        print("# ERROR: FOPDT failed: non-positive step response (dy=%.3f °C, du=%.2f%%)" % (dy, du))
        return None

    K = dy / du

    t28 = _find_time_at_fraction(t_list, y_s, t0_s, y_i, y_f, 0.283)
    t40 = _find_time_at_fraction(t_list, y_s, t0_s, y_i, y_f, 0.400)
    t63 = _find_time_at_fraction(t_list, y_s, t0_s, y_i, y_f, 0.632)
    t35 = _find_time_at_fraction(t_list, y_s, t0_s, y_i, y_f, 0.353)
    t85 = _find_time_at_fraction(t_list, y_s, t0_s, y_i, y_f, 0.853)

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
        se = 0.0
        n_err = 0
        for t, y_meas in zip(t_list, y_s):
            tr = float(t - t0_s)
            if tr < theta_i:
                y_hat = y_i
            else:
                x = (tr - theta_i) / tau_i
                y_hat = y_i + dy_step * (1.0 - math.exp(-x))
            e = float(y_meas) - float(y_hat)
            se += e * e
            n_err += 1
        rmse_i = math.sqrt(se / max(1, n_err))
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
    print("# FOPDT model")
    print("# initial steady temperature y0=%.3f °C" % y_i)
    print("# final steady temperature y1=%.3f °C" % y_f)
    print("# input step delta_u=%.1f%%" % du)
    print("# output step delta_y=%.3f °C" % dy)
    print("# process gain K=%.6f °C/%%" % K)
    print("# FOPDT candidates:")
    for method_name in ("SMITH", "SK", "BROIDA"):
        item = candidates.get(method_name, None)
        if item is None:
            print("#   %-6s unavailable (crossings not found)" % method_name)
        else:
            print("#   %-6s tau=%.2f s theta=%.2f s rmse=%.3f °C"
                  % (method_name, float(item["tau"]), float(item["theta"]), float(item["rmse"])))
    if broida_theta_clamped:
        print("# BROIDA theta was clamped to 0.0 s (raw estimate < 0)")
    print("# ========================================")
    print("# RESULT: applied method=%s (lowest RMSE): K=%.6f °C/%% tau=%.2f s theta=%.2f s rmse=%.3f °C"
          % (best_method, K, tau, theta, err))
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
