# controllers.py
"""controllers.py - Control algorithms (hardware-agnostic)

This module intentionally contains *no* direct hardware access. It implements
student-friendly controllers that output a percentage (0..100%), typically
mapped to a heater PWM duty cycle.

Controllers provided
--------------------
- TwoPositionPercent:
    Two-position (ON/OFF) thermostat with hysteresis and optional minimum switch
    time (anti-chatter).

- PIDParallelPercent:
    Parallel PID with selectable anti-windup mode:
    NONE (naive windup), CLAMP (conditional integration), BACKCALC.

- PID2DOFPercent:
    Two-degree-of-freedom PID with setpoint weighting (beta).

- PIDFeedForwardPercent (FF_PID):
    Feedforward + PID combined cleanly against actuator saturation.

- GainScheduledPIDPercent (GAIN_SCHED):
    PID with gains interpolated from a breakpoint table vs y or r.

- FuzzySugenoIncrementalPercent:
    Lightweight fuzzy logic controller (Sugeno singletons) using error and
    error-rate inputs, with an editable 5x5 rule table. Output is incremental
    and returned as a percent (0..100).

- MPCLitePercent:
    Simple prediction + constraints controller using an internal FOPDT model.
Units
-----
- r (setpoint/reference), y (measurement): °C
- dt: seconds
- u (output): percent (%), typically mapped to PWM duty
- External telemetry/CLI mapping: y <-> PV, r <-> SP, u <-> OP
"""

_EPSILON = 1e-6
_LARGE_TIME = 1e9  # large initial value to allow immediate first switch

import math

try:
    from array import array as _array
except Exception:
    _array = None


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _mf5(x: float):
    """Return 5 membership degrees for x in [-1, 1].

    Order: NB, NS, Z, PS, PB.
    Uses symmetric shoulder/triangle partitions with breakpoints at:
        -1, -0.5, 0, 0.5, 1
    NB/PB are shoulder sets: once x reaches +/-1, membership saturates at 1.0.
    """
    x = float(x)
    if x <= -1.0:
        return (1.0, 0.0, 0.0, 0.0, 0.0)
    if x < -0.5:
        nb = (-0.5 - x) / 0.5
        ns = (x + 1.0) / 0.5
        return (nb, ns, 0.0, 0.0, 0.0)
    if x < 0.0:
        ns = (-x) / 0.5
        z = (x + 0.5) / 0.5
        return (0.0, ns, z, 0.0, 0.0)
    if x < 0.5:
        z = (0.5 - x) / 0.5
        ps = x / 0.5
        return (0.0, 0.0, z, ps, 0.0)
    if x < 1.0:
        ps = (1.0 - x) / 0.5
        pb = (x - 0.5) / 0.5
        return (0.0, 0.0, 0.0, ps, pb)
    return (0.0, 0.0, 0.0, 0.0, 1.0)


def _series_int_clamp(x_int: float, integral_limit, kc: float) -> float:
    """Clamp normalized series integrator state using output-contribution limit."""
    if integral_limit is None:
        return x_int
    kc_abs = abs(float(kc))
    if kc_abs < _EPSILON:
        return 0.0
    lim_x = float(integral_limit) / kc_abs
    return _clamp(float(x_int), -lim_x, lim_x)


def _normalize_output_bounds(out_min: float, out_max: float):
    out_min_v = float(out_min)
    out_max_v = float(out_max)
    if out_max_v <= out_min_v:
        raise ValueError("out_max must be > out_min")
    return out_min_v, out_max_v


def _normalize_rate_limit(rate_limit):
    if rate_limit is None:
        return None
    rl = float(rate_limit)
    return rl if rl > 0.0 else None


def _normalize_integral_limit(integral_limit):
    if integral_limit is None:
        return None
    lim = float(integral_limit)
    return abs(lim) if lim > 0.0 else None


class ControllerBase:
    """Required interface for all controllers.

    update(sp_c, y_c, dt_s) -> float (0..100 percent)
    reset() -> None
    """

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class PIDParallelPercent(ControllerBase):
    """Parallel PID with selectable anti-windup policy."""

    __slots__ = (
        "kp", "ki", "kd", "aw_type", "kaw",
        "out_min", "out_max",
        "d_filter_alpha", "rate_limit", "integral_limit",
        "_integral", "_prev_meas", "_d_filt", "_prev_output",
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        aw_type: str = "CLAMP",
        kaw: float = 0.0,
        out_min: float = 0.0,
        out_max: float = 100.0,
        d_filter_alpha: float = 0.1,
        rate_limit=None,
        integral_limit=None,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.aw_type = str(aw_type).upper()
        if self.aw_type not in ("NONE", "CLAMP", "BACKCALC"):
            raise ValueError("aw_type must be NONE, CLAMP, or BACKCALC")
        self.kaw = float(kaw)
        if self.aw_type == "BACKCALC" and ((not _isfinite(self.kaw)) or (self.kaw < 0.0)):
            raise ValueError("kaw must be >= 0 for aw_type='BACKCALC'")

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.d_filter_alpha = float(_clamp(float(d_filter_alpha), 0.0, 1.0))

        self.rate_limit = _normalize_rate_limit(rate_limit)
        self.integral_limit = _normalize_integral_limit(integral_limit)

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._d_filt = 0.0
        self._prev_output = 0.0

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        kp = self.kp
        ki = self.ki
        kd = self.kd
        aw_type = self.aw_type
        kaw = self.kaw
        out_min = self.out_min
        out_max = self.out_max
        d_filter_alpha = self.d_filter_alpha
        rate_limit = self.rate_limit
        integral_limit = self.integral_limit
        prev_output = float(self._prev_output)

        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return prev_output

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return prev_output

        e = r - y
        p = kp * e

        if self._prev_meas is None:
            d_raw = 0.0
        else:
            dy_dt = (y - self._prev_meas) / dt_s
            d_raw = -kd * dy_dt

        a = d_filter_alpha
        self._d_filt += a * (d_raw - self._d_filt)
        d = self._d_filt

        if aw_type == "BACKCALC":
            u_unclamped = p + self._integral + d
            u_sat = float(_clamp(u_unclamped, out_min, out_max))
            self._integral += (ki * e * dt_s) + (kaw * (u_sat - u_unclamped) * dt_s)
            if integral_limit is not None:
                self._integral = _clamp(self._integral, -integral_limit, integral_limit)
        else:
            i_candidate = self._integral + (ki * e * dt_s)
            if integral_limit is not None:
                i_candidate = _clamp(i_candidate, -integral_limit, integral_limit)
            allow_integrate = True
            if aw_type == "CLAMP":
                u_unclamped = p + i_candidate + d
                if (u_unclamped > out_max) and (e > 0.0):
                    allow_integrate = False
                if (u_unclamped < out_min) and (e < 0.0):
                    allow_integrate = False
            if allow_integrate:
                self._integral = i_candidate

        u = p + self._integral + d
        u = float(_clamp(u, out_min, out_max))

        if rate_limit is not None:
            max_change = rate_limit * dt_s
            delta = u - prev_output
            if delta > max_change:
                u = prev_output + max_change
            elif delta < -max_change:
                u = prev_output - max_change
            u = float(_clamp(u, out_min, out_max))

        self._prev_meas = y
        self._prev_output = u
        return float(u)


class PIDSeriesPercent(ControllerBase):
    """Series/interacting PID with selectable anti-windup policy."""

    __slots__ = (
        "kc", "ti_s", "td_s", "aw_type", "kaw",
        "out_min", "out_max",
        "d_filter_alpha", "rate_limit", "integral_limit",
        "_x_int", "_prev_pi", "_d_filt", "_prev_output",
    )

    def __init__(
        self,
        kc: float,
        ti_s: float,
        td_s: float,
        aw_type: str = "CLAMP",
        kaw: float = 0.0,
        out_min: float = 0.0,
        out_max: float = 100.0,
        d_filter_alpha: float = 0.1,
        rate_limit=None,
        integral_limit=None,
    ):
        self.kc = float(kc)
        self.ti_s = float(ti_s)
        self.td_s = float(td_s)
        self.aw_type = str(aw_type).upper()
        if self.aw_type not in ("NONE", "CLAMP", "BACKCALC"):
            raise ValueError("aw_type must be NONE, CLAMP, or BACKCALC")
        self.kaw = float(kaw)
        if self.aw_type == "BACKCALC" and ((not _isfinite(self.kaw)) or (self.kaw < 0.0)):
            raise ValueError("kaw must be >= 0 for aw_type='BACKCALC'")

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.d_filter_alpha = float(_clamp(float(d_filter_alpha), 0.0, 1.0))
        self.rate_limit = _normalize_rate_limit(rate_limit)
        self.integral_limit = _normalize_integral_limit(integral_limit)

        self.reset()

    def reset(self):
        self._x_int = 0.0
        self._prev_pi = None
        self._d_filt = 0.0
        self._prev_output = 0.0

    def _series_d(self, pi_value: float, dt_s: float) -> float:
        if self.td_s <= 0.0:
            self._d_filt = 0.0
            return 0.0
        if self._prev_pi is None:
            d_raw = 0.0
        else:
            d_raw = self.kc * self.td_s * ((pi_value - self._prev_pi) / dt_s)
        a = self.d_filter_alpha
        self._d_filt += a * (d_raw - self._d_filt)
        return self._d_filt

    def _predict_d_from_pi(self, pi_value: float, dt_s: float) -> float:
        if self.td_s <= 0.0:
            return 0.0
        if self._prev_pi is None:
            d_raw_pred = 0.0
        else:
            d_raw_pred = self.kc * self.td_s * ((pi_value - self._prev_pi) / dt_s)
        return self._d_filt + (self.d_filter_alpha * (d_raw_pred - self._d_filt))

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return float(self._prev_output)

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return float(self._prev_output)

        e = r - y
        if self.aw_type == "NONE":
            if self.ti_s > 0.0:
                self._x_int += ((e / self.ti_s) * dt_s)
                self._x_int = _series_int_clamp(self._x_int, self.integral_limit, self.kc)
        elif self.aw_type == "BACKCALC":
            pi_value_pred = e + self._x_int
            d_pred = self._predict_d_from_pi(pi_value_pred, dt_s)
            u_unclamped = (self.kc * pi_value_pred) + d_pred
            u_sat = float(_clamp(u_unclamped, self.out_min, self.out_max))
            if self.ti_s > 0.0:
                self._x_int += ((e / self.ti_s) * dt_s)
                if abs(self.kc) > _EPSILON:
                    self._x_int += ((self.kaw / self.kc) * (u_sat - u_unclamped) * dt_s)
                self._x_int = _series_int_clamp(self._x_int, self.integral_limit, self.kc)
        else:
            x_candidate = self._x_int
            if self.ti_s > 0.0:
                x_candidate = self._x_int + ((e / self.ti_s) * dt_s)
                x_candidate = _series_int_clamp(x_candidate, self.integral_limit, self.kc)

            pi_candidate = e + x_candidate
            u_unclamped = (self.kc * pi_candidate) + self._predict_d_from_pi(pi_candidate, dt_s)
            saturating_high = u_unclamped > self.out_max
            saturating_low = u_unclamped < self.out_min
            allow_integrate = True
            if saturating_high and e > 0.0:
                allow_integrate = False
            if saturating_low and e < 0.0:
                allow_integrate = False
            if allow_integrate and self.ti_s > 0.0:
                self._x_int = x_candidate

        pi_value = e + self._x_int
        if self.aw_type == "BACKCALC":
            # Keep BACKCALC internally coherent in this step:
            # use the same derivative prediction form for anti-windup correction
            # and final output, then commit the filter state once.
            d = self._predict_d_from_pi(pi_value, dt_s)
            self._d_filt = d
        else:
            d = self._series_d(pi_value, dt_s)
        u = float(_clamp((self.kc * pi_value) + d, self.out_min, self.out_max))

        if self.rate_limit is not None:
            max_change = self.rate_limit * dt_s
            delta = u - self._prev_output
            if delta > max_change:
                u = self._prev_output + max_change
            elif delta < -max_change:
                u = self._prev_output - max_change
            u = float(_clamp(u, self.out_min, self.out_max))

        self._prev_pi = pi_value
        self._prev_output = u
        return float(u)



class PID2DOFPercent(ControllerBase):
    """Two-degree-of-freedom (2DOF) PID with setpoint weighting.

    Common industrial form:
      - P term uses weighted reference: e_p = beta*r - y
      - I term uses full error:         e_i = r - y
      - D term on measurement:          -kd*dy/dt

    beta in [0, 1] is the reference weight:
      beta=1.0 -> standard PID
      beta<1.0 -> reduced setpoint overshoot (less aggressive on r steps)
    """

    __slots__ = (
        "kp", "ki", "kd", "beta",
        "out_min", "out_max",
        "d_filter_alpha", "rate_limit", "integral_limit",
        "_integral", "_prev_meas", "_d_filt", "_prev_output",
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        beta: float = 1.0,
        out_min: float = 0.0,
        out_max: float = 100.0,
        d_filter_alpha: float = 0.1,
        rate_limit=None,
        integral_limit=None,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.beta = float(_clamp(float(beta), 0.0, 1.0))

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.d_filter_alpha = float(_clamp(float(d_filter_alpha), 0.0, 1.0))

        self.rate_limit = _normalize_rate_limit(rate_limit)
        self.integral_limit = _normalize_integral_limit(integral_limit)

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._d_filt = 0.0
        self._prev_output = 0.0

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return float(self._prev_output)

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return float(self._prev_output)

        e_i = r - y
        e_p = (self.beta * r) - y

        p = self.kp * e_p

        if self._prev_meas is None:
            d_raw = 0.0
        else:
            dy_dt = (y - self._prev_meas) / dt_s
            d_raw = -self.kd * dy_dt

        a = self.d_filter_alpha
        self._d_filt += a * (d_raw - self._d_filt)
        d = self._d_filt

        i_candidate = self._integral + (self.ki * e_i * dt_s)
        if self.integral_limit is not None:
            i_candidate = _clamp(i_candidate, -self.integral_limit, self.integral_limit)

        u_unclamped = p + i_candidate + d
        saturating_high = u_unclamped > self.out_max
        saturating_low = u_unclamped < self.out_min

        allow_integrate = True
        if saturating_high and e_i > 0.0:
            allow_integrate = False
        if saturating_low and e_i < 0.0:
            allow_integrate = False
        if allow_integrate:
            self._integral = i_candidate

        u = p + self._integral + d
        u = float(_clamp(u, self.out_min, self.out_max))

        if self.rate_limit is not None:
            max_change = self.rate_limit * dt_s
            delta = u - self._prev_output
            if delta > max_change:
                u = self._prev_output + max_change
            elif delta < -max_change:
                u = self._prev_output - max_change
            u = float(_clamp(u, self.out_min, self.out_max))

        self._prev_meas = y
        self._prev_output = u
        return float(u)


class PIDFeedForwardPercent(ControllerBase):
    """Feedforward + PID (single output, heater only).

    Total output:
        u = clamp(u_ff + u_fb, out_min..out_max)

    Feedforward options
    -------------------
    MANUAL:
        u_ff = ff_bias + ff_gain_pct_per_c * (r - ambient)

    FOPDT_GAIN:
        Uses a FOPDT steady-state gain K (°C/%):
        u_ff = u0 + (r - ambient) / K

    Notes
    -----
    - This controller applies anti-windup against the *total* saturation so the
      integrator does not wind up when feedforward pushes the actuator into its
      bounds.
    """

    __slots__ = (
        "kp", "ki", "kd",
        "ff_mode", "ff_gain", "ff_bias", "K_proc", "u0_pct", "ambient_init",
        "out_min", "out_max",
        "d_filter_alpha", "rate_limit", "integral_limit",
        "_integral", "_prev_meas", "_d_filt", "_prev_output",
        "_ambient",
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        ff_mode: str = "MANUAL",
        ff_gain_pct_per_c: float = 0.0,
        ff_bias_pct: float = 0.0,
        K_c_per_pct: float = 0.0,
        u0_pct: float = 0.0,
        ambient_c=None,
        out_min: float = 0.0,
        out_max: float = 100.0,
        d_filter_alpha: float = 0.1,
        rate_limit=None,
        integral_limit=None,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.ff_mode = str(ff_mode).upper()
        self.ff_gain = float(ff_gain_pct_per_c)
        self.ff_bias = float(ff_bias_pct)
        self.K_proc = float(K_c_per_pct)
        self.u0_pct = float(u0_pct)

        self.ambient_init = ambient_c  # may be None
        self._ambient = None

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.d_filter_alpha = float(_clamp(float(d_filter_alpha), 0.0, 1.0))

        self.rate_limit = _normalize_rate_limit(rate_limit)
        self.integral_limit = _normalize_integral_limit(integral_limit)

        if self.ff_mode not in ("MANUAL", "FOPDT_GAIN"):
            raise ValueError("ff_mode must be 'MANUAL' or 'FOPDT_GAIN'")

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._d_filt = 0.0
        self._prev_output = 0.0
        self._ambient = None

    def _compute_ambient(self, y: float):
        if self._ambient is not None:
            return self._ambient
        if self.ambient_init is None:
            self._ambient = float(y)
        else:
            self._ambient = float(self.ambient_init)
        return self._ambient

    def _feedforward(self, r: float, ambient: float) -> float:
        if self.ff_mode == "MANUAL":
            return float(self.ff_bias + (self.ff_gain * (r - ambient)))
        # FOPDT_GAIN
        if abs(self.K_proc) < 1e-9:
            return float(self.ff_bias)
        return float(self.u0_pct + ((r - ambient) / self.K_proc))

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return float(self._prev_output)

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return float(self._prev_output)

        ambient = self._compute_ambient(y)
        u_ff = self._feedforward(r, ambient)

        # Heater-only: do not allow negative feedforward contribution.
        # (The total output will be clamped anyway.)
        # u_ff itself may be negative depending on settings, so clamp modestly.
        u_ff = float(_clamp(u_ff, -1000.0, 1000.0))

        e = r - y

        p = self.kp * e

        if self._prev_meas is None:
            d_raw = 0.0
        else:
            dy_dt = (y - self._prev_meas) / dt_s
            d_raw = -self.kd * dy_dt

        a = self.d_filter_alpha
        self._d_filt += a * (d_raw - self._d_filt)
        d = self._d_filt

        i_candidate = self._integral + (self.ki * e * dt_s)
        if self.integral_limit is not None:
            i_candidate = _clamp(i_candidate, -self.integral_limit, self.integral_limit)

        u_fb_unclamped = p + i_candidate + d
        u_unclamped = u_ff + u_fb_unclamped

        saturating_high = u_unclamped > self.out_max
        saturating_low = u_unclamped < self.out_min

        allow_integrate = True
        if saturating_high and e > 0.0:
            allow_integrate = False
        if saturating_low and e < 0.0:
            allow_integrate = False

        if allow_integrate:
            self._integral = i_candidate

        u_fb = p + self._integral + d
        u = float(_clamp(u_ff + u_fb, self.out_min, self.out_max))

        if self.rate_limit is not None:
            max_change = self.rate_limit * dt_s
            delta = u - self._prev_output
            if delta > max_change:
                u = self._prev_output + max_change
            elif delta < -max_change:
                u = self._prev_output - max_change
            u = float(_clamp(u, self.out_min, self.out_max))

        self._prev_meas = y
        self._prev_output = u
        return float(u)


class GainScheduledPIDPercent(ControllerBase):
    """Gain-scheduled PID (piecewise-linear gain table).

    A gain schedule is a list of breakpoints:
        (temp_c, Kp, Ki, Kd)

    At runtime, gains are interpolated linearly between breakpoints based on
    either y or r (selectable). The controller is otherwise a standard PID
    with derivative-on-measurement and conditional-integration anti-windup.

    Note:
    The internal integral contribution is rescaled when Ki changes to keep
    integral action consistent across schedule regions. This can still produce
    a visible output step when crossing breakpoints.
    """

    __slots__ = (
        "schedule", "var",
        "out_min", "out_max",
        "d_filter_alpha", "rate_limit", "integral_limit",
        "kp", "ki", "kd",
        "_integral", "_prev_meas", "_d_filt", "_prev_output",
    )

    def __init__(
        self,
        schedule_table,
        schedule_variable: str = "PV",
        out_min: float = 0.0,
        out_max: float = 100.0,
        d_filter_alpha: float = 0.1,
        rate_limit=None,
        integral_limit=None,
    ):
        self.schedule = self._parse_schedule(schedule_table)
        self.var = str(schedule_variable).upper()
        if self.var not in ("PV", "SP"):
            raise ValueError("schedule_variable must be 'PV' or 'SP'")

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.d_filter_alpha = float(_clamp(float(d_filter_alpha), 0.0, 1.0))

        self.rate_limit = _normalize_rate_limit(rate_limit)
        self.integral_limit = _normalize_integral_limit(integral_limit)

        # Active gains (initialized from first breakpoint)
        t0, kp0, ki0, kd0 = self.schedule[0]
        self.kp = kp0
        self.ki = ki0
        self.kd = kd0

        self.reset()

    @staticmethod
    def _parse_schedule(schedule_table):
        if schedule_table is None:
            raise ValueError("schedule_table is required")
        if len(schedule_table) < 2:
            raise ValueError("schedule_table must have at least 2 breakpoints")
        pts = []
        for i, row in enumerate(schedule_table):
            if len(row) != 4:
                raise ValueError("schedule_table[%d] must be (temp,kp,ki,kd)" % i)
            t, kp, ki, kd = row
            t = float(t)
            kp = float(kp)
            ki = float(ki)
            kd = float(kd)
            pts.append((t, kp, ki, kd))
        pts.sort(key=lambda x: x[0])
        return pts

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._d_filt = 0.0
        self._prev_output = 0.0

    def _interp_gains(self, temp: float):
        pts = self.schedule
        if temp <= pts[0][0]:
            return pts[0][1], pts[0][2], pts[0][3]
        if temp >= pts[-1][0]:
            return pts[-1][1], pts[-1][2], pts[-1][3]
        for i in range(len(pts) - 1):
            t0, kp0, ki0, kd0 = pts[i]
            t1, kp1, ki1, kd1 = pts[i + 1]
            if t0 <= temp <= t1:
                if abs(t1 - t0) < 1e-9:
                    return kp1, ki1, kd1
                a = (temp - t0) / (t1 - t0)
                kp = kp0 + a * (kp1 - kp0)
                ki = ki0 + a * (ki1 - ki0)
                kd = kd0 + a * (kd1 - kd0)
                return kp, ki, kd
        return pts[-1][1], pts[-1][2], pts[-1][3]

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return float(self._prev_output)

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return float(self._prev_output)

        sched_var = y if (self.var == "PV") else r
        old_ki = float(self.ki)
        kp, ki, kd = self._interp_gains(float(sched_var))
        if (old_ki > _EPSILON) and (ki > _EPSILON) and (abs(ki - old_ki) > 1e-12):
            self._integral *= (ki / old_ki)
            if self.integral_limit is not None:
                self._integral = _clamp(self._integral, -self.integral_limit, self.integral_limit)
        self.kp = kp
        self.ki = ki
        self.kd = kd

        e = r - y
        p = self.kp * e

        if self._prev_meas is None:
            d_raw = 0.0
        else:
            dy_dt = (y - self._prev_meas) / dt_s
            d_raw = -self.kd * dy_dt

        a = self.d_filter_alpha
        self._d_filt += a * (d_raw - self._d_filt)
        d = self._d_filt

        i_candidate = self._integral + (self.ki * e * dt_s)
        if self.integral_limit is not None:
            i_candidate = _clamp(i_candidate, -self.integral_limit, self.integral_limit)

        u_unclamped = p + i_candidate + d
        saturating_high = u_unclamped > self.out_max
        saturating_low = u_unclamped < self.out_min

        allow_integrate = True
        if saturating_high and e > 0.0:
            allow_integrate = False
        if saturating_low and e < 0.0:
            allow_integrate = False
        if allow_integrate:
            self._integral = i_candidate

        u = p + self._integral + d
        u = float(_clamp(u, self.out_min, self.out_max))

        if self.rate_limit is not None:
            max_change = self.rate_limit * dt_s
            delta = u - self._prev_output
            if delta > max_change:
                u = self._prev_output + max_change
            elif delta < -max_change:
                u = self._prev_output - max_change
            u = float(_clamp(u, self.out_min, self.out_max))

        self._prev_meas = y
        self._prev_output = u
        return float(u)


class MPCLitePercent(ControllerBase):
    """Very small "MPC-lite" controller for a FOPDT heating plant.

    This is a deliberately simplified, computationally cheap MPC demonstration:

    - Uses an internal FOPDT model (K, tau, theta).
    - At each step, evaluates a small grid of candidate outputs within:
        - absolute bounds: [out_min..out_max]
        - move constraint: |u - u_prev| <= du_max_pct
    - Predicts y over a horizon with 2-move blocking:
        u[k], u[k+1], then hold u[k+1] for the rest of the horizon.
      This keeps search complexity O(M^2) over grid points M (instead of O(M^N)
      for an unconstrained N-step move sequence), which is practical on RP2040.
    - Minimizes:
        J = sum_k (r - y_pred[k])^2 + lambda_move * (u - u_prev)^2

    Standard discrete equations used here (first-order + dead-time model)
    ---------------------------------------------------------------------
    State/update:
        x[k+1] = alpha*x[k] + beta*K*(u_del[k] - u0)
        alpha = exp(-dt/tau), beta = 1 - alpha
        y[k]   = y0 + x[k]

    Cost for each candidate sequence [u0, u1]:
        J = sum_{i=1..N} (r - y_pred[i])^2
            + lambda_move * ((u0 - u_prev)^2 + (u1 - u0)^2)

    Notes
    -----
    - Heating only: out_min is typically 0.
    - This is not intended to be "industrial MPC"; it is meant to show the
      *idea* of prediction + constraints with approachable code.
    """

    __slots__ = (
        "K_proc", "tau_s", "theta_s", "u0_pct",
        "dt_nominal_s",
        "horizon_steps", "grid_step_pct", "du_max_pct", "lambda_move", "max_candidates",
        "observer_gain",
        "out_min", "out_max",
        "y0_runtime_capture",
        "rate_limit",
        "_alpha_nom", "_beta_nom",
        "_n_delay", "_u_hist", "_u_head",
        "_y0", "_x_hat", "_y_hat_last", "_y_pred_end_last", "_prev_output", "_pts_buf",
    )

    def __init__(
        self,
        K_c_per_pct: float,
        tau_s: float,
        theta_s: float,
        u0_pct: float,
        dt_nominal_s: float,
        horizon_steps: int = 15,
        grid_step_pct: float = 5.0,
        du_max_pct: float = 20.0,
        lambda_move: float = 0.1,
        max_candidates: int = 41,
        observer_gain: float = 0.5,
        out_min: float = 0.0,
        out_max: float = 100.0,
        y0_runtime_capture: bool = False,
        rate_limit=None,
        y0_c=None,
    ):
        self.K_proc = float(K_c_per_pct)
        self.tau_s = float(tau_s)
        self.theta_s = max(0.0, float(theta_s))
        self.u0_pct = float(u0_pct)
        self.dt_nominal_s = max(_EPSILON, float(dt_nominal_s))

        self.horizon_steps = int(horizon_steps)
        self.grid_step_pct = float(grid_step_pct)
        self.du_max_pct = float(du_max_pct)
        self.lambda_move = float(lambda_move)
        self.max_candidates = int(max_candidates)
        self.observer_gain = float(observer_gain)

        if (self.horizon_steps < 3) or (self.horizon_steps > 200):
            raise ValueError("horizon_steps must be in 3..200")
        if self.grid_step_pct <= 0.0:
            raise ValueError("grid_step_pct must be > 0")
        if self.du_max_pct <= 0.0:
            raise ValueError("du_max_pct must be > 0")
        if self.lambda_move < 0.0:
            raise ValueError("lambda_move must be >= 0")
        if (self.max_candidates < 3) or (self.max_candidates > 201):
            raise ValueError("max_candidates must be in 3..201")
        if (self.observer_gain < 0.0) or (self.observer_gain > 1.0):
            raise ValueError("observer_gain must be in 0..1")
        if (abs(self.K_proc) < 1e-9) or (self.tau_s <= 0.0):
            raise ValueError("invalid model parameters (K and tau required)")

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.y0_runtime_capture = bool(y0_runtime_capture)

        self.rate_limit = _normalize_rate_limit(rate_limit)

        # Exact ZOH discretization for first-order model:
        # x[k+1] = alpha*x[k] + (1-alpha)*K*(u_del-u0), alpha = exp(-dt/tau)
        self._alpha_nom = float(math.exp(-self.dt_nominal_s / self.tau_s))
        self._beta_nom = float(1.0 - self._alpha_nom)

        # Use ceil to avoid underestimating dead-time when theta is between samples.
        self._n_delay = int(math.ceil(self.theta_s / self.dt_nominal_s))
        if self._n_delay < 0:
            self._n_delay = 0

        self._y0 = float(y0_c) if (y0_c is not None) else None
        self._pts_buf = []

        self.reset()

    def reset(self):
        self._prev_output = 0.0
        self._x_hat = 0.0
        self._y_hat_last = 0.0
        self._y_pred_end_last = 0.0
        if self._n_delay <= 0:
            self._u_hist = []
            self._u_head = 0
        else:
            hist = self._u_hist if hasattr(self, "_u_hist") else None
            if (hist is None) or (len(hist) != self._n_delay):
                if _array is not None:
                    self._u_hist = _array("f", [0.0] * self._n_delay)
                else:
                    self._u_hist = [0.0] * self._n_delay
            else:
                for i in range(self._n_delay):
                    self._u_hist[i] = 0.0
            self._u_head = 0
        # y0 baseline is preserved if already set, otherwise learned on first update

    def warm_start(self, y_c: float, u_c: float = 0.0):
        """Initialize state and delay line from current plant conditions.

        This aligns predictor state with measured y at run start and avoids
        large transient mismatch after mode switches or hot starts.
        """
        y = float(y_c)
        u = float(_clamp(float(u_c), self.out_min, self.out_max))
        y0 = self._ensure_y0(y)
        self._prev_output = u
        self._x_hat = float(y - y0)
        self._y_hat_last = float(y0 + self._x_hat)
        self._y_pred_end_last = float(self._y_hat_last)
        if self._n_delay > 0:
            for i in range(self._n_delay):
                self._u_hist[i] = float(u)
            self._u_head = 0

    def _ensure_y0(self, y: float):
        if self._y0 is None:
            if self.y0_runtime_capture:
                self._y0 = float(y)
            else:
                raise ValueError("MPC y0 baseline is undefined; provide y0_c or enable y0_runtime_capture")
            self._x_hat = 0.0
        return self._y0

    def _build_candidates(self, lo: float, hi: float, center: float):
        step = self.grid_step_pct
        n_pts_max = self.max_candidates
        pts = self._pts_buf
        del pts[:]
        pts.append(float(lo))
        u = math.floor(lo / step) * step
        if u <= lo + 1e-9:
            u += step
        while u <= hi + 1e-9 and len(pts) < n_pts_max:
            pts.append(float(u))
            u += step
        if (abs(pts[-1] - hi) > 1e-6) and (len(pts) < n_pts_max):
            pts.append(float(hi))
        center_clamped = float(_clamp(center, lo, hi))
        if (len(pts) < n_pts_max) and all(abs(p - center_clamped) > 1e-6 for p in pts):
            pts.append(center_clamped)
        return list(pts)

    def _simulate_cost(self, r: float, y0: float, x0: float, u0: float, u1: float):
        x = float(x0)
        # Keep model discretization fixed to nominal sample period so
        # dead-time steps (n_delay) remain time-consistent under loop jitter.
        alpha = self._alpha_nom
        beta = self._beta_nom
        K_proc = self.K_proc
        u0_pct = self.u0_pct

        # Move penalty on planned sequence increments.
        u_prev = float(self._prev_output)
        move_cost = self.lambda_move * (((float(u0) - u_prev) ** 2) + ((float(u1) - float(u0)) ** 2))

        cost = 0.0
        n_delay = self._n_delay
        horizon_steps = self.horizon_steps
        if n_delay > 0:
            head = int(self._u_head)
            hist = self._u_hist
            for k in range(horizon_steps):
                # Current state estimate already used hist[head] at "now".
                # For prediction from next sample onward, consume queued delayed
                # inputs starting at head+1, then apply planned sequence.
                if k < (n_delay - 1):
                    u_del = float(hist[(head + 1 + k) % n_delay])
                else:
                    j = k - (n_delay - 1)
                    u_del = float(u0) if j <= 0 else float(u1)

                x = alpha * x + beta * (K_proc * (u_del - u0_pct))
                y_pred = y0 + x
                e = r - y_pred
                cost += e * e
        else:
            for k in range(horizon_steps):
                u_del = float(u0) if k <= 0 else float(u1)
                x = alpha * x + beta * (K_proc * (u_del - u0_pct))
                y_pred = y0 + x
                e = r - y_pred
                cost += e * e

        return float(cost + move_cost), float(y0 + x)

    def get_estimates(self):
        return (float(self._y_hat_last), float(self._y_pred_end_last))

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        prev_output = float(self._prev_output)
        out_min = self.out_min
        out_max = self.out_max
        n_delay = self._n_delay
        rate_limit = self.rate_limit
        du_max_pct = self.du_max_pct
        observer_gain = self.observer_gain
        K_proc = self.K_proc
        u0_pct = self.u0_pct
        u_head = int(self._u_head)
        u_hist = self._u_hist

        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return prev_output

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return prev_output

        y0 = self._ensure_y0(y)
        alpha = self._alpha_nom
        beta = self._beta_nom

        # Input that affects the plant output at this step (respect dead-time queue convention).
        if n_delay > 0:
            u_del_now = float(u_hist[u_head])
        else:
            u_del_now = prev_output

        # One-step model prediction (exact ZOH first-order discretization):
        #   x <- alpha*x + beta*K*(u_del-u0)
        #   y_hat = y0 + x
        self._x_hat = alpha * self._x_hat + beta * (K_proc * (u_del_now - u0_pct))
        y_hat = y0 + self._x_hat
        innovation = y - y_hat
        self._x_hat += observer_gain * innovation
        y_hat = y0 + self._x_hat
        self._y_hat_last = float(y_hat)

        # Candidate grid around previous output, bounded by du_max and output bounds
        u_center = prev_output
        du_limit = float(du_max_pct)
        if rate_limit is not None:
            du_limit = min(du_limit, max(0.0, float(rate_limit) * float(dt_s)))
        lo = max(out_min, u_center - du_limit)
        hi = min(out_max, u_center + du_limit)

        pts0 = self._build_candidates(lo, hi, u_center)

        best_u = float(_clamp(u_center, lo, hi))
        best_cost = None
        best_y_end = float(y_hat)
        for u0 in pts0:
            lo1 = max(out_min, float(u0) - du_limit)
            hi1 = min(out_max, float(u0) + du_limit)
            pts1 = self._build_candidates(lo1, hi1, float(u0))
            for u1 in pts1:
                c, y_end = self._simulate_cost(r, y0, self._x_hat, float(u0), float(u1))
                if (best_cost is None) or (c < best_cost):
                    best_cost = c
                    best_u = float(u0)
                    best_y_end = float(y_end)
        self._y_pred_end_last = float(best_y_end)

        u_out = float(_clamp(best_u, out_min, out_max))

        # Optional additional slew-rate limit (hardware protection)
        if rate_limit is not None:
            max_change = rate_limit * dt_s
            delta = u_out - prev_output
            if delta > max_change:
                u_out = prev_output + max_change
            elif delta < -max_change:
                u_out = prev_output - max_change
            u_out = float(_clamp(u_out, out_min, out_max))

        # Update model input history (for dead-time)
        if n_delay > 0:
            u_hist[u_head] = float(u_out)
            u_head += 1
            if u_head >= n_delay:
                u_head = 0
            self._u_head = u_head

        self._prev_output = u_out
        return float(u_out)

class TwoPositionPercent(ControllerBase):
    """Two-position (ON/OFF) regulator with hysteresis.

    This is a classical thermostat controller:

        If y <= r - hyst:  output = ON
        If y >= r + hyst:  output = OFF
        Otherwise: keep previous state

    Parameters
    ----------
    hyst_c:
        Hysteresis band in °C (half-band). Typical values: 0.5..2.0 °C.

    on_percent:
        Output in percent when ON. For a true 2-position regulator this is 100%.
        You can set a lower value (e.g., 60-80%) if full power overshoots too fast.

    min_switch_s:
        Minimum time (seconds) between state changes (anti-chatter).
        This is recommended when sensor noise is high.

    out_min, out_max:
        Output saturation limits (percent). Defaults to 0..100.

    Notes for students
    ------------------
    - Smaller hysteresis -> tighter regulation but more switching.
    - Larger hysteresis -> less switching but more temperature oscillation.
    """

    __slots__ = (
        "hyst_c", "out_min", "out_max", "on_percent", "min_switch_s",
        "_is_on", "_since_switch_s",
    )

    def __init__(
        self,
        hyst_c: float = 1.0,
        on_percent: float = 100.0,
        min_switch_s: float = 0.0,
        out_min: float = 0.0,
        out_max: float = 100.0,
    ):
        self.hyst_c = abs(float(hyst_c))
        if self.hyst_c <= 0.0:
            raise ValueError("hyst_c must be > 0")

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.on_percent = float(_clamp(float(on_percent), self.out_min, self.out_max))
        self.min_switch_s = max(0.0, float(min_switch_s))

        self.reset()

    def reset(self):
        """Reset internal state to OFF and allow immediate switching."""
        self._is_on = False
        self._since_switch_s = _LARGE_TIME

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        """Compute ON/OFF output in percent."""
        dt_s = float(dt_s)
        if (dt_s > 0.0) and _isfinite(dt_s):
            self._since_switch_s += dt_s

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return self.on_percent if self._is_on else self.out_min

        lo = r - self.hyst_c
        hi = r + self.hyst_c

        can_switch = self._since_switch_s >= self.min_switch_s

        if can_switch:
            if (not self._is_on) and (y <= lo):
                self._is_on = True
                self._since_switch_s = 0.0
            elif self._is_on and (y >= hi):
                self._is_on = False
                self._since_switch_s = 0.0

        return self.on_percent if self._is_on else self.out_min



class FuzzySugenoIncrementalPercent(ControllerBase):
    """Fuzzy logic controller (Sugeno singletons) with incremental output.

    Inputs
    ------
    - e  = r - y (°C)
    - de = d(e)/dt (°C/s)

    Both inputs are normalized to [-1, 1] using the provided scales.

    Fuzzification
    ------------
    Uses 5 linguistic sets for each input: NB, NS, Z, PS, PB.

    Inference / Defuzzification
    ---------------------------
    Product inference with a 5x5 editable rule table and Sugeno singleton
    consequents. The table entries are integer indices in {-2,-1,0,1,2} and
    are mapped internally to singleton outputs in {-1.0,-0.5,0.0,0.5,1.0}.

    Output
    ------
    The fuzzy output is interpreted as a *change rate* and integrated:
        u[k] = clamp(u[k-1] + du_rate_max * dt * du_norm, out_min..out_max)

    A separate optional rate_limit (percent/second) may also be applied for
    hardware protection, matching the PID controller interface.
    """

    __slots__ = (
        'e_scale_c', 'de_scale_c_per_s', 'du_rate_max', 'd_filter_alpha',
        'out_min', 'out_max', 'rate_limit',
        '_rules', '_prev_error', '_de_filt', '_prev_output',
    )

    def __init__(
        self,
        e_scale_c: float,
        de_scale_c_per_s: float,
        du_rate_max: float,
        rules_table,
        d_filter_alpha: float = 0.2,
        out_min: float = 0.0,
        out_max: float = 100.0,
        rate_limit=None,
    ):
        self.e_scale_c = float(e_scale_c)
        self.de_scale_c_per_s = float(de_scale_c_per_s)
        self.du_rate_max = float(du_rate_max)

        if (self.e_scale_c <= 0.0) or (not _isfinite(self.e_scale_c)):
            raise ValueError('e_scale_c must be > 0')
        if (self.de_scale_c_per_s <= 0.0) or (not _isfinite(self.de_scale_c_per_s)):
            raise ValueError('de_scale_c_per_s must be > 0')
        if (self.du_rate_max <= 0.0) or (not _isfinite(self.du_rate_max)):
            raise ValueError('du_rate_max must be > 0')

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.d_filter_alpha = float(_clamp(float(d_filter_alpha), 0.0, 1.0))

        self.rate_limit = _normalize_rate_limit(rate_limit)

        self._rules = self._parse_rules_table(rules_table)
        self.reset()

    @staticmethod
    def _parse_rules_table(rules_table):
        """Validate and flatten a 5x5 rule table.

        Expected format: 5 rows x 5 cols, entries in {-2,-1,0,1,2}.
        Returns a flat tuple of 25 singleton outputs in [-1, 1].
        """
        if rules_table is None:
            raise ValueError('rules_table is required')
        if len(rules_table) != 5:
            raise ValueError('rules_table must have 5 rows')
        flat = []
        for r in range(5):
            row = rules_table[r]
            if len(row) != 5:
                raise ValueError('rules_table row %d must have 5 columns' % r)
            for c in range(5):
                v = row[c]
                try:
                    vi = int(v)
                except Exception:
                    raise ValueError('rules_table[%d][%d] must be int' % (r, c))
                if vi < -2 or vi > 2:
                    raise ValueError('rules_table[%d][%d] must be in -2..2' % (r, c))
                flat.append(0.5 * float(vi))
        return tuple(flat)

    def reset(self):
        self._prev_error = None
        self._de_filt = 0.0
        self._prev_output = 0.0

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return float(self._prev_output)

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return float(self._prev_output)

        e = r - y
        if self._prev_error is None:
            de_raw = 0.0
        else:
            de_raw = (e - self._prev_error) / dt_s

        a = self.d_filter_alpha
        self._de_filt += a * (de_raw - self._de_filt)
        de = self._de_filt

        e_n = _clamp(e / self.e_scale_c, -1.0, 1.0)
        de_n = _clamp(de / self.de_scale_c_per_s, -1.0, 1.0)

        mu_e = _mf5(e_n)
        mu_de = _mf5(de_n)

        num = 0.0
        den = 0.0
        rules = self._rules
        k = 0
        for i in range(5):
            mi = mu_e[i]
            if mi <= 0.0:
                k += 5
                continue
            for j in range(5):
                w = mi * mu_de[j]
                if w > 0.0:
                    num += w * rules[k + j]
                    den += w
            k += 5

        du_norm = (num / den) if den > _EPSILON else 0.0

        u = self._prev_output + (self.du_rate_max * dt_s * du_norm)
        u = float(_clamp(u, self.out_min, self.out_max))

        if self.rate_limit is not None:
            max_change = self.rate_limit * dt_s
            delta = u - self._prev_output
            if delta > max_change:
                u = self._prev_output + max_change
            elif delta < -max_change:
                u = self._prev_output - max_change
            u = float(_clamp(u, self.out_min, self.out_max))

        self._prev_error = e
        self._prev_output = u
        return float(u)


# =============================================================================
# Model-based control (advanced): Smith Predictor PI
# =============================================================================

class SmithPredictorPI(ControllerBase):
    """Smith predictor PI controller (percent output) for a FOPDT plant.

    Plant model:
        G(s) = K / (tau*s + 1) * exp(-theta*s)

    The controller uses the model to predict a delay-free y and applies PI to
    that predicted y. A correction term (y_meas - y_model_with_delay) improves
    robustness to modeling errors.

    Notes for the lab
    -----------------
    - PI only (no derivative).
    - Requires active FOPDT model values (K, tau, theta).
    - Uses a fixed sample time for the dead-time buffer (dt_nominal_s).
    """

    __slots__ = (
        "kp", "ki",
        "K_proc", "tau_s", "theta_s", "u0_pct", "dt_nominal_s",
        "out_min", "out_max", "rate_limit", "integral_limit",
        "_alpha_nom", "_beta_nom",
        "_n_delay", "_integral", "_prev_output",
        "_x_delay", "_x_nodelay", "_pv0", "_u_hist", "_u_head",
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        K_proc: float,
        tau_s: float,
        theta_s: float,
        dt_nominal_s: float,
        u0_pct: float = 0.0,
        out_min: float = 0.0,
        out_max: float = 100.0,
        rate_limit=None,
        integral_limit=None,
    ):
        self.kp = float(kp)
        self.ki = float(ki)

        self.K_proc = float(K_proc)
        self.tau_s = float(tau_s)
        self.theta_s = max(0.0, float(theta_s))
        self.u0_pct = float(u0_pct)
        self.dt_nominal_s = max(_EPSILON, float(dt_nominal_s))
        self._alpha_nom = float(math.exp(-self.dt_nominal_s / max(self.tau_s, _EPSILON)))
        self._beta_nom = float(1.0 - self._alpha_nom)

        self.out_min, self.out_max = _normalize_output_bounds(out_min, out_max)

        self.rate_limit = _normalize_rate_limit(rate_limit)
        self.integral_limit = _normalize_integral_limit(integral_limit)

        # Use ceil to avoid underestimating dead-time when theta is between samples.
        self._n_delay = int(math.ceil(self.theta_s / self.dt_nominal_s))
        if self._n_delay < 0:
            self._n_delay = 0

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_output = 0.0

        self._x_delay = 0.0
        self._x_nodelay = 0.0
        self._pv0 = None

        if self._n_delay <= 0:
            self._u_hist = []
            self._u_head = 0
        else:
            hist = self._u_hist if hasattr(self, "_u_hist") else None
            if (hist is None) or (len(hist) != self._n_delay):
                if _array is not None:
                    self._u_hist = _array("f", [0.0] * self._n_delay)
                else:
                    self._u_hist = [0.0] * self._n_delay
            else:
                for i in range(self._n_delay):
                    self._u_hist[i] = 0.0
            self._u_head = 0

    def update(self, sp_c: float, y_c: float, dt_s: float) -> float:
        dt_s = float(dt_s)
        if (dt_s <= 0.0) or (not _isfinite(dt_s)):
            return float(self._prev_output)

        r = float(sp_c)
        y = float(y_c)
        if (not _isfinite(r)) or (not _isfinite(y)):
            return float(self._prev_output)

        if self._pv0 is None:
            self._pv0 = y

        if self._n_delay <= 0:
            u_delayed = float(self._prev_output)
        else:
            u_delayed = float(self._u_hist[self._u_head])

        # Use same exact ZOH first-order discretization as MPC for consistency:
        # x[k+1] = alpha*x[k] + (1-alpha)*K*(u[k]-u0), alpha = exp(-dt_nom/tau).
        alpha = self._alpha_nom
        beta = self._beta_nom
        self._x_delay = alpha * self._x_delay + beta * (self.K_proc * (u_delayed - self.u0_pct))
        self._x_nodelay = alpha * self._x_nodelay + beta * (self.K_proc * (float(self._prev_output) - self.u0_pct))

        y_model_delay = float(self._pv0 + self._x_delay)
        y_model_nodelay = float(self._pv0 + self._x_nodelay)

        y_pred = y_model_nodelay + (y - y_model_delay)

        e = r - y_pred
        p = self.kp * e

        i_candidate = self._integral + (self.ki * e * dt_s)
        if self.integral_limit is not None:
            i_candidate = _clamp(i_candidate, -self.integral_limit, self.integral_limit)

        u_unclamped = p + i_candidate
        saturating_high = u_unclamped > self.out_max
        saturating_low = u_unclamped < self.out_min

        allow_integrate = True
        if saturating_high and e > 0.0:
            allow_integrate = False
        if saturating_low and e < 0.0:
            allow_integrate = False
        if allow_integrate:
            self._integral = i_candidate

        u = p + self._integral
        u = float(_clamp(u, self.out_min, self.out_max))

        if self.rate_limit is not None:
            max_change = self.rate_limit * dt_s
            delta = u - self._prev_output
            if delta > max_change:
                u = self._prev_output + max_change
            elif delta < -max_change:
                u = self._prev_output - max_change
            u = float(_clamp(u, self.out_min, self.out_max))

        if self._n_delay > 0:
            self._u_hist[self._u_head] = float(u)
            self._u_head += 1
            if self._u_head >= self._n_delay:
                self._u_head = 0

        self._prev_output = u
        return float(u)
