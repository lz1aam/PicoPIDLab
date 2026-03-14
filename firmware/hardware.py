# hardware.py
"""Board-specific hardware drivers for PicoPID Lab.

This module keeps all RP2040-facing I/O in one place:
- NTC thermistor temperature sensing
- PWM heater output
- WS2812 status indicator
"""

import math
import time

from machine import ADC, PWM, Pin

try:
    import machine  # for machine.mem32 on RP2
except ImportError:  # pragma: no cover (non-MicroPython environments)
    machine = None

try:
    import neopixel as _neopixel
except ImportError:  # pragma: no cover (boards/builds without neopixel module)
    _neopixel = None

try:
    from machine import idle as _machine_idle
except Exception:
    _machine_idle = None


_ADC_FULL_SCALE_U16 = 65535.0
_EPSILON = 1e-6
_ADC_TEMP_PIN = 26
_ADC_VREF_V = 3.3
_R_FIXED_OHM = 10000.0
_NTC_N_AVG = 10
_NTC_INTER_DELAY_MS = 2
_TICKS_MS = getattr(time, "ticks_ms", None)
_TICKS_ADD = getattr(time, "ticks_add", None)
_TICKS_DIFF = getattr(time, "ticks_diff", None)

_PWM_FREQ_MIN_HZ = 10
_PWM_FREQ_MAX_HZ = 1000
_PWM_HEATER_PIN = 6
_HEATER_PWM_FREQ_HZ = 100
_HEATER_PWM_DRIVE_MA = 12

_STATUS_RGB_PIN = 16
_STATUS_RGB_BRIGHTNESS = 0.01
_INDICATOR_NEAR_ZERO_TOL_C = 0.2
_INDICATOR_BAND_C = 1.0
_INDICATOR_TIGHT_BAND_C = 0.25
_SETPOINT_ZERO_EPS = 1e-6


def yield_cpu() -> None:
    if _machine_idle is not None:
        try:
            _machine_idle()
        except Exception:
            pass


def advance_deadline(next_ms: int, period_ms: int, now_ms: int, max_lag_periods: int = 3) -> int:
    """Advance a periodic deadline without accumulating an unbounded backlog."""
    lag_ms = time.ticks_diff(now_ms, next_ms)
    max_lag_ms = int(max(1, max_lag_periods) * int(period_ms))
    if lag_ms > max_lag_ms:
        return time.ticks_add(now_ms, int(period_ms))
    return time.ticks_add(next_ms, int(period_ms))


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _wait_ms_cooperative(delay_ms: int) -> None:
    d = int(delay_ms)
    if d <= 0:
        return
    if callable(_TICKS_MS) and callable(_TICKS_ADD) and callable(_TICKS_DIFF):
        deadline = _TICKS_ADD(_TICKS_MS(), d)
        while _TICKS_DIFF(_TICKS_MS(), deadline) < 0:
            yield_cpu()
        return
    end = time.time() + (d / 1000.0)
    while time.time() < end:
        yield_cpu()


def _avg_u16(adc: ADC, n_avg: int, delay_ms: int) -> int:
    """Average multiple ADC samples (u16) with an optional inter-sample delay."""
    n = max(1, int(n_avg))
    dly = max(0, int(delay_ms))
    s = 0
    for _ in range(n):
        s += adc.read_u16()
        if dly:
            _wait_ms_cooperative(dly)
    return int(s / n)


class TemperatureSensor:
    """Interface/base class for temperature sensors."""

    def read_c(self) -> float:
        raise NotImplementedError


class NTCLG100E2103JB(TemperatureSensor):
    """Vishay NTCLG100E2103JB (10k NTC) using Vishay inverse polynomial."""

    R25_OHM = 10000.0
    A1 = 0.00335401643468053
    B1 = 0.00025698501802
    C1 = 0.0000026201306709
    D1 = 0.000000063830907998

    def __init__(
        self,
        adc_pin: int = _ADC_TEMP_PIN,
        r_fixed_ohm: float = _R_FIXED_OHM,
        n_avg: int = _NTC_N_AVG,
        inter_delay_ms: int = _NTC_INTER_DELAY_MS,
    ):
        self._adc = ADC(adc_pin)
        self._r_fixed = float(r_fixed_ohm)
        self._n_avg = int(n_avg)
        self._dly = int(inter_delay_ms)
        self._v_eps = _EPSILON
        self._r_eps = _EPSILON

    def _read_voltage_v(self) -> float:
        raw = _avg_u16(self._adc, self._n_avg, self._dly)
        v = (raw * _ADC_VREF_V) / _ADC_FULL_SCALE_U16
        return max(self._v_eps, min(_ADC_VREF_V - self._v_eps, v))

    def _read_resistance_ohm(self) -> float:
        v = self._read_voltage_v()
        r_ntc = self._r_fixed * v / (_ADC_VREF_V - v)
        return max(self._r_eps, r_ntc)

    @classmethod
    def _temp_c_from_resistance_ohm(cls, r_ohm: float) -> float:
        r = float(r_ohm)
        if r <= 0.0:
            return float("nan")
        x = math.log(r / cls.R25_OHM)
        inv_t = cls.A1 + (cls.B1 * x) + (cls.C1 * x * x) + (cls.D1 * x * x * x)
        if inv_t <= 0.0:
            return float("nan")
        return (1.0 / inv_t) - 273.15

    def read_c(self) -> float:
        return self._temp_c_from_resistance_ohm(self._read_resistance_ohm())


_PADS_BANK0_BASE = 0x4001C000
_PADS_BANK0_GPIO0_OFFSET = 0x04
_DRIVE_SHIFT = 4
_DRIVE_MASK = 0b11 << _DRIVE_SHIFT

_DRIVE_MA_TO_CODE = {
    2: 0,
    4: 1,
    8: 2,
    12: 3,
}


def _pads_bank0_gpio_reg_addr(gpio: int) -> int:
    return _PADS_BANK0_BASE + _PADS_BANK0_GPIO0_OFFSET + (int(gpio) * 4)


def set_rp2040_gpio_drive_strength(gpio: int, drive_ma: int) -> bool:
    if machine is None or not hasattr(machine, "mem32"):
        return False
    gpio_i = int(gpio)
    if gpio_i < 0 or gpio_i > 29:
        return False
    code = _DRIVE_MA_TO_CODE.get(int(drive_ma))
    if code is None:
        return False
    addr = _pads_bank0_gpio_reg_addr(gpio_i)
    try:
        reg = int(machine.mem32[addr])
    except Exception:
        return False
    reg = (reg & ~_DRIVE_MASK) | ((code & 0b11) << _DRIVE_SHIFT)
    try:
        machine.mem32[addr] = reg
    except Exception:
        return False
    return True


class Heater:
    """PWM-driven heater output."""

    __slots__ = ("_pin", "_pwm", "_percent")

    def __init__(
        self,
        pwm_pin: int = _PWM_HEATER_PIN,
        pwm_freq_hz: int = _HEATER_PWM_FREQ_HZ,
        drive_ma: int = None,
    ):
        self._pin = Pin(int(pwm_pin), Pin.OUT, value=0)
        if drive_ma is None:
            drive_ma = _HEATER_PWM_DRIVE_MA
        set_rp2040_gpio_drive_strength(int(pwm_pin), int(drive_ma))
        self._pwm = PWM(self._pin)
        f = int(pwm_freq_hz)
        if f < _PWM_FREQ_MIN_HZ or f > _PWM_FREQ_MAX_HZ:
            raise ValueError(
                "pwm_freq_hz must be in %d..%d Hz (got %d)"
                % (_PWM_FREQ_MIN_HZ, _PWM_FREQ_MAX_HZ, f)
            )
        self._pwm.freq(f)
        self._percent = 0.0
        self.off()

    def set_percent(self, percent: float):
        if not _isfinite(percent):
            p = 0.0
        else:
            p = _clamp(float(percent), 0.0, 100.0)
        self._pwm.duty_u16(int(p * 65535.0 / 100.0))
        self._percent = float(p)

    def off(self):
        self._pwm.duty_u16(0)
        self._percent = 0.0

    def current_percent(self) -> float:
        return float(self._percent)


class StatusRGB:
    def __init__(
        self,
        pin: int = _STATUS_RGB_PIN,
        brightness: float = _STATUS_RGB_BRIGHTNESS,
        band_percent: float = 5.0,
        tight_band_percent: float = 0.5,
        band_abs_c: float = _INDICATOR_BAND_C,
        tight_band_abs_c: float = _INDICATOR_TIGHT_BAND_C,
        near_zero_abs_tol_c: float = _INDICATOR_NEAR_ZERO_TOL_C,
        blink_period_ms: int = 800,
        red_level: int = 255,
        blue_level: int = 255,
        green_level: int = 255,
    ):
        self._enabled = (_neopixel is not None) and (pin is not None)
        self._np = None
        if self._enabled:
            self._np = _neopixel.NeoPixel(Pin(int(pin), Pin.OUT), 1)

        self._brightness = _clamp(float(brightness), 0.0, 1.0)
        self._band = float(band_percent) / 100.0
        self._tight_band = float(tight_band_percent) / 100.0
        self._band_abs_c = None if (band_abs_c is None) else abs(float(band_abs_c))
        self._tight_band_abs_c = None if (tight_band_abs_c is None) else abs(float(tight_band_abs_c))
        nz_tol = float(near_zero_abs_tol_c)
        self._near_zero_abs_tol_c = nz_tol if nz_tol > 0.0 else _INDICATOR_NEAR_ZERO_TOL_C
        self._blink_period_ms = max(500, int(blink_period_ms))

        self._red_level = int(_clamp(red_level, 0, 255))
        self._blue_level = int(_clamp(blue_level, 0, 255))
        self._green_level = int(_clamp(green_level, 0, 255))

        self._tight_r = 0
        self._tight_g = int(self._green_level)
        self._tight_b = int(self._blue_level * 0.2)
        self._last_rgb = (-1, -1, -1)

    def _write(self, r: int, g: int, b: int):
        if not self._enabled:
            return
        r = int(_clamp(r * self._brightness, 0, 255))
        g = int(_clamp(g * self._brightness, 0, 255))
        b = int(_clamp(b * self._brightness, 0, 255))
        if (r, g, b) == self._last_rgb:
            return
        self._np[0] = (r, g, b)
        self._np.write()
        self._last_rgb = (r, g, b)

    def _within_band(self, temp_c: float, setpoint_c: float, band: float) -> bool:
        r = float(setpoint_c)
        y = float(temp_c)
        if abs(r) < _SETPOINT_ZERO_EPS:
            return abs(y - r) <= self._near_zero_abs_tol_c
        return abs(y - r) <= (abs(r) * float(band))

    def _within_abs_band(self, temp_c: float, setpoint_c: float, tol_abs_c) -> bool:
        if tol_abs_c is None:
            return False
        return abs(float(temp_c) - float(setpoint_c)) <= float(tol_abs_c)

    def update(self, temp_c: float, setpoint_c: float):
        y = float(temp_c)
        r = float(setpoint_c)
        if self._within_abs_band(y, r, self._tight_band_abs_c) or self._within_band(y, r, self._tight_band):
            self._write(self._tight_r, self._tight_g, self._tight_b)
            return

        reached = self._within_abs_band(y, r, self._band_abs_c) or self._within_band(y, r, self._band)
        if y > r:
            base_r, base_g, base_b = self._red_level, 0, 0
        else:
            base_r, base_g, base_b = 0, 0, self._blue_level

        if reached:
            self._write(base_r, base_g, base_b)
        else:
            phase = time.ticks_ms() % self._blink_period_ms
            on = phase < (self._blink_period_ms // 2)
            self._write(base_r if on else 0, base_g if on else 0, base_b if on else 0)

    def off(self):
        self._write(0, 0, 0)
