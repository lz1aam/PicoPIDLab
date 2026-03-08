# indicator.py
"""
WS2812 indicator (single RGB LED).

Rule set:
- Very close to setpoint:
    - absolute tight band when `tight_band_abs_c` is set
    - otherwise +/-tight_band_percent of |SP|
  -> LIGHT GREEN, SOLID
- Otherwise:
    Color encodes sign of error:
        temp > setpoint  -> RED
        temp <= setpoint -> BLUE
    Regulation:
        within absolute band when `band_abs_c` is set, else +/-band_percent of |SP| -> SOLID
        outside band         -> BLINK

Call update(temp_c, setpoint_c) frequently (e.g., every 20-100 ms) for smooth blink.
"""

import time

try:
    from machine import Pin
except ImportError:  # pragma: no cover (non-MicroPython environments)
    Pin = None

try:
    import neopixel as _neopixel
except ImportError:  # pragma: no cover (boards/builds without neopixel module)
    _neopixel = None

_STATUS_RGB_PIN = 16
_STATUS_RGB_BRIGHTNESS = 0.01
_INDICATOR_NEAR_ZERO_TOL_C = 0.2
_INDICATOR_BAND_C = 1.0
_INDICATOR_TIGHT_BAND_C = 0.25


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class StatusRGB:
    def __init__(self,
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
                 green_level: int = 255):
        # Optional dependency and hardware:
        # - Some MicroPython builds omit the neopixel module.
        # - Some lab setups do not mount a WS2812.
        # In these cases we degrade to a no-op indicator rather than failing.
        self._enabled = (Pin is not None) and (_neopixel is not None) and (pin is not None)
        self._np = None
        if self._enabled:
            self._np = _neopixel.NeoPixel(Pin(int(pin), Pin.OUT), 1)

        self._brightness = _clamp(float(brightness), 0.0, 1.0)
        self._band = float(band_percent) / 100.0
        self._tight_band = float(tight_band_percent) / 100.0
        self._band_abs_c = None if (band_abs_c is None) else abs(float(band_abs_c))
        self._tight_band_abs_c = None if (tight_band_abs_c is None) else abs(float(tight_band_abs_c))
        nz_tol = float(near_zero_abs_tol_c)
        self._near_zero_abs_tol_c = nz_tol if nz_tol > 0.0 else 0.2
        self._blink_period_ms = int(blink_period_ms)

        self._red_level = int(_clamp(red_level, 0, 255))
        self._blue_level = int(_clamp(blue_level, 0, 255))
        self._green_level = int(_clamp(green_level, 0, 255))

        # Light green for tight band (tuned)
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

        # Near SP=0, a percentage tolerance collapses toward zero; use an
        # explicit absolute tolerance instead.
        if abs(r) < 1e-6:
            return abs(y - r) <= self._near_zero_abs_tol_c

        tol = abs(r) * float(band)
        return abs(y - r) <= tol

    def _within_abs_band(self, temp_c: float, setpoint_c: float, tol_abs_c) -> bool:
        if tol_abs_c is None:
            return False
        return abs(float(temp_c) - float(setpoint_c)) <= float(tol_abs_c)

    def update(self, temp_c: float, setpoint_c: float):
        y = float(temp_c)
        r = float(setpoint_c)

        # Tight band: light green solid
        if self._within_abs_band(y, r, self._tight_band_abs_c) or self._within_band(y, r, self._tight_band):
            self._write(self._tight_r, self._tight_g, self._tight_b)
            return

        reached = self._within_abs_band(y, r, self._band_abs_c) or self._within_band(y, r, self._band)

        # Color by sign of error: above->red, below/equal->blue
        if y > r:
            base_r, base_g, base_b = self._red_level, 0, 0
        else:
            base_r, base_g, base_b = 0, 0, self._blue_level

        if reached:
            self._write(base_r, base_g, base_b)
        else:
            phase = time.ticks_ms() % self._blink_period_ms
            on = phase < (self._blink_period_ms // 2)
            self._write(base_r if on else 0,
                        base_g if on else 0,
                        base_b if on else 0)

    def off(self):
        self._write(0, 0, 0)
