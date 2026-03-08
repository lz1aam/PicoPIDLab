# actuator.py
"""
Actuator abstraction (heater).

This module intentionally keeps the actuator interface small and explicit, to
support control-lab exercises (manual steps, clear labeling, predictable I/O).

Contract
--------
- set_percent(0..100)
- off()

Notes for RP2040 GPIO drive strength
-----------------------------------
On RP2040, each GPIO pad has a configurable output *drive strength* setting
(2/4/8/12 mA). This setting affects the output driver impedance and thus how
well the pin can hold a logic level when sourcing/sinking current. It is **not**
a current limiter.

If you are driving a BJT base (through a resistor) with PWM and need a bit more
headroom, you can raise the pad drive strength to 12 mA.

Implementation detail: as of Feb 2026, the official MicroPython rp2 port still
does not consistently expose a stable high-level API for pad drive strength on
all builds, so this module applies the setting by writing the RP2040 pad control
register (PADS_BANK0). If a future MicroPython build exposes Pin(..., drive=...)
for rp2, this direct register write will still work and remains explicit.
"""

from machine import PWM, Pin
import math

try:
    import machine  # for machine.mem32 on RP2
except ImportError:  # pragma: no cover (non-MicroPython environments)
    machine = None


# ---------------------------------------------------------------------------
# RP2040 pad-control helpers (drive strength)
# ---------------------------------------------------------------------------

# RP2040: PADS_BANK0 pad control registers start at 0x4001C000.
# GPIO0 register is at offset 0x04, then +4 bytes per GPIO number.
_PADS_BANK0_BASE = 0x4001C000
_PADS_BANK0_GPIO0_OFFSET = 0x04

# PADS_BANK0 GPIOx register:
# bits 5:4 DRIVE = {0:2mA, 1:4mA, 2:8mA, 3:12mA}
_DRIVE_SHIFT = 4
_DRIVE_MASK = 0b11 << _DRIVE_SHIFT

_DRIVE_MA_TO_CODE = {
    2: 0,
    4: 1,
    8: 2,
    12: 3,
}

_PWM_FREQ_MIN_HZ = 10
_PWM_FREQ_MAX_HZ = 1000
_PWM_HEATER_PIN = 6
_HEATER_PWM_FREQ_HZ = 100
_HEATER_PWM_DRIVE_MA = 12


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _pads_bank0_gpio_reg_addr(gpio: int) -> int:
    """Return the absolute address of PADS_BANK0 GPIOx pad-control register."""
    return _PADS_BANK0_BASE + _PADS_BANK0_GPIO0_OFFSET + (int(gpio) * 4)


def set_rp2040_gpio_drive_strength(gpio: int, drive_ma: int) -> bool:
    """Set RP2040 GPIO pad drive strength.

    Parameters
    ----------
    gpio:
        GPIO number (0..29) in the user bank.
    drive_ma:
        One of {2, 4, 8, 12}. Values outside this set are rejected.

    Returns
    -------
    bool
        True if the setting was applied (RP2 platform + valid inputs),
        False if skipped (wrong platform) or rejected (invalid inputs).

    Safety / Control-lab note
    -------------------------
    This changes pad characteristics only; it does not guarantee a particular
    source/sink current into an arbitrary load.
    """
    # Avoid fragile platform-string checks. If mem32 is not present/usable,
    # the function will fail safely and return False.
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

    # Replace DRIVE field while preserving all other pad settings (pulls, schmitt, slew).
    reg = (reg & ~_DRIVE_MASK) | ((code & 0b11) << _DRIVE_SHIFT)
    try:
        machine.mem32[addr] = reg
    except Exception:
        return False
    return True


# ---------------------------------------------------------------------------
# Actuators
# ---------------------------------------------------------------------------

class Heater:
    """PWM-driven heater output.

    Parameters
    ----------
    pwm_pin:
        GPIO used for PWM output.
    pwm_freq_hz:
        PWM frequency for the heater drive.
    drive_ma:
        Requested GPIO drive strength in mA (2/4/8/12) on RP2040.
        Defaults to local `_HEATER_PWM_DRIVE_MA` (12 mA).
    """

    __slots__ = ("_pin", "_pwm", "_percent")

    def __init__(
        self,
        pwm_pin: int = _PWM_HEATER_PIN,
        pwm_freq_hz: int = _HEATER_PWM_FREQ_HZ,
        drive_ma: int = None,
    ):
        # Configure the GPIO first (safe default low), then attach PWM.
        self._pin = Pin(int(pwm_pin), Pin.OUT, value=0)

        if drive_ma is None:
            drive_ma = _HEATER_PWM_DRIVE_MA

        # Best-effort: apply pad drive strength on RP2040.
        # If running on a different port, this is a no-op.
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
