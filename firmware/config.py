# config.py
"""Student-editable experiment knobs. See docs/en/CONFIG_TUTORIAL.md for usage guidance."""

# =============================================================================
# Core Runtime
# =============================================================================
TS_S = 0.25                 # Sample period [s]
SETPOINT_C = 50.0           # Target temperature [degC]
EXPERIMENT_RUN_S = 200.0    # Auto-stop time [s] or None
TELEMETRY_MODE = "NORMAL"   # INFO / NORMAL / MPC
SETPOINT_TYPE = "STEP"      # STEP or RAMP
SETPOINT_RAMP_RATE = 5.0    # Ramp rate [degC/s]
CONTROL_MODE = "PID"        # ONOFF / PID / FUZZY / MPC

# Disturbance injection (control runs only)
DIST_ENABLE = False         # Enable additive OP disturbance
DIST_MODE = "STEP"          # STEP or PULSE
DIST_MAG_PCT = -25.0        # Disturbance magnitude [%]
DIST_START_S = 60.0         # Disturbance start time [s]
DIST_DURATION_S = 30.0      # Pulse duration [s]


# =============================================================================
# Safety
# =============================================================================
TEMP_CUTOFF_C = 85.0        # Hard cutoff [degC]


# =============================================================================
# PID Mode
# =============================================================================
PID_VARIANT = "PID"         # PID / 2DOF / FF_PID / GAIN_SCHED / SMITH_PI
PID_AW_TYPE = "CLAMP"       # NONE / CLAMP / BACKCALC
PID_ALGORITHM = "PARALLEL"  # PARALLEL / IDEAL / SERIES

# Parallel gains (PID_ALGORITHM == "PARALLEL")
KP = 43.671               # Proportional gain
KI = 8.47154              # Integral gain [1/s]
KD = 56.2813              # Derivative gain [s]

# Ideal/series gains (PID_ALGORITHM == "IDEAL" or "SERIES")
KC = 43.671               # Ideal/series gain
TI_S = 5.15503            # Integral time [s]
TD_S = 1.28876            # Derivative time [s]
SPAN = 60.0               # Engineering span [degC]

# Shared PID shaping
DERIVATIVE_FILTER_ALPHA = 0.2  # Derivative LPF alpha [0..1]
PID_INTEGRAL_LIMIT = 100       # Integral clamp [%] or None

# Variant-specific knobs
PID_AW_TRACKING_TIME_S = 5.0   # BACKCALC tracking time [s]
PID_BETA = 0.6                 # 2DOF setpoint weight [0..1]
FF_MODE = "FOPDT_GAIN"         # MANUAL or FOPDT_GAIN
FF_GAIN_PCT_PER_C = 1.5        # Manual FF gain [%/degC]
FF_BIAS_PCT = 0.0              # Manual FF bias [%]
FF_AMBIENT_C = 25              # Ambient reference [degC] or None
GS_VARIABLE = "PV"             # Schedule on PV or SP
GS_TABLE = (                   # (temp_c, kp, ki, kd)
    (25.0, 6.0, 0.40, 0.05),
    (50.0, 5.0, 0.50, 0.10),
    (75.0, 4.0, 0.60, 0.15),
)

# Tuning
TUNING_METHOD = "CC_PID"        # Model: ZN1_*/CC_*  Relay: ZN2_*/TL_*

# Relay tuning
TUNING_TARGET_C = 50.0        # Relay target [degC]
TUNING_BAND_C = 1.0           # Relay half-band [degC]
TUNING_CYCLES = 5             # Relay cycles to average


# =============================================================================
# ON/OFF Mode
# =============================================================================
ONOFF_HYST_C = 1.0           # Hysteresis half-band [degC]
ONOFF_ON_PERCENT = 100.0     # ON output [%]
ONOFF_MIN_SWITCH_S = 1.0     # Minimum dwell [s]


# =============================================================================
# FUZZY Mode
# =============================================================================
FUZZY_E_SCALE_C = 8.0           # Error scale [degC]
FUZZY_DE_SCALE_C_PER_S = 1.0    # dError scale [degC/s]
FUZZY_DU_RATE_MAX = 20.0        # Output slew cap [%/s]
FUZZY_DE_FILTER_ALPHA = 0.6     # dError filter alpha [0..1]
FUZZY_RULE_TABLE = (
    (-2, -2, -2, -1, 0),
    (-2, -2, -1, 0, 1),
    (-2, -1, 0, 1, 2),
    (-1, 0, 1, 2, 2),
    (0, 1, 2, 2, 2),
)


# =============================================================================
# MPC Mode
# =============================================================================
MPC_HORIZON_STEPS = 24        # Prediction horizon
MPC_GRID_STEP_PCT = 5.0       # Candidate grid step [%]
MPC_DU_MAX_PCT = 15.0         # Max move per step [%]
MPC_LAMBDA_MOVE = 0.02        # Move suppression weight
MPC_MAX_CANDIDATES = 11       # Search breadth
MPC_OBSERVER_GAIN = 0.5       # Observer gain [0..1]


# =============================================================================
# FOPDT Model
# =============================================================================
FOPDT_U1_PERCENT = 100.0       # Step output [%]
FOPDT_SMOOTH_N = 10            # Smoothing window
STEADY_WINDOW_S = 10.0         # Steady-state window [s]
STEADY_BAND_C = 0.5            # Steady-state band [degC]

# Runtime defaults when no model is loaded in RAM
MODEL_K = 0.587880            # Process gain [degC/%]
MODEL_TAU_S = 55.14           # Time constant [s]
MODEL_THETA_S = 2.58          # Dead time [s]
MODEL_U0_PCT = 0.0            # Operating output [%]
MODEL_Y0 = 24.652             # Operating PV [degC]
MODEL_Y1 = 83.440             # Optional final PV [degC]
MODEL_RMSE = 0.621            # Optional fit RMSE [degC]
MODEL_METHOD = "SK"           # Optional method label


# =============================================================================
# Validation
# =============================================================================
_ALLOWED_CONTROL_MODES = ("PID", "ONOFF", "FUZZY", "MPC")
_ALLOWED_SETPOINT_TYPES = ("STEP", "RAMP")
_ALLOWED_DIST_MODES = ("STEP", "PULSE")
_ALLOWED_TUNING_METHODS = (
    "ZN1_P", "ZN1_PI", "ZN1_PID",
    "CC_P", "CC_PI", "CC_PID",
    "ZN2_P", "ZN2_PI", "ZN2_PID",
    "TL_P", "TL_PI", "TL_PID",
)
MODEL_TUNING_METHODS = (
    "ZN1_P", "ZN1_PI", "ZN1_PID",
    "CC_P", "CC_PI", "CC_PID",
)
RELAY_TUNING_METHODS = (
    "ZN2_P", "ZN2_PI", "ZN2_PID",
    "TL_P", "TL_PI", "TL_PID",
)
_ALLOWED_PID_VARIANTS = ("PID", "2DOF", "FF_PID", "GAIN_SCHED", "SMITH_PI")
_ALLOWED_PID_AW_TYPES = ("NONE", "CLAMP", "BACKCALC")
_ALLOWED_PID_ALGORITHM = ("IDEAL", "PARALLEL", "SERIES")
_ALLOWED_TELEMETRY_MODES = ("INFO", "NORMAL", "MPC")
_PID_ZERO_EPS = 1e-12


def _validate_core() -> None:
    if CONTROL_MODE not in _ALLOWED_CONTROL_MODES:
        raise ValueError("CONTROL_MODE must be one of: %s" % (_ALLOWED_CONTROL_MODES,))
    if SETPOINT_TYPE not in _ALLOWED_SETPOINT_TYPES:
        raise ValueError("SETPOINT_TYPE must be 'STEP' or 'RAMP'")
    if float(TS_S) <= 0.0:
        raise ValueError("TS_S must be > 0")
    if float(STEADY_WINDOW_S) <= 0.0:
        raise ValueError("STEADY_WINDOW_S must be > 0")
    if float(STEADY_BAND_C) <= 0.0:
        raise ValueError("STEADY_BAND_C must be > 0")
    tm = str(TELEMETRY_MODE).upper()
    if tm not in _ALLOWED_TELEMETRY_MODES:
        raise ValueError("TELEMETRY_MODE must be one of: %s" % (_ALLOWED_TELEMETRY_MODES,))
    dm = str(DIST_MODE).upper()
    if dm not in _ALLOWED_DIST_MODES:
        raise ValueError("DIST_MODE must be one of: %s" % (_ALLOWED_DIST_MODES,))
    if float(DIST_START_S) < 0.0:
        raise ValueError("DIST_START_S must be >= 0")
    if (dm == "PULSE") and (float(DIST_DURATION_S) <= 0.0):
        raise ValueError("DIST_DURATION_S must be > 0 when DIST_MODE='PULSE'")
    if (str(SETPOINT_TYPE).upper() == "RAMP") and (float(SETPOINT_RAMP_RATE) <= 0.0):
        raise ValueError("SETPOINT_RAMP_RATE must be > 0 when using RAMP")
    exp_s = EXPERIMENT_RUN_S
    if (exp_s is not None) and (float(exp_s) <= 0.0):
        raise ValueError("EXPERIMENT_RUN_S must be > 0 (or set to None to disable)")


def _validate_safety() -> None:
    if (float(TEMP_CUTOFF_C) <= -50.0) or (float(TEMP_CUTOFF_C) >= 140.0):
        raise ValueError("TEMP_CUTOFF_C looks unrealistic; check units (°C)")


def _validate_onoff() -> None:
    if str(CONTROL_MODE).upper() != "ONOFF":
        return
    if float(ONOFF_HYST_C) <= 0.0:
        raise ValueError("ONOFF_HYST_C must be > 0")
    if (float(ONOFF_ON_PERCENT) < 0.0) or (float(ONOFF_ON_PERCENT) > 100.0):
        raise ValueError("ONOFF_ON_PERCENT must be in 0..100")
    if float(ONOFF_MIN_SWITCH_S) < 0.0:
        raise ValueError("ONOFF_MIN_SWITCH_S must be >= 0")


def _validate_pid_variant(variant: str, aw_type: str, algorithm: str) -> None:
    if (variant == "PID") and (aw_type == "BACKCALC") and (float(PID_AW_TRACKING_TIME_S) <= 0.0):
        raise ValueError("PID_AW_TRACKING_TIME_S must be > 0")
    if variant == "2DOF":
        beta = float(PID_BETA)
        if (beta < 0.0) or (beta > 1.0):
            raise ValueError("PID_BETA must be in 0..1")
        if algorithm != "PARALLEL":
            raise ValueError("PID_VARIANT='2DOF' requires PID_ALGORITHM='PARALLEL' (current: %s)" % algorithm)
    if variant == "FF_PID":
        ff_mode = str(FF_MODE).upper()
        if ff_mode not in ("MANUAL", "FOPDT_GAIN"):
            raise ValueError("FF_MODE must be 'MANUAL' or 'FOPDT_GAIN'")
        if (FF_AMBIENT_C is not None) and ((float(FF_AMBIENT_C) <= -50.0) or (float(FF_AMBIENT_C) >= 200.0)):
            raise ValueError("FF_AMBIENT_C looks unrealistic; check units (°C)")
        if algorithm != "PARALLEL":
            raise ValueError("PID_VARIANT='FF_PID' requires PID_ALGORITHM='PARALLEL' (current: %s)" % algorithm)
    if variant == "GAIN_SCHED":
        if str(GS_VARIABLE).upper() not in ("PV", "SP"):
            raise ValueError("GS_VARIABLE must be 'PV' or 'SP'")
        if (GS_TABLE is None) or (len(GS_TABLE) < 2):
            raise ValueError("GS_TABLE must have at least 2 breakpoints")
        for i, row in enumerate(GS_TABLE):
            if len(row) != 4:
                raise ValueError("GS_TABLE[%d] must be (temp,kp,ki,kd)" % i)


def _validate_pid_tuning() -> None:
    rule = str(TUNING_METHOD).upper()
    if rule not in _ALLOWED_TUNING_METHODS:
        raise ValueError("TUNING_METHOD must be one of: %s" % (_ALLOWED_TUNING_METHODS,))
    target_c = float(TUNING_TARGET_C)
    if (target_c <= -50.0) or (target_c >= 200.0):
        raise ValueError("TUNING_TARGET_C looks unrealistic; check units (°C)")
    if rule in RELAY_TUNING_METHODS:
        if float(TUNING_BAND_C) <= 0.0:
            raise ValueError("TUNING_BAND_C must be > 0")
        if int(TUNING_CYCLES) < 1:
            raise ValueError("TUNING_CYCLES must be >= 1")


def _validate_pid() -> None:
    if str(CONTROL_MODE).upper() != "PID":
        return
    _validate_pid_tuning()
    variant = str(PID_VARIANT).upper()
    aw_type = str(PID_AW_TYPE).upper()
    algorithm = str(PID_ALGORITHM).upper()
    if variant not in _ALLOWED_PID_VARIANTS:
        raise ValueError("PID_VARIANT must be one of: %s" % (_ALLOWED_PID_VARIANTS,))
    if aw_type not in _ALLOWED_PID_AW_TYPES:
        raise ValueError("PID_AW_TYPE must be one of: %s" % (_ALLOWED_PID_AW_TYPES,))
    if algorithm not in _ALLOWED_PID_ALGORITHM:
        raise ValueError("PID_ALGORITHM must be one of: %s" % (_ALLOWED_PID_ALGORITHM,))
    if (PID_INTEGRAL_LIMIT is not None) and (float(PID_INTEGRAL_LIMIT) <= 0.0):
        raise ValueError("PID_INTEGRAL_LIMIT must be > 0, or None")
    if float(SPAN) <= 0.0:
        raise ValueError("SPAN must be > 0")
    if algorithm in ("IDEAL", "SERIES"):
        ti = float(TI_S)
        td = float(TD_S)
        if ti < 0.0:
            raise ValueError("TI_S must be >= 0 for PID_ALGORITHM='IDEAL'/'SERIES'")
        if td < 0.0:
            raise ValueError("TD_S must be >= 0 for PID_ALGORITHM='IDEAL'/'SERIES'")
        if (algorithm == "SERIES") and (abs(float(KC)) <= _PID_ZERO_EPS) and ((ti > 0.0) or (td > 0.0)):
            raise ValueError("PID_ALGORITHM='SERIES' requires KC != 0 when TI_S or TD_S is active")
    _validate_pid_variant(variant, aw_type, algorithm)


def _validate_fuzzy() -> None:
    if str(CONTROL_MODE).upper() != "FUZZY":
        return
    if float(FUZZY_E_SCALE_C) <= 0.0:
        raise ValueError("FUZZY_E_SCALE_C must be > 0")
    if float(FUZZY_DE_SCALE_C_PER_S) <= 0.0:
        raise ValueError("FUZZY_DE_SCALE_C_PER_S must be > 0")
    if float(FUZZY_DU_RATE_MAX) <= 0.0:
        raise ValueError("FUZZY_DU_RATE_MAX must be > 0")
    alpha = float(FUZZY_DE_FILTER_ALPHA)
    if (alpha < 0.0) or (alpha > 1.0):
        raise ValueError("FUZZY_DE_FILTER_ALPHA must be in 0..1")
    if (FUZZY_RULE_TABLE is None) or (len(FUZZY_RULE_TABLE) != 5):
        raise ValueError("FUZZY_RULE_TABLE must have 5 rows")
    for r in range(5):
        row = FUZZY_RULE_TABLE[r]
        if len(row) != 5:
            raise ValueError("FUZZY_RULE_TABLE row %d must have 5 columns" % r)
        for c in range(5):
            try:
                v = int(row[c])
            except Exception:
                raise ValueError("FUZZY_RULE_TABLE[%d][%d] must be int" % (r, c))
            if (v < -2) or (v > 2):
                raise ValueError("FUZZY_RULE_TABLE[%d][%d] must be in -2..2" % (r, c))


def _validate_mpc() -> None:
    if str(CONTROL_MODE).upper() != "MPC":
        return
    if int(MPC_HORIZON_STEPS) < 3:
        raise ValueError("MPC_HORIZON_STEPS must be >= 3")
    if float(MPC_GRID_STEP_PCT) <= 0.0:
        raise ValueError("MPC_GRID_STEP_PCT must be > 0")
    if float(MPC_DU_MAX_PCT) <= 0.0:
        raise ValueError("MPC_DU_MAX_PCT must be > 0")
    if float(MPC_LAMBDA_MOVE) < 0.0:
        raise ValueError("MPC_LAMBDA_MOVE must be >= 0")
    max_candidates = int(MPC_MAX_CANDIDATES)
    if (max_candidates < 3) or (max_candidates > 201):
        raise ValueError("MPC_MAX_CANDIDATES must be in 3..201")
    observer_gain = float(MPC_OBSERVER_GAIN)
    if (observer_gain < 0.0) or (observer_gain > 1.0):
        raise ValueError("MPC_OBSERVER_GAIN must be in 0..1")


def _validate_model_defaults() -> None:
    u1 = float(FOPDT_U1_PERCENT)
    if (u1 < 0.0) or (u1 > 100.0):
        raise ValueError("FOPDT_U1_PERCENT must be in 0..100")
    if u1 <= 0.0:
        raise ValueError("FOPDT_U1_PERCENT must be > 0 (heating step from 0%% baseline)")
    if int(FOPDT_SMOOTH_N) < 1:
        raise ValueError("FOPDT_SMOOTH_N must be >= 1")
    if abs(float(MODEL_K)) <= _PID_ZERO_EPS:
        raise ValueError("MODEL_K must be non-zero")
    if float(MODEL_TAU_S) <= 0.0:
        raise ValueError("MODEL_TAU_S must be > 0")
    if float(MODEL_THETA_S) < 0.0:
        raise ValueError("MODEL_THETA_S must be >= 0")
    u0_pct = float(MODEL_U0_PCT)
    if (u0_pct < 0.0) or (u0_pct > 100.0):
        raise ValueError("MODEL_U0_PCT must be in 0..100")


def validate() -> None:
    _validate_core()
    _validate_safety()
    _validate_onoff()
    _validate_pid()
    _validate_fuzzy()
    _validate_mpc()
    _validate_model_defaults()
