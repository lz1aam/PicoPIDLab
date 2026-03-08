# profile.py
"""Student-editable experiment knobs. See docs/en/PROFILE_TUTORIAL.md for usage guidance."""

# Validation constants (single source of truth)
try:
    from builder import CONTROL_MODES as ALLOWED_CONTROL_MODES
except Exception:
    ALLOWED_CONTROL_MODES = ("PID", "ONOFF", "FUZZY", "MPC")
ALLOWED_SETPOINT_TYPES = ("STEP", "RAMP")
ALLOWED_DIST_MODES = ("STEP", "PULSE")
ALLOWED_TUNING_RULES = (
    "ZN_1_P", "ZN_1_PI", "ZN_1_PID",
    "CC_P", "CC_PI", "CC_PID",
    "ZN_2_P", "ZN_2_PI", "ZN_2_PID",
    "TL_P", "TL_PI", "TL_PID",
)
MODEL_TUNING_RULES = (
    "ZN_1_P", "ZN_1_PI", "ZN_1_PID",
    "CC_P", "CC_PI", "CC_PID",
)
RELAY_TUNING_RULES = (
    "ZN_2_P", "ZN_2_PI", "ZN_2_PID",
    "TL_P", "TL_PI", "TL_PID",
)
ALLOWED_PID_VARIANTS = ("PID", "2DOF", "FF_PID", "GAIN_SCHED", "SMITH_PI")
ALLOWED_PID_AW_TYPES = ("NONE", "CLAMP", "BACKCALC")
ALLOWED_PID_ALGORITHM = ("IDEAL", "PARALLEL", "SERIES")
ALLOWED_TELEMETRY_MODES = ("INFO", "NORMAL", "MPC")


# Session + core runtime (all modes)
CONTROL_MODE = "PID"             # Effect: selects controller family (ONOFF/PID/FUZZY/MPC), default PID
SETPOINT_TYPE = "STEP"           # Effect: STEP = abrupt SP, RAMP = slewed SP
SETPOINT_C = 50.0                 # Effect: raises/lowers target temperature
SETPOINT_RAMP_RATE = 5.0          # [degC/s], Effect: lower = smoother SP change
EXPERIMENT_RUN_S = 200.0          # [s] or None, Effect: auto-stop run length
TS_S = 0.25                       # [s], Effect: loop cadence vs CPU load
TELEMETRY_MODE = "NORMAL"        # Effect: INFO/NORMAL/MPC runtime print volume

# Control-path disturbance injection (applies during control runs only)
DIST_ENABLE = False               # Effect: enables/disables OP disturbance injection
DIST_MODE = "STEP"               # STEP or PULSE
DIST_MAG_PCT = -25.0              # [%], Effect: additive output disturbance magnitude
DIST_START_S = 60.0               # [s], Effect: disturbance start time from run start
DIST_DURATION_S = 30.0            # [s], Effect: pulse width when DIST_MODE='PULSE'


# Safety + hardware (all modes)
TEMP_CUTOFF_C = 85.0              # [degC], Effect: hard safety cutoff
SAFETY_HOLD_S = 5.0               # [s], Effect: post-cutoff hold before recovery
SAFETY_HYST_C = 5.0               # [degC], Effect: cooldown margin to clear cutoff


# PID mode (CONTROL_MODE == "PID")
PID_VARIANT = "PID"              # Effect: PID / 2DOF / FF_PID / GAIN_SCHED / SMITH_PI
PID_AW_TYPE = "CLAMP"            # Effect: NONE / CLAMP / BACKCALC anti-windup
PID_ALGORITHM = "PARALLEL"       # Effect: gain form used by active controller

# PARALLEL gains (used when PID_ALGORITHM == "PARALLEL")
KP = 43.671                       # Effect: proportional aggressiveness
KI = 8.47154                      # Effect: integral aggressiveness
KD = 56.2813                      # Effect: derivative damping/anticipation

# IDEAL/SERIES gains (used when PID_ALGORITHM == "IDEAL" or "SERIES")
KC = 43.671                       # Effect: ideal/series proportional gain
TI_S = 5.15503                    # [s], Effect: lower = stronger integral action
TD_S = 1.28876                    # [s], Effect: higher = stronger derivative action
SPAN = 60.0                       # [degC], Effect: PB-equivalent reporting/conversion span (not plant dynamics)

# Shared PID shaping
DERIVATIVE_FILTER_ALPHA = 0.2     # [0..1], Effect: higher = less derivative filtering
PID_INTEGRAL_LIMIT = 100          # [%] or None, Effect: caps integral contribution

# Variant-specific PID knobs
PID_AW_TRACKING_TIME_S = 5.0      # [s], BACKCALC only; lower = stronger unwind
PID_BETA = 0.6                    # [0..1], 2DOF only; lower = softer SP response
FF_MODE = "FOPDT_GAIN"            # FF_PID only; MANUAL or FOPDT_GAIN
FF_GAIN_PCT_PER_C = 1.5           # FF_PID MANUAL gain
FF_BIAS_PCT = 0.0                 # FF_PID MANUAL bias
FF_AMBIENT_C = 25                 # [degC] or None; FF ambient reference
GS_VARIABLE = "PV"                # GAIN_SCHED only; schedule on PV or SP
GS_TABLE = (                      # GAIN_SCHED only; (temp_c, kp, ki, kd)
    (25.0, 6.0, 0.40, 0.05),
    (50.0, 5.0, 0.50, 0.10),
    (75.0, 4.0, 0.60, 0.15),
)

# PID tuning workflow
TUNING_RULE = "CC_PID"           # model: ZN_1_*/CC_* ; relay: ZN_2_*/TL_*

# Relay tuning knobs
TUNING_TARGET_C = 50.0            # [degC], relay target
TUNING_BAND_C = 1.0               # [degC], relay hysteresis half-band
TUNING_CYCLES = 5                 # Effect: relay cycle count used for averaging


# ON/OFF mode (CONTROL_MODE == "ONOFF")
ONOFF_HYST_C = 1.0                # [degC], Effect: wider = fewer switches, larger swing
ONOFF_ON_PERCENT = 100.0          # [%], Effect: ON-state heater power
ONOFF_MIN_SWITCH_S = 1.0          # [s], Effect: anti-chatter minimum dwell


# FUZZY mode (CONTROL_MODE == "FUZZY")
FUZZY_E_SCALE_C = 8.0             # Effect: input error normalization scale
FUZZY_DE_SCALE_C_PER_S = 1.0      # Effect: d(error)/dt normalization scale
FUZZY_DU_RATE_MAX = 20.0          # [%/s], Effect: output slew cap
FUZZY_DE_FILTER_ALPHA = 0.6       # [0..1], Effect: derivative filtering
FUZZY_RULE_TABLE = (
    (-2, -2, -2, -1, 0),
    (-2, -2, -1, 0, 1),
    (-2, -1, 0, 1, 2),
    (-1, 0, 1, 2, 2),
    (0, 1, 2, 2, 2),
)


# MPC mode (CONTROL_MODE == "MPC")
MPC_HORIZON_STEPS = 100           # Effect: larger horizon improves foresight, costs CPU
MPC_GRID_STEP_PCT = 2.0           # [%], Effect: smaller step gives finer search, costs CPU
MPC_DU_MAX_PCT = 15.0             # [%], Effect: move-rate cap
MPC_LAMBDA_MOVE = 0.02            # Effect: move suppression weight
MPC_MAX_CANDIDATES = 41           # Effect: search breadth per update
MPC_OBSERVER_GAIN = 1.0           # [0..1], Effect: model correction gain


# FOPDT identification (used by 'model' command)
FOPDT_U1_PERCENT = 100.0           # [%], Effect: stepped output level
FOPDT_SMOOTH_N = 10                # Effect: smoothing window for identification
STEADY_WINDOW_S = 5.0             # [s], Effect: steady-state observation window (model phases)
STEADY_BAND_C = 0.5                # [degC], Effect: steady-state PV band (model phases)

# FOPDT runtime defaults (used by MPC/SMITH/FF when no runtime model is in RAM)
MODEL_K = 0.587880                 # [degC/%], Effect: default process gain for model-based controllers
MODEL_TAU_S = 55.14                # [s], Effect: default model time constant
MODEL_THETA_S = 2.58               # [s], Effect: default model dead time
MODEL_U0_PCT = 0.0                 # [%], Effect: default model operating output
MODEL_Y0 = 24.652                  # [degC], Effect: default model operating PV
MODEL_Y1 = 83.440                  # [degC], Effect: optional reference final PV for reports
MODEL_RMSE = 0.621                 # [degC], Effect: optional model-fit reference metric for reports
MODEL_METHOD = "SK"                # Effect: optional reference method label for reports


# Validation (teacher/debug helper)
def validate() -> None:
    """Validate profile parameters.

    Raises ValueError with a clear message if something is invalid.
    """
    if CONTROL_MODE not in ALLOWED_CONTROL_MODES:
        raise ValueError("CONTROL_MODE must be one of: %s" % (ALLOWED_CONTROL_MODES,))

    if SETPOINT_TYPE not in ALLOWED_SETPOINT_TYPES:
        raise ValueError("SETPOINT_TYPE must be 'STEP' or 'RAMP'")

    if TS_S <= 0:
        raise ValueError("TS_S must be > 0")
    if float(STEADY_WINDOW_S) <= 0.0:
        raise ValueError("STEADY_WINDOW_S must be > 0")
    if float(STEADY_BAND_C) <= 0.0:
        raise ValueError("STEADY_BAND_C must be > 0")
    tm = str(TELEMETRY_MODE).upper()
    if tm not in ALLOWED_TELEMETRY_MODES:
        raise ValueError("TELEMETRY_MODE must be one of: %s" % (ALLOWED_TELEMETRY_MODES,))
    dm = str(DIST_MODE).upper()
    if dm not in ALLOWED_DIST_MODES:
        raise ValueError("DIST_MODE must be one of: %s" % (ALLOWED_DIST_MODES,))
    if float(DIST_START_S) < 0.0:
        raise ValueError("DIST_START_S must be >= 0")
    if dm == "PULSE" and float(DIST_DURATION_S) <= 0.0:
        raise ValueError("DIST_DURATION_S must be > 0 when DIST_MODE='PULSE'")
    if SETPOINT_TYPE == "RAMP" and SETPOINT_RAMP_RATE <= 0:
        raise ValueError("SETPOINT_RAMP_RATE must be > 0 when using RAMP")

    # Experiment timer (optional)
    exp_s = EXPERIMENT_RUN_S
    if exp_s is not None:
        if float(exp_s) <= 0.0:
            raise ValueError("EXPERIMENT_RUN_S must be > 0 (or set to None to disable)")

    # Safety
    if TEMP_CUTOFF_C <= -50 or TEMP_CUTOFF_C >= 140:
        raise ValueError("TEMP_CUTOFF_C looks unrealistic; check units (°C)")
    if SAFETY_HOLD_S < 0:
        raise ValueError("SAFETY_HOLD_S must be >= 0")
    if SAFETY_HYST_C < 0:
        raise ValueError("SAFETY_HYST_C must be >= 0")
    # ON/OFF controller
    if CONTROL_MODE == "ONOFF":
        if ONOFF_HYST_C <= 0:
            raise ValueError("ONOFF_HYST_C must be > 0")
        if ONOFF_ON_PERCENT < 0 or ONOFF_ON_PERCENT > 100:
            raise ValueError("ONOFF_ON_PERCENT must be in 0..100")
        if ONOFF_MIN_SWITCH_S < 0:
            raise ValueError("ONOFF_MIN_SWITCH_S must be >= 0")

    # PID family
    if CONTROL_MODE == "PID":
        if str(TUNING_RULE).upper() not in ALLOWED_TUNING_RULES:
            raise ValueError("TUNING_RULE must be one of: %s" % (ALLOWED_TUNING_RULES,))

        v = str(PID_VARIANT).upper()
        aw = str(PID_AW_TYPE).upper()

        if v not in ALLOWED_PID_VARIANTS:
            raise ValueError("PID_VARIANT must be one of: %s" % (ALLOWED_PID_VARIANTS,))
        if aw not in ALLOWED_PID_AW_TYPES:
            raise ValueError("PID_AW_TYPE must be one of: %s" % (ALLOWED_PID_AW_TYPES,))

        p_algo = str(PID_ALGORITHM).upper()
        if p_algo not in ALLOWED_PID_ALGORITHM:
            raise ValueError("PID_ALGORITHM must be one of: %s" % (ALLOWED_PID_ALGORITHM,))

        if PID_INTEGRAL_LIMIT is not None and float(PID_INTEGRAL_LIMIT) <= 0:
            raise ValueError("PID_INTEGRAL_LIMIT must be > 0, or None")

        # SPAN is required for Pb-equivalent reporting and relay quality-gate checks.
        if float(SPAN) <= 0.0:
            raise ValueError("SPAN must be > 0")

        # IDEAL/SERIES forms use KC,TI_S,TD_S directly.
        if p_algo in ("IDEAL", "SERIES"):
            ti = float(TI_S)
            td = float(TD_S)
            if ti < 0.0:
                raise ValueError("TI_S must be >= 0 for PID_ALGORITHM='IDEAL'/'SERIES'")
            if td < 0.0:
                raise ValueError("TD_S must be >= 0 for PID_ALGORITHM='IDEAL'/'SERIES'")
            if p_algo == "SERIES":
                if abs(float(KC)) <= 1e-12 and ((ti > 0.0) or (td > 0.0)):
                    raise ValueError("PID_ALGORITHM='SERIES' requires KC != 0 when TI_S or TD_S is active")

        # Variant-specific validation
        if v == "PID" and aw == "BACKCALC":
            if PID_AW_TRACKING_TIME_S <= 0:
                raise ValueError("PID_AW_TRACKING_TIME_S must be > 0")

        if v == "2DOF":
            if float(PID_BETA) < 0.0 or float(PID_BETA) > 1.0:
                raise ValueError("PID_BETA must be in 0..1")
            if p_algo != "PARALLEL":
                raise ValueError("PID_VARIANT='2DOF' requires PID_ALGORITHM='PARALLEL' (current: %s)" % p_algo)

        if v == "FF_PID":
            m = str(FF_MODE).upper()
            if m not in ("MANUAL", "FOPDT_GAIN"):
                raise ValueError("FF_MODE must be 'MANUAL' or 'FOPDT_GAIN'")
            if FF_AMBIENT_C is not None and (FF_AMBIENT_C <= -50 or FF_AMBIENT_C >= 200):
                raise ValueError("FF_AMBIENT_C looks unrealistic; check units (°C)")
            if p_algo != "PARALLEL":
                raise ValueError("PID_VARIANT='FF_PID' requires PID_ALGORITHM='PARALLEL' (current: %s)" % p_algo)

        if v == "GAIN_SCHED":
            if str(GS_VARIABLE).upper() not in ("PV", "SP"):
                raise ValueError("GS_VARIABLE must be 'PV' or 'SP'")
            if GS_TABLE is None or len(GS_TABLE) < 2:
                raise ValueError("GS_TABLE must have at least 2 breakpoints")
            for i, row in enumerate(GS_TABLE):
                if len(row) != 4:
                    raise ValueError("GS_TABLE[%d] must be (temp,kp,ki,kd)" % i)

        # Smith predictor requires PI-capable runtime gains and active MODEL_* values in controller build path.
        if v == "SMITH_PI":
            pass

    # FUZZY controller
    if CONTROL_MODE == "FUZZY":
        if FUZZY_E_SCALE_C <= 0:
            raise ValueError("FUZZY_E_SCALE_C must be > 0")
        if FUZZY_DE_SCALE_C_PER_S <= 0:
            raise ValueError("FUZZY_DE_SCALE_C_PER_S must be > 0")
        if FUZZY_DU_RATE_MAX <= 0:
            raise ValueError("FUZZY_DU_RATE_MAX must be > 0")
        a = float(FUZZY_DE_FILTER_ALPHA)
        if a < 0.0 or a > 1.0:
            raise ValueError("FUZZY_DE_FILTER_ALPHA must be in 0..1")

        tbl = FUZZY_RULE_TABLE
        if tbl is None or len(tbl) != 5:
            raise ValueError("FUZZY_RULE_TABLE must have 5 rows")
        for r in range(5):
            row = tbl[r]
            if len(row) != 5:
                raise ValueError("FUZZY_RULE_TABLE row %d must have 5 columns" % r)
            for c in range(5):
                try:
                    v = int(row[c])
                except Exception:
                    raise ValueError("FUZZY_RULE_TABLE[%d][%d] must be int" % (r, c))
                if v < -2 or v > 2:
                    raise ValueError("FUZZY_RULE_TABLE[%d][%d] must be in -2..2" % (r, c))

    # MPC
    if CONTROL_MODE == "MPC":
        if MPC_HORIZON_STEPS < 3:
            raise ValueError("MPC_HORIZON_STEPS must be >= 3")
        if MPC_GRID_STEP_PCT <= 0:
            raise ValueError("MPC_GRID_STEP_PCT must be > 0")
        if MPC_DU_MAX_PCT <= 0:
            raise ValueError("MPC_DU_MAX_PCT must be > 0")
        if MPC_LAMBDA_MOVE < 0:
            raise ValueError("MPC_LAMBDA_MOVE must be >= 0")
        if int(MPC_MAX_CANDIDATES) < 3 or int(MPC_MAX_CANDIDATES) > 201:
            raise ValueError("MPC_MAX_CANDIDATES must be in 3..201")
        if MPC_OBSERVER_GAIN < 0 or MPC_OBSERVER_GAIN > 1:
            raise ValueError("MPC_OBSERVER_GAIN must be in 0..1")

    if CONTROL_MODE == "PID":
        if float(TUNING_BAND_C) <= 0:
            raise ValueError("TUNING_BAND_C must be > 0")
        if int(TUNING_CYCLES) < 1:
            raise ValueError("TUNING_CYCLES must be >= 1")
        if float(TUNING_TARGET_C) <= -50 or float(TUNING_TARGET_C) >= 200:
            raise ValueError("TUNING_TARGET_C looks unrealistic; check units (°C)")

    # FOPDT identification knobs (used by 'model' command)
    if FOPDT_U1_PERCENT < 0 or FOPDT_U1_PERCENT > 100:
        raise ValueError("FOPDT_U1_PERCENT must be in 0..100")
    if FOPDT_U1_PERCENT <= 0:
        raise ValueError("FOPDT_U1_PERCENT must be > 0 (heating step from 0%% baseline)")
    if int(FOPDT_SMOOTH_N) < 1:
        raise ValueError("FOPDT_SMOOTH_N must be >= 1")

    # FOPDT model defaults (used by model-based controllers when runtime model is absent)
    if abs(float(MODEL_K)) <= 1e-12:
        raise ValueError("MODEL_K must be non-zero")
    if float(MODEL_TAU_S) <= 0.0:
        raise ValueError("MODEL_TAU_S must be > 0")
    if float(MODEL_THETA_S) < 0.0:
        raise ValueError("MODEL_THETA_S must be >= 0")
    if float(MODEL_U0_PCT) < 0.0 or float(MODEL_U0_PCT) > 100.0:
        raise ValueError("MODEL_U0_PCT must be in 0..100")
