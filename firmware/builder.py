"""Controller factory for runtime profile -> controller instance mapping."""

import math

from controllers import (
    PIDParallelPercent,
    PIDSeriesPercent,
    PID2DOFPercent,
    PIDFeedForwardPercent,
    GainScheduledPIDPercent,
    TwoPositionPercent,
    SmithPredictorPI,
    FuzzySugenoIncrementalPercent,
    MPCLitePercent,
)
from model import load_effective_model


def pid_forms_from_gains(kp: float, ki: float, kd: float, span: float):
    """Return K/IDEAL/PB equivalent forms derived from internal gains.

    Pedagogical symbol mapping:
    - K form:  Kp=kp, Ki=ki, Kd=kd
    - IDEAL form: Kc, Ti, Td
    - PB view: Pb[%] = 10000 / (Kp * SPAN[degC])
    """
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
    pb_pct = None
    if kp > 0.0:
        pb_pct = 10000.0 / (kp * span_val)

    return {
        "K": {"KP": kp, "KI": ki, "KD": kd},
        "IDEAL": {"KC": kp, "TI_S": ti_s, "TD_S": td_s},
        "PB": {"PB": pb_pct, "SPAN": span_val, "TI_S": ti_s, "TD_S": td_s},
    }


def _series_to_ideal_equivalent(kc: float, ti_s: float, td_s: float):
    """Convert SERIES (interacting) parameters to effective IDEAL equivalent."""
    kc = float(kc)
    ti_s = float(ti_s)
    td_s = float(td_s)

    if ti_s <= 0.0:
        return {"KC": kc, "TI_S": None, "TD_S": td_s if td_s > 0.0 else 0.0}
    if td_s <= 0.0:
        return {"KC": kc, "TI_S": ti_s, "TD_S": 0.0}

    kc_eff = kc * (1.0 + (td_s / ti_s))
    ti_eff = ti_s + td_s
    td_eff = (ti_s * td_s) / (ti_s + td_s)
    return {"KC": kc_eff, "TI_S": ti_eff, "TD_S": td_eff}


def pid_descriptor_from_profile(profile):
    """Return PID descriptor derived directly from PID_ALGORITHM."""
    algorithm = str(profile.PID_ALGORITHM).upper()
    if algorithm not in ("IDEAL", "PARALLEL", "SERIES"):
        raise ValueError("PID_ALGORITHM must be one of: ('IDEAL', 'PARALLEL', 'SERIES')")

    if algorithm == "PARALLEL":
        kp = float(profile.KP)
        ki = float(profile.KI)
        kd = float(profile.KD)
        kc = kp
        ti_s = (kp / ki) if (ki > 0.0 and abs(kp) > 1e-12) else 0.0
        td_s = (kd / kp) if abs(kp) > 1e-12 else 0.0
        configured = {"KP": kp, "KI": ki, "KD": kd}
        eff_ideal = {
            "KC": kp,
            "TI_S": (kp / ki) if (ki > 0.0 and abs(kp) > 1e-12) else None,
            "TD_S": (kd / kp) if abs(kp) > 1e-12 else (0.0 if kd == 0.0 else None),
        }
    else:
        kc = float(profile.KC)
        ti_s = float(profile.TI_S)
        td_s = float(profile.TD_S)
        configured = {"KC": kc, "TI_S": ti_s, "TD_S": td_s}

        if algorithm == "SERIES":
            if abs(float(kc)) <= 1e-12 and ((float(ti_s) > 0.0) or (float(td_s) > 0.0)):
                raise ValueError("PID_ALGORITHM='SERIES' requires KC != 0 when TI_S or TD_S is active")
            eff_ideal = _series_to_ideal_equivalent(kc, ti_s, td_s)
            kp = float(eff_ideal["KC"])
            ti_eff = eff_ideal.get("TI_S")
            td_eff = eff_ideal.get("TD_S")
            ki = (kp / float(ti_eff)) if (ti_eff is not None and float(ti_eff) > 0.0) else 0.0
            kd = (kp * float(td_eff)) if (td_eff is not None and float(td_eff) > 0.0) else 0.0
        else:
            kp = kc
            ki = (kc / ti_s) if ti_s > 0.0 else 0.0
            kd = (kc * td_s) if td_s > 0.0 else 0.0
            eff_ideal = {"KC": kc, "TI_S": ti_s if ti_s > 0.0 else None, "TD_S": td_s if td_s > 0.0 else 0.0}

    forms = pid_forms_from_gains(kp, ki, kd, float(profile.SPAN))
    return {
        "algorithm": algorithm,
        "configured": configured,
        "parallel": {"KP": kp, "KI": ki, "KD": kd},
        "ideal": {"KC": kc, "TI_S": ti_s, "TD_S": td_s},
        "effective_ideal": eff_ideal,
        "forms": forms,
    }


def pid_selection_from_profile(profile):
    """Return canonical PID family + AW selection from runtime profile.

    Canonical family:
    - PID, 2DOF, FF_PID, GAIN_SCHED, SMITH_PI
    Canonical AW type (used when family == PID):
    - NONE, CLAMP, BACKCALC

    Settings are strict: no backward-compat aliases.
    """
    raw_variant = str(profile.PID_VARIANT).upper()
    aw_type_cfg = str(profile.PID_AW_TYPE).upper()
    return {"raw_variant": raw_variant, "variant": raw_variant, "aw_type": aw_type_cfg}


def _resolve_fopdt_model(profile):
    """Return active model from runtime profile values."""
    return load_effective_model(profile)

def _build_onoff(profile, _sensor, _heater, _indicator, _poll_cmd=None):
    print("# INFO: ON/OFF config: hyst=+/-%.2f°C  on=%.1f%%  min_switch=%.2fs"
          % (profile.ONOFF_HYST_C, profile.ONOFF_ON_PERCENT, profile.ONOFF_MIN_SWITCH_S))
    controller = TwoPositionPercent(
        hyst_c=profile.ONOFF_HYST_C,
        on_percent=profile.ONOFF_ON_PERCENT,
        min_switch_s=profile.ONOFF_MIN_SWITCH_S,
    )
    return controller, False


def _build_fuzzy(profile, sensor, heater, indicator, poll_cmd=None):
    print("# INFO: FUZZY config: E_scale=%.3f°C  dE_scale=%.3f°C/s  du_rate_max=%.2f%%/s  de_alpha=%.2f"
          % (profile.FUZZY_E_SCALE_C, profile.FUZZY_DE_SCALE_C_PER_S, profile.FUZZY_DU_RATE_MAX, profile.FUZZY_DE_FILTER_ALPHA))
    controller = FuzzySugenoIncrementalPercent(
        e_scale_c=profile.FUZZY_E_SCALE_C,
        de_scale_c_per_s=profile.FUZZY_DE_SCALE_C_PER_S,
        du_rate_max=profile.FUZZY_DU_RATE_MAX,
        rules_table=profile.FUZZY_RULE_TABLE,
        d_filter_alpha=profile.FUZZY_DE_FILTER_ALPHA,
    )
    return controller, False


def _build_mpc(profile, _sensor, _heater, _indicator, _poll_cmd=None):
    model = _resolve_fopdt_model(profile)
    if model is None:
        raise ValueError("MPC requires MODEL_* values. Run 'model' or set MODEL_* in profile.py.")
    K = float(model["K"])
    tau = float(model["tau_s"])
    theta = float(model["theta_s"])
    u0 = float(model["u0_pct"])
    y0 = float(model["y0"])
    runtime_y0 = False
    print("# INFO: MPC(FOPDT): K=%.6f °C/%% tau=%.2fs theta=%.2fs u0=%.2f%% y0=%.2f L=%.2f"
          % (K, tau, theta, u0, y0, float(profile.MPC_OBSERVER_GAIN)))

    horizon = int(profile.MPC_HORIZON_STEPS)
    eff_max_candidates = int(profile.MPC_MAX_CANDIDATES)
    dt_nom = float(profile.TS_S)
    n_delay = int(math.ceil(max(0.0, float(theta)) / max(1e-9, dt_nom)))
    min_horizon = n_delay + 1
    if horizon < min_horizon:
        raise ValueError(
            "MPC_HORIZON_STEPS too short for dead-time: horizon=%d, delay_steps=%d. "
            "Set MPC_HORIZON_STEPS >= %d (theta=%.2fs, TS_S=%.3fs)."
            % (horizon, n_delay, min_horizon, float(theta), dt_nom)
        )

    controller = MPCLitePercent(
        K_c_per_pct=K,
        tau_s=tau,
        theta_s=theta,
        u0_pct=u0,
        dt_nominal_s=dt_nom,
        horizon_steps=int(horizon),
        grid_step_pct=float(profile.MPC_GRID_STEP_PCT),
        du_max_pct=float(profile.MPC_DU_MAX_PCT),
        lambda_move=float(profile.MPC_LAMBDA_MOVE),
        max_candidates=int(eff_max_candidates),
        observer_gain=float(profile.MPC_OBSERVER_GAIN),
        y0_runtime_capture=runtime_y0,
        y0_c=y0,
    )
    return controller, False


def _build_pid(profile, sensor, heater, indicator, poll_cmd=None):
    tuning_safety = False
    controller = None
    pid_desc = pid_descriptor_from_profile(profile)
    _model_loaded = False
    _cached_model = None

    def _get_model():
        nonlocal _model_loaded, _cached_model
        if not _model_loaded:
            _cached_model = _resolve_fopdt_model(profile)
            _model_loaded = True
        return _cached_model

    pid_sel = pid_selection_from_profile(profile)
    pid_variant = pid_sel["variant"]
    pid_aw_type = pid_sel["aw_type"]

    KP = float(pid_desc["parallel"]["KP"])
    KI = float(pid_desc["parallel"]["KI"])
    KD = float(pid_desc["parallel"]["KD"])
    KC = float(pid_desc["ideal"]["KC"])
    TI_S = float(pid_desc["ideal"]["TI_S"])
    TD_S = float(pid_desc["ideal"]["TD_S"])
    pid_algo = pid_desc["algorithm"]
    use_ideal_form = pid_algo in ("IDEAL", "SERIES")
    profile.KP = float(KP)
    profile.KI = float(KI)
    profile.KD = float(KD)
    profile.KC = float(KC)
    profile.TI_S = float(TI_S)
    profile.TD_S = float(TD_S)
    if TI_S <= 0.0:
        print("# INFO: PID params: Ti (TI_S)<=0 -> integral disabled (Ki=0)")
    if TD_S <= 0.0:
        print("# INFO: PID params: Td (TD_S)<=0 -> derivative disabled (Kd=0)")

    if pid_variant == "SMITH_PI":
        model = _get_model()
        if model is None:
            raise ValueError("SMITH_PI requires MODEL_* values (run 'model' or set MODEL_* in profile.py)")
        if abs(float(KP)) <= 1e-12:
            raise ValueError("SMITH_PI requires non-zero Kp from active runtime gains")
        if float(KI) <= 0.0:
            raise ValueError("SMITH_PI requires Ki>0 from active runtime gains (PI action)")
        print("# INFO: SMITH_PI model: K=%.6f °C/%% tau=%.2fs theta=%.2fs"
              % (model.get("K", 0.0), model.get("tau_s", 0.0), model.get("theta_s", 0.0)))
        print("# INFO: SMITH_PI gains from runtime values: Kp=%.3f Ki=%.3f" % (KP, KI))
        controller = SmithPredictorPI(
            kp=KP,
            ki=KI,
            K_proc=float(model.get("K", 0.0)),
            tau_s=float(model.get("tau_s", 0.0)),
            theta_s=float(model.get("theta_s", 0.0)),
            dt_nominal_s=float(profile.TS_S),
            u0_pct=float(model.get("u0_pct", 0.0)),
        )

    if controller is None:
        if pid_variant == "PID":
            aw_type = pid_aw_type if pid_aw_type in ("NONE", "BACKCALC", "CLAMP") else "CLAMP"
            k_aw = 0.0
            tt = None
            if aw_type == "BACKCALC":
                tt = float(profile.PID_AW_TRACKING_TIME_S)
                k_aw = 1.0 / tt if tt > 0 else 0.0

            if pid_algo == "SERIES":
                if aw_type == "BACKCALC":
                    print("# INFO: PID(PID_AW_TYPE=BACKCALC/SERIES): Kc=%.3f Ti=%.3fs Td=%.3fs Tt=%.2fs (K_aw=%.3f 1/s)"
                          % (KC, TI_S, TD_S, tt, k_aw))
                else:
                    print("# INFO: PID(PID_AW_TYPE=%s/SERIES): Kc=%.3f Ti=%.3fs Td=%.3fs" % (aw_type, KC, TI_S, TD_S))
                controller = PIDSeriesPercent(
                    kc=KC,
                    ti_s=TI_S,
                    td_s=TD_S,
                    aw_type=aw_type,
                    kaw=k_aw,
                    d_filter_alpha=profile.DERIVATIVE_FILTER_ALPHA,
                    integral_limit=profile.PID_INTEGRAL_LIMIT,
                )
            else:
                if (aw_type == "NONE") and (profile.PID_INTEGRAL_LIMIT is not None):
                    print("# WARNING: PID integral_limit is set with PID_AW_TYPE=NONE; windup demonstration may be masked")
                if aw_type == "BACKCALC":
                    if use_ideal_form:
                        print("# INFO: PID(PID_AW_TYPE=BACKCALC/%s): Kc=%.3f Ti=%.3fs Td=%.3fs Tt=%.2fs (K_aw=%.3f 1/s)"
                              % (pid_algo, KC, TI_S, TD_S, tt, k_aw))
                    else:
                        print("# INFO: PID(PID_AW_TYPE=BACKCALC/%s): Kp=%.3f  Ki=%.3f  Kd=%.3f  Tt=%.2fs (K_aw=%.3f 1/s)"
                              % (pid_algo, KP, KI, KD, tt, k_aw))
                else:
                    if use_ideal_form:
                        print("# INFO: PID(PID_AW_TYPE=%s/%s): Kc=%.3f Ti=%.3fs Td=%.3fs" % (aw_type, pid_algo, KC, TI_S, TD_S))
                    else:
                        print("# INFO: PID(PID_AW_TYPE=%s/%s): Kp=%.3f  Ki=%.3f  Kd=%.3f" % (aw_type, pid_algo, KP, KI, KD))
                controller = PIDParallelPercent(
                    kp=KP,
                    ki=KI,
                    kd=KD,
                    aw_type=aw_type,
                    kaw=k_aw,
                    d_filter_alpha=profile.DERIVATIVE_FILTER_ALPHA,
                    integral_limit=profile.PID_INTEGRAL_LIMIT,
                )
        elif pid_variant == "2DOF":
            beta = float(profile.PID_BETA)
            if pid_algo == "SERIES":
                raise ValueError("PID_VARIANT='2DOF' supports PID_ALGORITHM='PARALLEL' only")
            print("# INFO: PID(2DOF): Kp=%.3f  Ki=%.3f  Kd=%.3f  beta=%.2f"
                  % (KP, KI, KD, beta))
            controller = PID2DOFPercent(
                kp=KP, ki=KI, kd=KD, beta=beta,
                d_filter_alpha=profile.DERIVATIVE_FILTER_ALPHA,
                integral_limit=profile.PID_INTEGRAL_LIMIT,
            )
        elif pid_variant == "FF_PID":
            ff_mode = str(profile.FF_MODE).upper()
            if pid_algo == "SERIES":
                raise ValueError("PID_VARIANT='FF_PID' supports PID_ALGORITHM='PARALLEL' only")
            ambient = profile.FF_AMBIENT_C
            K_ff = 0.0
            u0_ff = 0.0
            if ff_mode == "FOPDT_GAIN":
                model = _get_model()
                if model is None:
                    raise ValueError("FF_MODE='FOPDT_GAIN' requires MODEL_* values")
                else:
                    K_ff = float(model.get("K", 0.0))
                    u0_ff = float(model.get("u0_pct", 0.0))
                    if ambient is None:
                        ambient = model.get("y0", None)
            print("# INFO: PID(FF_PID): Kp=%.3f Ki=%.3f Kd=%.3f  FF_MODE=%s" % (KP, KI, KD, ff_mode))
            controller = PIDFeedForwardPercent(
                kp=KP, ki=KI, kd=KD,
                ff_mode=ff_mode,
                ff_gain_pct_per_c=float(profile.FF_GAIN_PCT_PER_C),
                ff_bias_pct=float(profile.FF_BIAS_PCT),
                K_c_per_pct=K_ff,
                u0_pct=u0_ff,
                ambient_c=ambient,
                d_filter_alpha=profile.DERIVATIVE_FILTER_ALPHA,
                integral_limit=profile.PID_INTEGRAL_LIMIT,
            )
        elif pid_variant == "GAIN_SCHED":
            print("# INFO: PID(GAIN_SCHED): variable=%s  breakpoints=%d"
                  % (profile.GS_VARIABLE, len(profile.GS_TABLE)))
            controller = GainScheduledPIDPercent(
                schedule_table=profile.GS_TABLE,
                schedule_variable=profile.GS_VARIABLE,
                d_filter_alpha=profile.DERIVATIVE_FILTER_ALPHA,
                integral_limit=profile.PID_INTEGRAL_LIMIT,
            )
        else:
            raise ValueError("Unknown PID_VARIANT '%s'" % pid_variant)

    return controller, tuning_safety


CONTROL_MODE_REGISTRY = {
    "PID": _build_pid,
    "ONOFF": _build_onoff,
    "FUZZY": _build_fuzzy,
    "MPC": _build_mpc,
}
CONTROL_MODES = tuple(CONTROL_MODE_REGISTRY.keys())


def build_controller(profile, sensor, heater, indicator, poll_cmd=None):
    """Return (controller, tuning_safety_flag) for current profile mode."""
    try:
        profile._RUN_ABORTED = False
    except Exception:
        pass
    mode = str(profile.CONTROL_MODE).upper()
    builder = CONTROL_MODE_REGISTRY.get(mode)
    if builder is None:
        raise ValueError("Unknown CONTROL_MODE: %s" % mode)
    return builder(profile, sensor, heater, indicator, poll_cmd)
