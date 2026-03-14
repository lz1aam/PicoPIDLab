"""Terminal command interface for runtime configuration and control."""
import sys
from hardware import yield_cpu

try:
    import uselect as _select
except Exception:
    try:
        import select as _select
    except Exception:
        _select = None


_STRING_PARAMS_UPPER = {
    "CONTROL_MODE",
    "SETPOINT_TYPE",
    "TUNING_RULE",
    "PID_VARIANT",
    "PID_AW_TYPE",
    "PID_ALGORITHM",
    "TELEMETRY_MODE",
    "GS_VARIABLE",
    "FF_MODE",
    "MODEL_METHOD",
    "DIST_MODE",
}
_COMPLEX_ASSIGN_BLOCKLIST = {"GS_TABLE", "FUZZY_RULE_TABLE"}
_poll_err_count = 0


_PARAM_GROUPS = {
    "core": (
        "General settings",
        (
            ("CONTROL_MODE", "ONOFF/PID/FUZZY/MPC"),
            ("SETPOINT_TYPE", "STEP or RAMP"),
            ("SETPOINT_C", "Target [°C]"),
            ("SETPOINT_RAMP_RATE", "Ramp rate [°C/s]"),
            ("SPAN", "PB span [°C]"),
            ("TS_S", "Sample period [s]"),
            ("TELEMETRY_MODE", "INFO/NORMAL/MPC"),
            ("EXPERIMENT_RUN_S", "Run time [s], None=until stop"),
            ("DIST_ENABLE", "Enable disturbance"),
            ("DIST_MODE", "STEP/PULSE"),
            ("DIST_MAG_PCT", "Disturbance OP [%]"),
            ("DIST_START_S", "Disturbance start [s]"),
            ("DIST_DURATION_S", "Pulse duration [s]"),
        ),
    ),
    "safety": (
        "Safety",
        (
            ("TEMP_CUTOFF_C", "High-temp cutoff [°C]"),
        ),
    ),
    "pid": (
        "PID settings",
        (
            ("PID_VARIANT", "PID/2DOF/FF_PID/GAIN_SCHED/SMITH_PI"),
            ("PID_AW_TYPE", "NONE/CLAMP/BACKCALC"),
            ("TUNING_RULE", "ZN1_*/CC_* or ZN2_*/TL_*"),
            ("PID_ALGORITHM", "IDEAL/PARALLEL/SERIES"),
            ("KP", "Kp"),
            ("KI", "Ki [1/s]"),
            ("KD", "Kd [s]"),
            ("KC", "Kc"),
            ("TI_S", "Ti [s]"),
            ("TD_S", "Td [s]"),
            ("DERIVATIVE_FILTER_ALPHA", "D filter alpha"),
            ("PID_INTEGRAL_LIMIT", "Integral clamp"),
            ("PID_AW_TRACKING_TIME_S", "Backcalc Tt [s]"),
            ("PID_BETA", "2DOF beta"),
            ("FF_MODE", "MANUAL or FOPDT_GAIN"),
            ("FF_GAIN_PCT_PER_C", "FF gain [%/°C]"),
            ("FF_BIAS_PCT", "FF bias [%]"),
            ("FF_AMBIENT_C", "Ambient [°C], None=auto"),
            ("GS_VARIABLE", "Gain schedule variable (PV/SP)"),
            ("GS_TABLE", "Schedule table"),
            ("TUNING_TARGET_C", "Relay target [°C]"),
            ("TUNING_BAND_C", "Relay half-band [°C]"),
            ("TUNING_CYCLES", "Relay cycles"),
        ),
    ),
    "onoff": (
        "ON/OFF settings",
        (
            ("ONOFF_HYST_C", "Half-band [°C]"),
            ("ONOFF_ON_PERCENT", "ON level [%]"),
            ("ONOFF_MIN_SWITCH_S", "Min switch [s]"),
        ),
    ),
    "fuzzy": (
        "Fuzzy settings",
        (
            ("FUZZY_E_SCALE_C", "Error scale [°C]"),
            ("FUZZY_DE_SCALE_C_PER_S", "dError scale [°C/s]"),
            ("FUZZY_DU_RATE_MAX", "du max [%/s]"),
            ("FUZZY_DE_FILTER_ALPHA", "dError alpha"),
            ("FUZZY_RULE_TABLE", "Rule matrix"),
        ),
    ),
    "mpc": (
        "MPC settings",
        (
            ("MPC_HORIZON_STEPS", "Horizon"),
            ("MPC_GRID_STEP_PCT", "Grid step [%]"),
            ("MPC_DU_MAX_PCT", "du max [%]"),
            ("MPC_LAMBDA_MOVE", "Move weight"),
            ("MPC_MAX_CANDIDATES", "Candidate limit"),
            ("MPC_OBSERVER_GAIN", "Observer gain [0..1]"),
        ),
    ),
    "tune": (
        "Tune settings",
        (
            ("TUNING_RULE", "ZN1_*/CC_* or ZN2_*/TL_*"),
            ("TUNING_TARGET_C", "Relay target [°C]"),
            ("TUNING_BAND_C", "Relay half-band [°C]"),
            ("TUNING_CYCLES", "Relay cycles"),
        ),
    ),
    "model": (
        "Model settings",
        (
            ("FOPDT_U1_PERCENT", "Step output [%]"),
            ("FOPDT_SMOOTH_N", "Smoothing window"),
            ("STEADY_WINDOW_S", "Steady window [s]"),
            ("STEADY_BAND_C", "Steady half-band [°C]"),
            ("MODEL_K", "Model K [°C/%]"),
            ("MODEL_TAU_S", "Model tau [s]"),
            ("MODEL_THETA_S", "Model theta [s]"),
            ("MODEL_U0_PCT", "Model u0 [%]"),
            ("MODEL_Y0", "Model y0 [°C]"),
            ("MODEL_METHOD", "Method label"),
            ("MODEL_RMSE", "RMSE hint"),
        ),
    ),
}

_MODE_TO_GROUP = {"PID": "pid", "ONOFF": "onoff", "FUZZY": "fuzzy", "MPC": "mpc"}

# Hidden in default/group params views to keep teaching-facing output concise.
# These remain visible in the full catalog view (`params all`).
_ADVANCED_PARAM_KEYS = {
    "DERIVATIVE_FILTER_ALPHA",
    "PID_INTEGRAL_LIMIT",
    "PID_AW_TRACKING_TIME_S",
    "GS_TABLE",
    "MPC_MAX_CANDIDATES",
}

_PID_ALWAYS_VISIBLE_KEYS = {"PID_VARIANT", "PID_AW_TYPE", "TUNING_RULE", "PID_ALGORITHM"}
_PID_PARALLEL_GAIN_KEYS = {"KP", "KI", "KD"}
_PID_IDEAL_GAIN_KEYS = {"KC", "TI_S", "TD_S"}
_PID_FF_KEYS = {"FF_MODE", "FF_GAIN_PCT_PER_C", "FF_BIAS_PCT", "FF_AMBIENT_C"}
_PID_GS_KEYS = {"GS_VARIABLE", "GS_TABLE"}
_PID_RELAY_TUNING_KEYS = {"TUNING_TARGET_C", "TUNING_BAND_C", "TUNING_CYCLES"}
_PID_SHAPING_KEYS = {"DERIVATIVE_FILTER_ALPHA", "PID_INTEGRAL_LIMIT"}

_COMMAND_SPECS = (
    ("status", "show session status"),
    ("control", "start control"),
    ("tune", "run PID tuning"),
    ("model", "run model ID"),
    ("monitor", "heater-off monitor"),
    ("params [active|all|core|safety|pid|onoff|fuzzy|mpc|tune|model]", "show params"),
    ("<PARAM> <VALUE>", "assign parameter"),
    ("<PARAM>=<VALUE>", "assign parameter"),
    ("pid report", "show PID forms"),
    ("check", "validate config"),
    ("help [commands|settings|<group>]", "show help"),
    ("exit", "leave prompt"),
)

def _parse_cli_value(name: str, raw: str):
    s = raw.strip()
    if len(s) >= 2 and ((s[0] == '"' and s[-1] == '"') or (s[0] == "'" and s[-1] == "'")):
        s = s[1:-1]

    sl = s.lower()
    if sl == "none":
        return None
    if sl == "true":
        return True
    if sl == "false":
        return False

    try:
        return int(s, 10)
    except Exception:
        pass

    try:
        return float(s)
    except Exception:
        pass

    if name in _STRING_PARAMS_UPPER:
        return s.upper()
    return s


def _format_param_value(v) -> str:
    if isinstance(v, (list, tuple)):
        n = len(v)
        if n > 6:
            head = ", ".join(repr(x) for x in v[:3])
            tail = ", ".join(repr(x) for x in v[-2:])
            return "[%s, ..., %s] (len=%d)" % (head, tail, n)
    s = repr(v)
    return s if len(s) <= 100 else (s[:97] + "...")


def _pid_param_visible_in_active_view(profile_mod, key: str) -> bool:
    algorithm = str(profile_mod.PID_ALGORITHM).upper()
    variant = str(profile_mod.PID_VARIANT).upper()
    aw_type = str(profile_mod.PID_AW_TYPE).upper()
    tuning_rule = str(profile_mod.TUNING_RULE).upper()

    if key in _PID_ALWAYS_VISIBLE_KEYS:
        return True
    if key in _PID_PARALLEL_GAIN_KEYS:
        return algorithm == "PARALLEL"
    if key in _PID_IDEAL_GAIN_KEYS:
        return algorithm in ("IDEAL", "SERIES")
    if key == "PID_BETA":
        return variant == "2DOF"
    if key in _PID_FF_KEYS:
        return variant == "FF_PID"
    if key in _PID_GS_KEYS:
        return variant == "GAIN_SCHED"
    if key == "PID_AW_TRACKING_TIME_S":
        return (variant == "PID") and (aw_type == "BACKCALC")
    if key in _PID_RELAY_TUNING_KEYS:
        return tuning_rule in getattr(profile_mod, "RELAY_TUNING_RULES", ())
    if key in _PID_SHAPING_KEYS:
        return variant in ("PID", "2DOF", "FF_PID", "GAIN_SCHED")
    return False


def _is_param_visible_in_active_view(profile_mod, group: str, key: str) -> bool:
    mode = str(profile_mod.CONTROL_MODE).upper()
    if group == "core":
        if key == "SETPOINT_RAMP_RATE":
            return str(profile_mod.SETPOINT_TYPE).upper() == "RAMP"
        if key in ("DIST_MODE", "DIST_MAG_PCT", "DIST_START_S", "DIST_DURATION_S"):
            if not bool(profile_mod.DIST_ENABLE):
                return False
            if key == "DIST_DURATION_S":
                return str(profile_mod.DIST_MODE).upper() == "PULSE"
        return True

    if group == "pid":
        if mode != "PID":
            return False
        return _pid_param_visible_in_active_view(profile_mod, key)

    if group == "onoff":
        return mode == "ONOFF"
    if group == "fuzzy":
        return mode == "FUZZY"
    if group == "mpc":
        return mode == "MPC"
    if group == "tune":
        return False
    if group == "model":
        return False
    if group == "safety":
        return True
    return True


def _print_group_params_view(profile_mod, group: str, include_advanced: bool = False, active_view: bool = False) -> None:
    meta = _PARAM_GROUPS.get(group)
    if meta is None:
        print("# ERROR: unknown params group '%s'" % group)
        return
    title, items = meta
    shown = 0
    print("# ========================================")
    print("# INFO: params group %s - %s" % (group.upper(), title))
    for key, desc in items:
        if (not include_advanced) and (key in _ADVANCED_PARAM_KEYS):
            continue
        if not hasattr(profile_mod, key):
            continue
        if active_view and (not _is_param_visible_in_active_view(profile_mod, group, key)):
            continue
        print("#   %-30s = %-28s  # %s" % (key, _format_param_value(getattr(profile_mod, key)), desc))
        shown += 1
    if shown <= 0:
        print("#   (no active parameters in this group)")


def _print_command_catalog() -> None:
    print("# INFO:")
    print("# ========================================")
    print("# commands:")
    for syntax, desc in _COMMAND_SPECS:
        print("#   %-58s # %s" % (syntax, desc))
    print("# during active run: stop, restart, help")
    print("# ========================================")


def show_runtime_params(profile_mod, scope: str = "active") -> None:
    s = str(scope or "active").strip().lower()
    if s in ("", "active", "current"):
        mode = str(profile_mod.CONTROL_MODE).upper()
        print("# RESULT:")
        print("# ========================================")
        print("# parameter report (active)")
        _print_group_params_view(profile_mod, "core", include_advanced=False, active_view=True)
        _print_group_params_view(profile_mod, "safety", include_advanced=False, active_view=True)
        _print_group_params_view(profile_mod, _MODE_TO_GROUP.get(mode, "pid"), include_advanced=False, active_view=True)
    elif s in ("all", "*"):
        print("# RESULT:")
        print("# ========================================")
        print("# parameter catalog (all)")
        for g in ("core", "safety", "pid", "onoff", "fuzzy", "mpc", "tune", "model"):
            _print_group_params_view(profile_mod, g, include_advanced=True, active_view=False)
    else:
        print("# RESULT:")
        print("# ========================================")
        print("# parameter report (%s)" % s)
        active_groups = ("core", "safety", "pid", "onoff", "fuzzy", "mpc")
        _print_group_params_view(
            profile_mod,
            s,
            include_advanced=False,
            active_view=(s in active_groups),
        )

    print("# ========================================")


def print_help(profile_mod, topic: str = "") -> None:
    t = str(topic or "").strip().lower()
    if t in ("", "commands", "cmd"):
        _print_command_catalog()
        print("# INFO: examples: params all, KP 6.5, TUNING_RULE CC_PID, pid report")
        return
    if t in ("settings", "groups", "params"):
        print("# INFO:")
        print("# ========================================")
        print("# settings groups:")
        print("#   %s" % ", ".join(_PARAM_GROUPS.keys()))
        print("# use: params <group>   or   help <group>")
        print("# note: params/group views show core teaching knobs; use 'params all' for advanced engineering options")
        print("# ========================================")
        return
    if t in _PARAM_GROUPS:
        _print_group_params_view(profile_mod, t, include_advanced=False, active_view=False)
        return
    print("# ERROR: unknown help topic '%s' (use: help settings)" % t)


def apply_runtime_param(profile_mod, name: str, raw_value: str) -> None:
    key = str(name).strip().upper()
    if not key:
        print("# ERROR: usage: <PARAM> <VALUE>  or  <PARAM>=<VALUE>")
        return
    if not hasattr(profile_mod, key):
        print("# ERROR: unknown parameter: %s" % key)
        return

    old_value = getattr(profile_mod, key)
    if key in _COMPLEX_ASSIGN_BLOCKLIST:
        print("# ERROR: assign failed: %s is structured; edit config.py directly" % key)
        return
    try:
        value = _parse_cli_value(key, raw_value)
    except Exception as ex:
        print("# ERROR: parse failed for %s: %s" % (key, ex))
        return

    setattr(profile_mod, key, value)
    try:
        profile_mod.validate()
    except Exception as ex:
        # Rollback so runtime config never enters an invalid state.
        setattr(profile_mod, key, old_value)
        print("# ERROR: assign failed: %s -> %s (%s)" % (key, value, ex))
        _print_assign_dependency_hint(profile_mod, key, value, str(ex))
        return
    print("# RESULT: %s = %r" % (key, value))


def _print_assign_dependency_hint(profile_mod, key: str, value, err_text: str) -> None:
    """Print actionable dependency hints for common validation traps."""
    e = str(err_text)

    if ("PID_ALGORITHM='SERIES' requires KC != 0" in e) and (str(profile_mod.PID_ALGORITHM).upper() == "SERIES"):
        print("# INFO: hint: with PID_ALGORITHM=SERIES, either:")
        print("#   KC non-zero")
        print("#   or TI_S=0 and TD_S=0 for pure-zero action")


def _fmt_pid_num(v, unit: str = "") -> str:
    if v is None:
        return "disabled"
    try:
        x = float(v)
    except Exception:
        return str(v)
    suffix = unit if unit else ""
    return "%.6g%s" % (x, suffix)


def pid_report(profile_mod) -> None:
    """Teaching-focused PID parameter report."""
    try:
        from control import pid_descriptor_from_profile, pid_selection_from_profile

        pid_sel = pid_selection_from_profile(profile_mod)
        desc = pid_descriptor_from_profile(profile_mod)
    except Exception as ex:
        print("# ERROR: PID report failed: %s" % ex)
        return

    variant = str(pid_sel.get("variant", "PID")).upper()
    aw_type = str(pid_sel.get("aw_type", "CLAMP")).upper()
    algorithm = str(desc.get("algorithm", "PARALLEL")).upper()
    configured = desc.get("configured", {})
    p = desc.get("parallel", {})
    ideal = desc.get("ideal", {})
    pb = desc.get("forms", {}).get("PB", {})

    print("# RESULT: PID parameter report")
    if variant == "PID":
        print("#   family        : PID  (AW=%s)" % aw_type)
    else:
        print("#   family        : %s" % variant)
    print("#   algorithm     : %s" % algorithm)
    if algorithm == "PARALLEL":
        print("#   configured (K): Kp=%s  Ki=%s  Kd=%s"
              % (_fmt_pid_num(configured.get("KP")), _fmt_pid_num(configured.get("KI")), _fmt_pid_num(configured.get("KD"))))
    else:
        print("#   configured (IDEAL): Kc=%s  Ti=%s  Td=%s"
              % (_fmt_pid_num(configured.get("KC")),
                 _fmt_pid_num(configured.get("TI_S"), "s"),
                 _fmt_pid_num(configured.get("TD_S"), "s")))
    print("#   K view        : Kp=%s  Ki=%s  Kd=%s"
          % (_fmt_pid_num(p.get("KP")), _fmt_pid_num(p.get("KI")), _fmt_pid_num(p.get("KD"))))
    print("#   IDEAL view    : Kc=%s  Ti=%s  Td=%s"
          % (_fmt_pid_num(ideal.get("KC")), _fmt_pid_num(ideal.get("TI_S"), "s"), _fmt_pid_num(ideal.get("TD_S"), "s")))
    print("#   PB-equivalent : Pb=%s (span=%s)  Ti=%s  Td=%s"
          % (_fmt_pid_num(pb.get("PB"), "%"),
             _fmt_pid_num(pb.get("SPAN"), "°C"),
             _fmt_pid_num(pb.get("TI_S"), "s"),
             _fmt_pid_num(pb.get("TD_S"), "s")))

    if algorithm == "SERIES":
        eff = desc.get("effective_ideal", {})
        print("#   SERIES eqv IDEAL: Kc_eff=%s  Ti_eff=%s  Td_eff=%s"
              % (_fmt_pid_num(eff.get("KC")),
                 _fmt_pid_num(eff.get("TI_S"), "s"),
                 _fmt_pid_num(eff.get("TD_S"), "s")))

        ti_s = ideal.get("TI_S")
        td_s = ideal.get("TD_S")
        if (ti_s is not None) and (td_s is not None):
            try:
                ti = float(ti_s)
                td = float(td_s)
                if ti > 0.0:
                    if td >= ti:
                        print("# WARNING: SERIES with Td >= Ti can behave strongly interacting/unintuitive")
                    elif td >= (0.8 * ti):
                        print("# WARNING: SERIES with Td close to Ti can feel highly interacting; tune cautiously")
            except Exception:
                pass


def wait_for_run_command(profile_mod) -> str:
    """REPL command loop before run start."""
    print("# READY: run [control|tune|model|monitor]  inspect [status|params|check]  set [PARAM VALUE]")
    while True:
        try:
            line = input("lab> ").strip()
        except EOFError:
            yield_cpu()
            continue
        except KeyboardInterrupt:
            print("\n# INFO: keyboard interrupt at prompt")
            return "exit"

        cmd = line.lower()
        if cmd in ("status", "control", "tune", "model", "monitor", "exit"):
            return cmd
        if (cmd == "params") or cmd.startswith("params "):
            parts = line.split(None, 1)
            show_runtime_params(profile_mod, parts[1] if len(parts) > 1 else "active")
            continue
        if cmd == "check":
            try:
                profile_mod.validate()
                print("# RESULT: check passed (config is valid)")
            except Exception as ex:
                print("# ERROR: check failed: %s" % ex)
            continue
        if cmd == "pid report":
            pid_report(profile_mod)
            continue
        if (cmd == "help") or cmd.startswith("help "):
            parts = line.split(None, 1)
            print_help(profile_mod, parts[1] if len(parts) > 1 else "")
            continue
        if cmd == "":
            continue
        if cmd.startswith("set "):
            print("# ERROR: command 'set' was removed; use '<PARAM> <VALUE>' or '<PARAM>=<VALUE>'")
            continue
        if "=" in line:
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            if key and value:
                apply_runtime_param(profile_mod, key, value)
                continue
        parts = line.split(None, 1)
        if len(parts) == 2:
            apply_runtime_param(profile_mod, parts[0], parts[1])
            continue
        print("# ERROR: unknown command '%s' (type 'help')" % cmd)


def poll_command_nonblocking():
    """Read one line command from stdin if available, else return None."""
    global _poll_err_count
    if _select is None:
        return None
    try:
        ready, _, _ = _select.select([sys.stdin], [], [], 0)
        if not ready:
            return None
        line = sys.stdin.readline()
        if line is None:
            return None
        cmd = str(line).strip().lower()
        return cmd if cmd else None
    except Exception as ex:
        _poll_err_count += 1
        if _poll_err_count <= 3:
            print("# WARNING: stdin poll error (%d): %s" % (_poll_err_count, ex))
        return None
