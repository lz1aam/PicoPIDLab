#!/usr/bin/env python3
"""
PicoPID Lab PC-side experiment runner.

Usage examples:
  python3 runner/lab.py                                       # interactive catalog from config experiments

Dependencies:
  pyserial, numpy, matplotlib
"""

from __future__ import annotations

import csv
import itertools
import json
import math
import os
import re
import select
import sys
import time
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
import serial
import serial.tools.list_ports

import matplotlib

# Keep plotting non-blocking and portable by default.
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import yaml as _yaml
except Exception:
    _yaml = None



TS_S_DEFAULT = 0.25
BAUD_DEFAULT = 115200
SERIAL_TIMEOUT_S = 0.05
PROMPT_TOKEN = "lab>"
REPL_TOKEN = ">>>"
DONE_TOKEN = "# FINISH: done"
DONE_TOKENS = (DONE_TOKEN,)
READY_TOKEN = "# READY:"
FOPDT_SUMMARY_RE = re.compile(r"fopdt:K=([0-9.eE+-]+)\s+tau=([0-9.eE+-]+)\s+theta=([0-9.eE+-]+)\s+rmse=([0-9.eE+-]+)")
FOPDT_APPLIED_RE = re.compile(
    r"applied method=.*?:\s*K=([0-9.eE+-]+)\s*.*?\s+tau=([0-9.eE+-]+)\s*s\s+theta=([0-9.eE+-]+)\s*s\s+rmse=([0-9.eE+-]+)",
    re.IGNORECASE,
)
FOPDT_STEP_RE = re.compile(r"step:\s*([0-9.eE+-]+)%\s*->\s*([0-9.eE+-]+)%", re.IGNORECASE)
FOPDT_Y0_RE = re.compile(r"initial steady temperature y0=([0-9.eE+-]+)", re.IGNORECASE)
FOPDT_Y1_RE = re.compile(r"final steady temperature y1=([0-9.eE+-]+)", re.IGNORECASE)
FOPDT_METHOD_RE = re.compile(r"applied method=([A-Za-z0-9_]+)", re.IGNORECASE)
TUNING_GAINS_RE = re.compile(r"TUNING: runtime gains updated -> Kp=([0-9.eE+-]+)\s+Ki=([0-9.eE+-]+)\s+Kd=([0-9.eE+-]+)")
TUNING_PARALLEL_FORM_RE = re.compile(
    r"Applied PARALLEL form:\s*Kp=([^\s]+)\s+Ki=([^\s]+)\s+Kd=([^\s]+)",
    re.IGNORECASE,
)
TUNING_RELAY_METRICS_RE = re.compile(
    r"relay metrics:\s*A=([0-9.eE+-]+).*d_eff=([0-9.eE+-]+).*Ku=([0-9.eE+-]+)\s+Pu=([0-9.eE+-]+)\s*s\s+PV_pp=([0-9.eE+-]+)",
    re.IGNORECASE,
)
TUNING_CYCLES_RE = re.compile(r"cycles_used=([0-9]+)", re.IGNORECASE)
TUNING_APPLIED_SET_RE = re.compile(r"Applied set:\s*(.+)$", re.IGNORECASE)
_TELEM_MAIN_RE = re.compile(r"\bPV:\s*([0-9.eE+-]+)\s+SP:\s*([0-9.eE+-]+)\s+OP:\s*([0-9.eE+-]+)\b", re.IGNORECASE)
_TELEM_YH_RE = re.compile(r"\bYH:\s*([0-9.eE+-]+)\b", re.IGNORECASE)
_TELEM_YP_RE = re.compile(r"\bYP:\s*([0-9.eE+-]+)\b", re.IGNORECASE)
_TELEM_T_RE = re.compile(r"\bT:\s*([0-9.eE+-]+)\b", re.IGNORECASE)
_PARAM_ASSIGN_RE = re.compile(r"\b([A-Z][A-Z0-9_]{1,63})\s*=")
_CONFIG_ASSIGN_RE = re.compile(r"^\s*([A-Z][A-Z0-9_]{1,63})\s*=", re.MULTILINE)

# Firmware command-response tokens (current unified style).
CHECK_OK_TOKENS = ("# RESULT: check passed (config is valid)",)
CHECK_ERR_TOKENS = ("# ERROR: check failed:",)
ASSIGN_OK_TOKEN = "# RESULT: "
ASSIGN_ERR_TOKENS = ("# ERROR: assign failed:", "# ERROR:")
UNKNOWN_CMD_TOKENS = ("# ERROR: unknown command",)

# TODO(runner):
# - Add firmware/runner schema preflight check (fail early with actionable diff
#   when an experiment parameter is unsupported by current firmware build).

MODEL_K_C_PER_PCT = 0.467
COLLECT_LOOP_SLEEP_IDLE_S = 0.02
COLLECT_LOOP_SLEEP_ACTIVE_S = 0.002
_APP_ROOT = Path(__file__).resolve().parent
_REPO_ROOT = _APP_ROOT.parent
_LAST_OUTPUT_SOURCE = None  # "lab" | "fw" | None


def _emit_lab(msg: str = "") -> None:
    global _LAST_OUTPUT_SOURCE
    text = str(msg)
    if text == "":
        print("")
        return
    if _LAST_OUTPUT_SOURCE == "fw":
        print("")
    if text.startswith("LAB:"):
        print(text)
    else:
        print("LAB: %s" % text)
    _LAST_OUTPUT_SOURCE = "lab"


def _emit_fw(msg: str) -> None:
    global _LAST_OUTPUT_SOURCE
    text = str(msg)
    if _LAST_OUTPUT_SOURCE == "lab":
        print("")
    print(text)
    _LAST_OUTPUT_SOURCE = "fw"


def _default_config_path() -> Path:
    base = _APP_ROOT
    p_yaml = base / "lab.yaml"
    return p_yaml


def _safe_float(x, default=None):
    try:
        return float(x)
    except Exception:
        return default


@dataclass
class ExperimentSpec:
    number: int
    exp_id: str
    shortname: str
    description: str
    base_params: Dict[str, object]
    kind: str = "standard"  # standard | fopdt | tuning | sweep
    plot_cfg: Dict[str, object] = field(default_factory=dict)
    metrics_cfg: Dict[str, object] = field(default_factory=dict)
    sweep_cfg: Dict[str, object] = field(default_factory=dict)
    completion_cfg: Dict[str, object] = field(default_factory=dict)
    run_command: str = ""


@dataclass
class RunResult:
    telemetry: np.ndarray
    raw_lines: List[str]
    metrics: Dict[str, object]
    params_sent: Dict[str, object]
    run_dir: Path


@dataclass
class RunnerConfig:
    port: str
    baud: int
    out_root: Path
    timeout_s: float
    live: bool
    settle_band_c: float
    verbose: bool
    live_refresh_s: float = 0.15  # 6-7 Hz to avoid heavy redraw at Ts=0.25 s
    plot_cfg: Dict[str, object] = field(default_factory=dict)
    metrics_cfg: Dict[str, object] = field(default_factory=dict)


class LivePlotter:
    def __init__(
        self,
        enabled: bool,
        refresh_s: float = 0.15,
        plot_cfg: Optional[Dict[str, object]] = None,
        title: Optional[str] = None,
        expect_mpc_estimates: bool = False,
    ):
        self.enabled = bool(enabled)
        self.refresh_s = max(0.05, float(refresh_s))
        self.plot_cfg = plot_cfg or {}
        self.title = str(title).strip() if title is not None else ""
        self._last_draw = 0.0
        self._fig = None
        self._ax = None
        self._pv_line = None
        self._sp_line = None
        self._op_line = None
        self._yh_line = None
        self._yp_line = None
        self._mpc_legend_on = bool(expect_mpc_estimates)
        if not self.enabled:
            return
        try:
            # Try switching to an interactive backend if available.
            self._enable_interactive_backend()
            plt.ion()
            self._fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            self._ax = (ax1, ax2)
            self._pv_line, = ax1.plot([], [], label="PV")
            self._sp_line, = ax1.plot([], [], label="SP")
            yh_label = "Yh (estimated)" if self._mpc_legend_on else "_nolegend_"
            yp_label = "Yp (horizon-end)" if self._mpc_legend_on else "_nolegend_"
            self._yh_line, = ax1.plot([], [], label=yh_label, linestyle="--", color="tab:green")
            self._yp_line, = ax1.plot([], [], label=yp_label, linestyle="-.", color="tab:red")
            self._op_line, = ax2.plot([], [], label="OP", color="tab:orange")
            ax1.set_ylabel("Temperature [°C]")
            ax2.set_ylabel("Output [%]")
            ax2.set_xlabel("time [s]")
            if self.title:
                self._fig.suptitle(self.title)
            ax1.grid(True)
            ax2.grid(True)
            ax1.legend(loc="best")
            ax2.legend(loc="best")
            # Apply configured view limits immediately (before first samples).
            apply_axis_config(ax1, ax2, self.plot_cfg)
            try:
                self._fig.canvas.draw_idle()
                self._fig.canvas.flush_events()
            except Exception:
                pass
        except Exception as ex:
            self.enabled = False
            self._fig = None
            _emit_lab("warning: live plot disabled (%s)" % ex)

    @staticmethod
    def _enable_interactive_backend():
        # Prefer explicit env override first, then common interactive backends.
        env = os.getenv("PICOPID_LIVE_BACKENDS", "")
        if env.strip():
            candidates = [x.strip() for x in env.split(",") if x.strip()]
        else:
            candidates = ["TkAgg", "QtAgg", "Qt5Agg", "GTK3Agg", "MacOSX"]

        current = matplotlib.get_backend()
        if str(current).lower() != "agg":
            return

        last_ex = None
        for name in candidates:
            try:
                plt.switch_backend(name)
                return
            except Exception as ex:
                last_ex = ex
        if last_ex is not None:
            raise RuntimeError("no interactive backend available; last error: %s" % last_ex)

    def update(
        self,
        t: np.ndarray,
        pv: np.ndarray,
        sp: np.ndarray,
        op: np.ndarray,
        y_hat: Optional[np.ndarray] = None,
        y_pred: Optional[np.ndarray] = None,
    ):
        if not self.enabled or self._fig is None:
            return
        now = time.monotonic()
        if (now - self._last_draw) < self.refresh_s:
            return
        self._last_draw = now
        try:
            self._pv_line.set_data(t, pv)
            self._sp_line.set_data(t, sp)
            has_yh = (y_hat is not None) and np.any(np.isfinite(y_hat))
            has_yp = (y_pred is not None) and np.any(np.isfinite(y_pred))
            if has_yh:
                self._yh_line.set_data(t, y_hat)
            else:
                self._yh_line.set_data([], [])
            if has_yp:
                self._yp_line.set_data(t, y_pred)
            else:
                self._yp_line.set_data([], [])
            self._op_line.set_data(t, op)
            mpc_on = bool(has_yh or has_yp)
            if mpc_on != self._mpc_legend_on:
                self._mpc_legend_on = mpc_on
                if mpc_on:
                    self._yh_line.set_label("Yh (estimated)")
                    self._yp_line.set_label("Yp (horizon-end)")
                else:
                    self._yh_line.set_label("_nolegend_")
                    self._yp_line.set_label("_nolegend_")
                self._ax[0].legend(loc="best")
            c = self.plot_cfg or {}
            has_xlim = isinstance(c.get("xlim"), (list, tuple)) and len(c.get("xlim")) == 2
            has_t_ylim = isinstance(c.get("temperature_ylim"), (list, tuple)) and len(c.get("temperature_ylim")) == 2
            has_o_ylim = isinstance(c.get("output_ylim"), (list, tuple)) and len(c.get("output_ylim")) == 2

            ax1, ax2 = self._ax
            # Only autoscale dimensions that are not fixed by config.
            if has_xlim and has_t_ylim:
                ax1.relim()
                ax1.autoscale_view(scalex=False, scaley=False)
            elif has_xlim and (not has_t_ylim):
                ax1.relim()
                ax1.autoscale_view(scalex=False, scaley=True)
            elif (not has_xlim) and has_t_ylim:
                ax1.relim()
                ax1.autoscale_view(scalex=True, scaley=False)
            else:
                ax1.relim()
                ax1.autoscale_view()

            if has_xlim and has_o_ylim:
                ax2.relim()
                ax2.autoscale_view(scalex=False, scaley=False)
            elif has_xlim and (not has_o_ylim):
                ax2.relim()
                ax2.autoscale_view(scalex=False, scaley=True)
            elif (not has_xlim) and has_o_ylim:
                ax2.relim()
                ax2.autoscale_view(scalex=True, scaley=False)
            else:
                ax2.relim()
                ax2.autoscale_view()
            if (not has_xlim) and (len(t) > 0):
                # Unified live policy: when x-limits are not fixed by config,
                # keep a stable runtime axis that always starts at 0 s and grows.
                t_end = float(t[-1])
                right = max(10.0, t_end + 0.5)
                ax1.set_xlim(0.0, right)
                ax2.set_xlim(0.0, right)
            apply_axis_config(self._ax[0], self._ax[1], self.plot_cfg)
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception:
            self.enabled = False

    def close(self, keep_open: bool = False):
        if self._fig is not None:
            try:
                if keep_open and self.enabled:
                    # Keep window open for comparison without blocking the CLI.
                    try:
                        self._fig.canvas.draw_idle()
                        self._fig.canvas.flush_events()
                    except Exception:
                        pass
                    self._fig = None
                    self._ax = None
                    return
                plt.close(self._fig)
            except Exception:
                pass
            finally:
                self._fig = None
                self._ax = None


def _build_metrics_lines(metrics: Dict[str, object], metrics_cfg: Optional[Dict[str, object]] = None) -> List[str]:
    lines: List[str] = []
    fopdt = metrics.get("fopdt")
    if isinstance(fopdt, dict):
        k = _safe_float(fopdt.get("K"), float("nan"))
        tau = _safe_float(fopdt.get("tau_s"), float("nan"))
        theta = _safe_float(fopdt.get("theta_s"), float("nan"))
        rmse = _safe_float(fopdt.get("rmse"), float("nan"))
        lines.append("FOPDT identification")
        lines.append(f"{'K':24s} {k:.6f} °C/%" if math.isfinite(k) else f"{'K':24s} n/a")
        lines.append(f"{'tau':24s} {tau:.2f} s" if math.isfinite(tau) else f"{'tau':24s} n/a")
        lines.append(f"{'theta':24s} {theta:.2f} s" if math.isfinite(theta) else f"{'theta':24s} n/a")
        lines.append(f"{'RMSE':24s} {rmse:.3f} °C" if math.isfinite(rmse) else f"{'RMSE':24s} n/a")
        dur = _safe_float(metrics.get("duration_s"), float("nan"))
        smp = metrics.get("samples")
        if math.isfinite(dur):
            lines.append(f"{'duration':24s} {dur:.1f} s")
        if isinstance(smp, int):
            lines.append(f"{'samples':24s} {smp}")
        return lines

    keys = _metric_keys_from_cfg(
        metrics_cfg,
        "display",
        ["IAE", "ISE", "ITAE", "steady_state_error_c", "rise_time_s", "settling_time_s", "overshoot_pct", "op_rms_pct"],
    )
    for k in keys:
        if k not in metrics:
            continue
        v = metrics.get(k)
        if v is None:
            txt = "n/a"
        elif isinstance(v, (int, float)):
            unit = _METRIC_UNITS.get(k, "")
            txt = f"{float(v):.3f}" + (f" {unit}" if unit else "")
        else:
            txt = str(v)
        label = _METRIC_LABELS.get(k, k)
        lines.append(f"{label:24s} {txt}")
    return lines


class LiveMetricsPanel:
    def __init__(self, enabled: bool, refresh_s: float, metrics_cfg: Optional[Dict[str, object]], settle_band_c: float, title: str = ""):
        self.enabled = bool(enabled)
        self.refresh_s = max(0.1, float(refresh_s))
        self.metrics_cfg = metrics_cfg or {}
        self.settle_band_c = float(settle_band_c)
        self.title = str(title).strip()
        self._fig = None
        self._ax = None
        self._text = None
        self._last_draw = 0.0
        if not self.enabled:
            return
        try:
            LivePlotter._enable_interactive_backend()
            plt.ion()
            self._fig = plt.figure(figsize=(5.2, 3.0), dpi=130)
            self._ax = self._fig.add_subplot(111)
            self._ax.axis("off")
            self._ax.set_title("Metrics" + (f" - {self.title}" if self.title else ""), loc="left")
            self._text = self._ax.text(
                0.01,
                0.97,
                "waiting for telemetry...",
                va="top",
                ha="left",
                family="monospace",
                fontsize=10,
                transform=self._ax.transAxes,
            )
            try:
                self._fig.canvas.draw_idle()
                self._fig.canvas.flush_events()
            except Exception:
                pass
        except Exception as ex:
            self.enabled = False
            self._fig = None
            _emit_lab("warning: live metrics disabled (%s)" % ex)

    def update(self, telemetry: np.ndarray):
        if (not self.enabled) or (self._fig is None) or (telemetry.size == 0):
            return
        now = time.monotonic()
        if (now - self._last_draw) < self.refresh_s:
            return
        self._last_draw = now
        try:
            metrics_now = compute_metrics(telemetry, self.settle_band_c)
            lines = _build_metrics_lines(metrics_now, self.metrics_cfg)
            if not lines:
                lines = ["no metrics selected"]
            self._text.set_text("\n".join(lines))
            w, h = _metrics_figure_size_from_lines(lines, min_w=5.0)
            self._fig.set_size_inches(w, h, forward=True)
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception:
            self.enabled = False

    def close(self, keep_open: bool = False):
        if self._fig is None:
            return
        try:
            if keep_open and self.enabled:
                # Keep metrics window open for comparison without blocking.
                try:
                    self._fig.canvas.draw_idle()
                    self._fig.canvas.flush_events()
                except Exception:
                    pass
                self._fig = None
                self._ax = None
                self._text = None
                return
            plt.close(self._fig)
        except Exception:
            pass
        finally:
            self._fig = None
            self._ax = None
            self._text = None


def _metrics_figure_size_from_lines(lines: List[str], min_w: float = 4.0) -> Tuple[float, float]:
    if not lines:
        return (min_w, 2.0)
    max_chars = max(len(s) for s in lines)
    width = max(min_w, min(7.2, 0.09 * float(max_chars) + 2.0))
    height = max(2.0, min(8.0, 0.30 * float(len(lines)) + 1.0))
    return (width, height)


def parse_telemetry_line(line: str):
    # Preferred firmware runtime format: PV:xx.x SP:xx.x OP:xx.x [YH:.. YP:..]
    # Parse case-insensitively and tolerate extra leading/trailing text.
    m = _TELEM_MAIN_RE.search(line)
    if m is not None:
        try:
            pv = float(m.group(1))
            sp = float(m.group(2))
            op = float(m.group(3))
            mh = _TELEM_YH_RE.search(line)
            mp = _TELEM_YP_RE.search(line)
            mt = _TELEM_T_RE.search(line)
            y_hat = float(mh.group(1)) if mh is not None else float("nan")
            y_pred = float(mp.group(1)) if mp is not None else float("nan")
            t_s = float(mt.group(1)) if mt is not None else float("nan")
            return (t_s, pv, sp, op, y_hat, y_pred)
        except Exception:
            return None
    # Legacy CSV-like runtime line
    parts = line.split(",")
    if len(parts) == 5 and parts[0] == "DATA":
        try:
            return (float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), float("nan"), float("nan"))
        except Exception:
            return None
    return None


def list_ports() -> List[dict]:
    rows = []
    for p in serial.tools.list_ports.comports():
        rows.append({
            "device": p.device,
            "description": p.description or "n/a",
            "vid": getattr(p, "vid", None),
            "pid": getattr(p, "pid", None),
        })
    return rows


def score_port(row: dict) -> int:
    d = (row.get("description") or "").lower()
    dev = (row.get("device") or "").lower()
    vid = row.get("vid")
    pid = row.get("pid")
    s = 0
    if vid == 0x2E8A:
        s += 80
        if pid == 0x0005:
            s += 20
    if "pico" in d or "rp2" in d or "board cdc" in d:
        s += 50
    if "ttyacm" in dev or "usbmodem" in dev:
        s += 30
    return s


def autodetect_port() -> Optional[str]:
    ports = sorted(list_ports(), key=score_port, reverse=True)
    if not ports:
        return None
    if score_port(ports[0]) <= 0:
        return None
    return ports[0]["device"]


class PicoSession:
    def __init__(self, port: str, baud: int, timeout_s: float, verbose: bool = False):
        self.port = port
        self.baud = int(baud)
        self.timeout_s = float(timeout_s)
        self.verbose = bool(verbose)
        self.ser = serial.Serial(port, self.baud, timeout=SERIAL_TIMEOUT_S)
        self._rx_buf = ""
        self._param_keys_cache: Optional[set] = None
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def clear_pending_io(self):
        """Drop pending serial input/output and clear local RX buffer."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        try:
            self.ser.reset_output_buffer()
        except Exception:
            pass
        self._rx_buf = ""

    def _log(self, msg: str):
        if self.verbose:
            print(msg)

    def _write_line(self, line: str):
        self._log(f"> {line}")
        # CRLF is generally safest across USB-CDC terminal implementations.
        self.ser.write((line + "\r\n").encode("utf-8"))
        self.ser.flush()

    def _write_ctrl_c(self):
        self.ser.write(b"\x03")
        self.ser.flush()

    def _write_ctrl_d(self):
        self.ser.write(b"\x04")
        self.ser.flush()

    def _read_text_chunk(self) -> str:
        n = self.ser.in_waiting
        raw = self.ser.read(n if n > 0 else 1)
        if not raw:
            return ""
        return raw.decode("utf-8", errors="replace")

    def _consume_prompt_token(self):
        idx = self._rx_buf.rfind(PROMPT_TOKEN)
        if idx >= 0:
            self._rx_buf = self._rx_buf[idx + len(PROMPT_TOKEN) :].lstrip()

    def _saw_repl_prompt(self, lines: Sequence[str]) -> bool:
        if REPL_TOKEN in self._rx_buf:
            return True
        for l in lines[-20:]:
            if REPL_TOKEN in l:
                return True
        return False

    def _poll_lines(self, raw_lines: List[str], line_cb: Optional[Callable[[str], None]] = None) -> str:
        chunk = self._read_text_chunk()
        if not chunk:
            return ""
        # Normalize all line endings to '\n' to avoid CR/LF edge cases.
        norm = chunk.replace("\r\n", "\n").replace("\r", "\n")
        self._rx_buf += norm
        while True:
            idx = self._rx_buf.find("\n")
            if idx < 0:
                break
            line = self._rx_buf[:idx].strip()
            self._rx_buf = self._rx_buf[idx + 1 :]
            if line:
                raw_lines.append(line)
                if self.verbose:
                    print(f"< {line}")
                if line_cb is not None:
                    line_cb(line)
        return norm

    def _wait_for_tokens(
        self,
        timeout_s: float,
        tokens: Sequence[str],
        raw_lines: Optional[List[str]] = None,
    ) -> Tuple[bool, List[str], str]:
        lines = [] if raw_lines is None else raw_lines
        deadline = time.monotonic() + max(0.1, timeout_s)
        tok_list = [str(t) for t in tokens if str(t)]
        while time.monotonic() < deadline:
            before = len(lines)
            chunk = self._poll_lines(lines)
            scan_text = self._rx_buf
            if chunk:
                scan_text += chunk
            if PROMPT_TOKEN in tok_list and PROMPT_TOKEN in scan_text:
                self._consume_prompt_token()
                return True, lines, PROMPT_TOKEN
            if chunk:
                for line in lines[before:]:
                    for tok in tok_list:
                        if tok == PROMPT_TOKEN:
                            continue
                        if tok in line:
                            return True, lines, tok
                # Also search in the rolling buffer because tokens can be split across reads.
                for tok in tok_list:
                    if tok != PROMPT_TOKEN and tok in scan_text:
                        return True, lines, tok
            else:
                time.sleep(0.01)
        return False, lines, ""

    def wait_for_prompt(self, timeout_s: float, raw_lines: Optional[List[str]] = None) -> Tuple[bool, List[str]]:
        lines = [] if raw_lines is None else raw_lines
        deadline = time.monotonic() + max(0.1, timeout_s)
        saw_telemetry = False
        sent_stop = False
        while time.monotonic() < deadline:
            if PROMPT_TOKEN in self._rx_buf:
                self._consume_prompt_token()
                return True, lines
            chunk = self._poll_lines(lines)
            if chunk:
                if PROMPT_TOKEN in chunk or PROMPT_TOKEN in self._rx_buf:
                    self._consume_prompt_token()
                    return True, lines
                # Running loop -> ask it to stop once.
                for l in lines[-5:]:
                    if parse_telemetry_line(l) is not None:
                        saw_telemetry = True
                        break
                if saw_telemetry and not sent_stop:
                    self._write_line("stop")
                    sent_stop = True
            else:
                time.sleep(0.01)
        return False, lines

    def sync_prompt(self, timeout_s: float) -> None:
        # Robust command-mode handshake without strict dependence on visible prompt token:
        # command mode always answers "check" with "# CHECK ...",
        # and may also emit READY token before explicit prompt token.
        lines: List[str] = []
        deadline = time.monotonic() + max(2.0, float(timeout_s))
        per_try = min(2.5, max(0.8, 0.28 * timeout_s))
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            self._log(f"[sync] attempt {attempt}")
            # Drain any pending boot/REPL output first and react to state before probing.
            self._poll_lines(lines)
            if any(l.startswith(READY_TOKEN) for l in lines[-20:]):
                return
            if self._saw_repl_prompt(lines):
                self._write_ctrl_d()  # Soft reboot -> run main.py
                ok_ready, lines, _ = self._wait_for_tokens(max(3.0, timeout_s), (READY_TOKEN, PROMPT_TOKEN), lines)
                if ok_ready:
                    return
                continue

            self._write_line("check")
            ok, lines, tok = self._wait_for_tokens(
                per_try,
                CHECK_OK_TOKENS + CHECK_ERR_TOKENS + (READY_TOKEN,) + UNKNOWN_CMD_TOKENS + (PROMPT_TOKEN,),
                lines,
            )
            if ok and tok in CHECK_OK_TOKENS + CHECK_ERR_TOKENS:
                return
            if ok and tok == READY_TOKEN:
                return
            if ok and tok == PROMPT_TOKEN:
                # Prompt is alive; request explicit check response once more.
                self._write_line("check")
                ok2, lines, tok2 = self._wait_for_tokens(
                    per_try,
                    CHECK_OK_TOKENS + CHECK_ERR_TOKENS + (READY_TOKEN,) + UNKNOWN_CMD_TOKENS,
                    lines,
                )
                if ok2 and tok2 in CHECK_OK_TOKENS + CHECK_ERR_TOKENS + (READY_TOKEN,):
                    return
            if self._saw_repl_prompt(lines):
                self._write_ctrl_d()  # Ctrl+D soft reboot -> run main.py
                ok3, lines, _ = self._wait_for_tokens(max(3.0, timeout_s), (PROMPT_TOKEN, READY_TOKEN), lines)
                if ok3:
                    return
                continue
            self._write_line("stop")
            self._wait_for_tokens(min(1.2, per_try), (READY_TOKEN, PROMPT_TOKEN), lines)
            self._write_line("")
            self._wait_for_tokens(min(0.8, per_try), (PROMPT_TOKEN,), lines)
            # Last resort nudge: interrupt any blocked state.
            self._write_ctrl_c()
            self._wait_for_tokens(min(1.0, per_try), (REPL_TOKEN, PROMPT_TOKEN, READY_TOKEN), lines)
        tail = "\n".join(lines[-20:])
        raise RuntimeError(f"Could not sync to firmware command mode. Last lines:\n{tail}")

    def send_cmd_wait_prompt(self, cmd: str, timeout_s: float, retry_once: bool = True) -> List[str]:
        lines: List[str] = []
        # Drop stale prompt token from a previous transaction before issuing a new command.
        self._consume_prompt_token()
        self._write_line(cmd)
        ok, lines = self.wait_for_prompt(timeout_s, lines)
        if not ok:
            # Recovery attempt: request a fresh prompt once before failing.
            self._write_line("")
            ok2, lines = self.wait_for_prompt(2.5, lines)
            if ok2:
                return lines
            if retry_once:
                self.sync_prompt(max(4.0, timeout_s))
                return self.send_cmd_wait_prompt(cmd, timeout_s, retry_once=False)
            tail = "\n".join(lines[-20:])
            raise RuntimeError(f"Timeout waiting prompt after '{cmd}'. Last lines:\n{tail}")
        return lines

    def send_cmd_stream_wait_prompt(
        self,
        cmd: str,
        timeout_s: float,
        line_cb: Optional[Callable[[str], None]] = None,
    ) -> List[str]:
        lines: List[str] = []
        # Drop stale prompt token from a previous transaction before issuing a new command.
        self._consume_prompt_token()
        # Drop stale buffered text from previous sync/probe transactions.
        self._rx_buf = ""
        self._write_line(cmd)
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            if PROMPT_TOKEN in self._rx_buf:
                self._consume_prompt_token()
                return lines
            chunk = self._poll_lines(lines, line_cb=line_cb)
            if chunk:
                if PROMPT_TOKEN in chunk or PROMPT_TOKEN in self._rx_buf:
                    self._consume_prompt_token()
                    return lines
            else:
                time.sleep(0.01)
        tail = "\n".join(lines[-20:])
        raise RuntimeError(f"Timeout waiting prompt after '{cmd}'. Last lines:\n{tail}")

    def apply_param(self, key: str, value: object, timeout_s: float):
        cmd_value = value_to_cmd(value)
        key_u = str(key).upper()
        expected_prefix = "# RESULT: %s =" % key_u
        lines: List[str] = []
        self._write_line(f"{key_u} {cmd_value}")
        ok, lines, tok = self._wait_for_tokens(
            timeout_s,
            ASSIGN_ERR_TOKENS + (expected_prefix,) + UNKNOWN_CMD_TOKENS,
            lines,
        )
        if ok and (tok == expected_prefix):
            return
        if ok and (tok in ASSIGN_ERR_TOKENS):
            for l in lines[::-1]:
                if any(t in l for t in ASSIGN_ERR_TOKENS):
                    raise RuntimeError(l)
            raise RuntimeError(f"ASSIGN ERROR for {key_u}")
        if ok and (tok in UNKNOWN_CMD_TOKENS):
            self.sync_prompt(max(4.0, timeout_s))
            lines = []
            self._write_line(f"{key_u} {cmd_value}")
            ok2, lines, tok2 = self._wait_for_tokens(
                timeout_s,
                ASSIGN_ERR_TOKENS + (expected_prefix,) + UNKNOWN_CMD_TOKENS,
                lines,
            )
            if ok2 and (tok2 == expected_prefix):
                return
            if ok2 and (tok2 in ASSIGN_ERR_TOKENS):
                for l in lines[::-1]:
                    if any(t in l for t in ASSIGN_ERR_TOKENS):
                        raise RuntimeError(l)
                raise RuntimeError(f"ASSIGN ERROR for {key_u}")
            tail = "\n".join(lines[-20:])
            raise RuntimeError(f"Failed to assign {key_u} after resync. Last lines:\n{tail}")

        # No explicit token received in time (usually due USB line-fragment timing).
        # Retry once before failing hard.
        self.sync_prompt(max(4.0, timeout_s))
        lines = []
        self._write_line(f"{key_u} {cmd_value}")
        ok2, lines, tok2 = self._wait_for_tokens(
            timeout_s,
            ASSIGN_ERR_TOKENS + (expected_prefix,) + UNKNOWN_CMD_TOKENS,
            lines,
        )
        if ok2 and (tok2 == expected_prefix):
            return
        if ok2 and (tok2 in ASSIGN_ERR_TOKENS):
            for l in lines[::-1]:
                if any(t in l for t in ASSIGN_ERR_TOKENS):
                    raise RuntimeError(l)
            raise RuntimeError(f"ASSIGN ERROR for {key_u}")
        if ok2 and (tok2 in UNKNOWN_CMD_TOKENS):
            tail = "\n".join(lines[-20:])
            raise RuntimeError(f"Unknown command while assigning {key_u}. Last lines:\n{tail}")

        tail = "\n".join(lines[-20:])
        raise RuntimeError(f"Missing assign confirmation for {key_u}. Last lines:\n{tail}")

    def check_config(self, timeout_s: float):
        lines: List[str] = []

        def _do_check(timeout_once: float) -> Tuple[bool, List[str], str]:
            self._write_line("check")
            return self._wait_for_tokens(
                timeout_once,
                CHECK_OK_TOKENS + CHECK_ERR_TOKENS + (READY_TOKEN, PROMPT_TOKEN),
                lines,
            )

        ok, lines, tok = _do_check(timeout_s)
        if not ok:
            # Recovery path: resync prompt and retry check once.
            self.sync_prompt(max(4.0, timeout_s))
            lines = []
            ok, lines, tok = _do_check(timeout_s)
            if not ok:
                tail = "\n".join(lines[-20:])
                raise RuntimeError(f"Timeout waiting check response. Last lines:\n{tail}")

        if tok in CHECK_ERR_TOKENS:
            err_lines = []
            for l in lines:
                if any(t in l for t in CHECK_ERR_TOKENS):
                    err_lines.append(l)
            err = "\n".join(err_lines[-3:])
            raise RuntimeError(err if err else "check failed")

        # If we only observed READY/prompt (without explicit check result),
        # issue one explicit retry before proceeding.
        if tok in (READY_TOKEN, PROMPT_TOKEN):
            lines = []
            ok2, lines, tok2 = _do_check(max(2.0, timeout_s))
            if ok2 and tok2 in CHECK_ERR_TOKENS:
                err_lines = []
                for l in lines:
                    if any(t in l for t in CHECK_ERR_TOKENS):
                        err_lines.append(l)
                err = "\n".join(err_lines[-3:])
                raise RuntimeError(err if err else "check failed")

    def get_supported_param_keys(self, timeout_s: float) -> set:
        if isinstance(self._param_keys_cache, set):
            return set(self._param_keys_cache)
        keys = discover_firmware_param_keys(self, timeout_s)
        self._param_keys_cache = set(keys)
        return set(self._param_keys_cache)

    def run_command(self, command: str = "control"):
        self._write_line(str(command))

    def stop_run(self, timeout_s: float):
        self._write_line("stop")
        ok, lines, _ = self._wait_for_tokens(timeout_s, (READY_TOKEN, PROMPT_TOKEN))
        if ok:
            return
        # force interrupt fallback
        self._write_ctrl_c()
        time.sleep(0.1)
        self._write_line("")
        ok2, lines, _ = self._wait_for_tokens(timeout_s, (READY_TOKEN, PROMPT_TOKEN), lines)
        if not ok2:
            tail = "\n".join(lines[-20:])
            raise RuntimeError(f"Failed to return to prompt after stop. Last lines:\n{tail}")


def value_to_cmd(v: object) -> str:
    if v is None:
        return "None"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        if math.isfinite(v):
            return f"{v:.10g}"
        raise ValueError("non-finite float")
    return str(v)


def _slug(s: str) -> str:
    out = []
    for ch in str(s):
        if ch.isalnum():
            out.append(ch.lower())
        elif ch in (" ", "-", "_"):
            out.append("_")
    slug = "".join(out).strip("_")
    return slug or "experiment"


def catalog_from_config(cfg_payload: Dict[str, object]) -> List[ExperimentSpec]:
    exps = cfg_payload.get("experiments", {})
    catalog: List[ExperimentSpec] = []

    if isinstance(exps, list):
        rows = exps
    elif isinstance(exps, dict):
        rows = []
        for exp_id, body in exps.items():
            if isinstance(body, dict):
                row = dict(body)
                row.setdefault("id", exp_id)
                rows.append(row)
    else:
        rows = []

    for i, row in enumerate(rows):
        if not isinstance(row, dict):
            continue
        exp_id = str(row.get("id", row.get("exp_id", f"EXP_{i+1}"))).strip()
        if not exp_id:
            exp_id = f"EXP_{i+1}"
        params = row.get("params", {})
        if not isinstance(params, dict):
            params = {}
        plot = row.get("plot", {})
        if not isinstance(plot, dict):
            plot = {}
        metrics = row.get("metrics", {})
        if not isinstance(metrics, dict):
            metrics = {}
        completion = row.get("completion", {})
        if not isinstance(completion, dict):
            completion = {}

        kind = str(row.get("kind", "standard")).lower()
        sweep_cfg = row.get("sweep", {})
        if not isinstance(sweep_cfg, dict):
            sweep_cfg = {}
        if (kind == "standard") and sweep_cfg:
            kind = "sweep"

        if kind == "fopdt":
            completion.setdefault("mode", "token")
            completion.setdefault("done_tokens", [DONE_TOKEN])
            completion.setdefault("require_prompt", True)
            completion.setdefault("stop_on_done", False)
        elif kind == "tuning":
            completion.setdefault("mode", "token")
            completion.setdefault("done_tokens", [DONE_TOKEN])
            completion.setdefault("require_prompt", True)
            completion.setdefault("stop_on_done", False)

        catalog.append(
            ExperimentSpec(
                number=i,
                exp_id=exp_id,
                shortname=str(row.get("shortname", _slug(exp_id))),
                description=str(row.get("description", exp_id)),
                base_params=dict(params),
                kind=kind,
                plot_cfg=dict(plot),
                metrics_cfg=dict(metrics),
                sweep_cfg=dict(sweep_cfg),
                completion_cfg=dict(completion),
                run_command=str(row.get("command", "")).strip().lower(),
            )
        )
    return catalog


def catalog_lookup(catalog: Sequence[ExperimentSpec], key: str) -> ExperimentSpec:
    s = str(key).strip()
    for exp in catalog:
        if s == str(exp.number) or s.upper() == exp.exp_id.upper():
            return exp
    raise KeyError(f"Unknown experiment '{key}'")


def default_config_payload() -> Dict[str, object]:
    return {
        "runner": {
            "port": "auto",
            "baud": BAUD_DEFAULT,
            "timeout": 8.0,
            "live": True,
            "settle_band": 1.0,
            "verbose": False,
            "out": "runs",
        },
        "experiments": [],
        "metrics": {
            "compute": "all",
            "display": [
                "IAE",
                "ISE",
                "ITAE",
                "steady_state_error_c",
                "rise_time_s",
                "settling_time_s",
                "overshoot_pct",
                "op_rms_pct",
            ],
            "save_plot": True,
            "show_window": True,
        },
        "plot": {
            "figsize": [11, 7],
            "dpi": 140,
            "line_width": 1.6,
            "show_grid": True,
            "legend_loc": "best",
            "x_axis_mode": "zero",
            "temperature_axis_mode": "auto",
            "output_axis_mode": "custom",
            "xlim": None,
            "temperature_ylim": None,
            "output_ylim": [0, 110],
        },
    }


def _load_config_file(path: Path) -> Dict[str, object]:
    text = path.read_text(encoding="utf-8")
    if _yaml is None:
        raise RuntimeError("YAML config requires PyYAML. Install with: pip install pyyaml")
    payload = _yaml.safe_load(text)
    if not isinstance(payload, dict):
        raise RuntimeError(f"Invalid config file {path}: top level must be object")
    return payload


def _save_default_config(path: Path, payload: Dict[str, object]) -> None:
    if _yaml is None:
        raise RuntimeError("Cannot create YAML config because PyYAML is not installed. Install with: pip install pyyaml")
    header = (
        "# PicoPID experiments configuration\n"
        "# - runner: script-level defaults\n"
        "# - plot: global plot defaults\n"
        "# - experiments: define your experiments (dict or list forms supported)\n"
        "# - sweep support: set kind: sweep and provide sweep.grid with parameter arrays\n"
    )
    body = _yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)
    path.write_text(header + "\n" + body, encoding="utf-8")


def load_or_create_config(path: Path) -> Dict[str, object]:
    if not path.exists():
        payload = default_config_payload()
        _save_default_config(path, payload)
        _emit_lab("created config template: %s" % str(path))
        return payload
    try:
        payload = _load_config_file(path)
    except Exception as ex:
        raise RuntimeError(f"Invalid config file {path}: {ex}")
    return payload


def _runner_cfg_from_payload(cfg_payload: Dict[str, object]) -> Dict[str, object]:
    runner = cfg_payload.get("runner", {})
    if isinstance(runner, dict):
        return dict(runner)
    return {}


def merge_plot_cfg(base_cfg: Optional[Dict[str, object]], override_cfg: Optional[Dict[str, object]]) -> Dict[str, object]:
    out: Dict[str, object] = {}
    if isinstance(base_cfg, dict):
        out.update(base_cfg)
    if isinstance(override_cfg, dict):
        out.update(override_cfg)
    return out


def apply_run_plot_defaults(plot_cfg: Optional[Dict[str, object]], params: Dict[str, object]) -> Dict[str, object]:
    cfg = dict(plot_cfg or {})

    # Temperature axis default: show from ambient floor to a margin above SP.
    has_t_ylim = isinstance(cfg.get("temperature_ylim"), (list, tuple)) and len(cfg.get("temperature_ylim")) == 2
    if not has_t_ylim:
        sp = _safe_float(params.get("SETPOINT_C"), None)
        if sp is None:
            sp = _safe_float(params.get("TUNING_TARGET_C"), None)
        if sp is not None:
            cfg["temperature_ylim"] = [0.0, float(sp) + 10.0]

    # Output axis default: keep fixed engineering frame for readability.
    has_o_ylim = isinstance(cfg.get("output_ylim"), (list, tuple)) and len(cfg.get("output_ylim")) == 2
    if not has_o_ylim:
        cfg["output_ylim"] = [0.0, 110.0]

    # X axis default: derive from planned run length.
    has_xlim = isinstance(cfg.get("xlim"), (list, tuple)) and len(cfg.get("xlim")) == 2
    if has_xlim:
        return cfg

    run_s = _safe_float(params.get("EXPERIMENT_RUN_S"), None)
    if (run_s is not None) and (run_s > 0.0):
        cfg["xlim"] = [0.0, float(run_s)]
    return cfg


def apply_axis_config(ax1, ax2, cfg: Optional[Dict[str, object]] = None):
    c = cfg or {}
    x_mode = str(c.get("x_axis_mode", "zero")).lower()  # zero | auto | custom
    t_mode = str(c.get("temperature_axis_mode", "auto")).lower()  # zero | auto | custom
    o_mode = str(c.get("output_axis_mode", "custom")).lower()  # zero | auto | custom

    xlim = c.get("xlim", None)
    temp_ylim = c.get("temperature_ylim", None)
    out_ylim = c.get("output_ylim", None)

    # Explicit limits always win over mode flags.
    has_xlim = isinstance(xlim, (list, tuple)) and len(xlim) == 2
    has_t_ylim = isinstance(temp_ylim, (list, tuple)) and len(temp_ylim) == 2
    has_o_ylim = isinstance(out_ylim, (list, tuple)) and len(out_ylim) == 2

    if has_xlim:
        ax1.set_xlim(float(xlim[0]), float(xlim[1]))
        ax2.set_xlim(float(xlim[0]), float(xlim[1]))
    elif x_mode == "zero":
        ax1.set_xlim(left=0.0)
        ax2.set_xlim(left=0.0)

    if has_t_ylim:
        ax1.set_ylim(float(temp_ylim[0]), float(temp_ylim[1]))
    elif t_mode == "zero":
        ax1.set_ylim(bottom=0.0)

    if has_o_ylim:
        ax2.set_ylim(float(out_ylim[0]), float(out_ylim[1]))
    elif o_mode == "zero":
        ax2.set_ylim(bottom=0.0)


def build_run_dir(out_root: Path, exp: ExperimentSpec, suffix: str = "") -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"{ts}__{exp.exp_id}__{exp.shortname}"
    if suffix:
        stem += f"__{suffix}"
    run_dir = out_root / stem
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def apply_overrides(
    params: Dict[str, object],
    config_params: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    p = dict(params)
    if config_params:
        p.update(config_params)
    return p


def _parse_param_keys_from_lines(lines: Sequence[str]) -> set:
    keys = set()
    for line in lines:
        for m in _PARAM_ASSIGN_RE.finditer(str(line)):
            k = str(m.group(1)).strip().upper()
            if k:
                keys.add(k)
    return keys


def _config_declared_keys() -> set:
    config_path = _REPO_ROOT / "firmware" / "config.py"
    try:
        text = config_path.read_text(encoding="utf-8")
    except Exception:
        return set()
    out = set()
    for m in _CONFIG_ASSIGN_RE.finditer(text):
        out.add(str(m.group(1)).strip().upper())
    return out


def discover_firmware_param_keys(session: PicoSession, timeout_s: float) -> set:
    keys = set()
    for cmd in ("params all", "params", "params pid", "params onoff", "params fuzzy", "params mpc", "params tune", "params model"):
        try:
            lines = session.send_cmd_wait_prompt(cmd, timeout_s=max(2.0, timeout_s))
        except Exception:
            continue
        keys.update(_parse_param_keys_from_lines(lines))
    if keys:
        return keys
    return _config_declared_keys()


def preflight_recipe_params(session: PicoSession, params: Dict[str, object], timeout_s: float) -> None:
    recipe_keys = set()
    for k in params.keys():
        ku = str(k).strip().upper()
        if ku:
            recipe_keys.add(ku)
    if not recipe_keys:
        return
    supported = session.get_supported_param_keys(timeout_s)
    if not supported:
        return
    unsupported = sorted(k for k in recipe_keys if k not in supported)
    if not unsupported:
        return
    preview = ", ".join(unsupported[:10])
    if len(unsupported) > 10:
        preview += ", ..."
    raise RuntimeError(
        "Recipe contains unsupported parameter(s): %s. "
        "Verify firmware build/branch compatibility and run 'params' on device." % preview
    )


def safe_apply_params(session: PicoSession, params: Dict[str, object], timeout_s: float):
    p = dict(params)
    order1 = ["CONTROL_MODE"]
    order2 = ["TS_S", "EXPERIMENT_RUN_S", "SETPOINT_TYPE", "SETPOINT_C", "SETPOINT_RAMP_RATE"]
    order3 = [
        # Apply tuning dependency chain before variant selection to avoid rollback.
        "TUNING_RULE",
        "PID_VARIANT",
        "PID_AW_TYPE",
        "PID_ALGORITHM",
    ]
    sent_keys = []

    for k in order1 + order2:
        if k in p:
            session.apply_param(k, p.pop(k), timeout_s)
            sent_keys.append(k)

    for k in order3:
        if k in p:
            session.apply_param(k, p.pop(k), timeout_s)
            sent_keys.append(k)

    for k in sorted(p.keys()):
        session.apply_param(k, p[k], timeout_s)
        sent_keys.append(k)

    session.check_config(timeout_s)
    return sent_keys


def collect_run(
    session: PicoSession,
    run_timeout_s: float,
    stop_at_s: Optional[float],
    min_done_s: Optional[float],
    done_tokens: Optional[Sequence[str]],
    require_prompt_on_done: bool,
    stop_on_done: bool,
    show_info_lines: bool,
    live: LivePlotter,
    live_metrics: Optional[LiveMetricsPanel],
    verbose: bool,
    ts_nominal_s: Optional[float] = None,
    run_started_at_s: Optional[float] = None,
) -> Tuple[np.ndarray, List[str], bool]:
    raw_lines: List[str] = []
    rows: List[Tuple[float, float, float, float, float, float]] = []
    t0 = float(run_started_at_s) if (run_started_at_s is not None) else time.monotonic()
    last_ts = -1.0
    synthetic_t0_added = False
    prompt_seen = False
    done_seen = False
    saw_telemetry = False
    last_firmware_error = None
    done_tokens = [str(t) for t in (done_tokens or []) if str(t)]
    stop_sent = False
    interrupted = False
    stdin_poll_enabled = True

    while True:
        try:
            now = time.monotonic()
            if now - t0 > run_timeout_s:
                raise TimeoutError(f"Run timeout after {run_timeout_s:.1f}s")

            before = len(raw_lines)
            chunk = session._poll_lines(raw_lines)
            had_io = bool(chunk)
            if chunk:
                if (PROMPT_TOKEN in chunk) or (PROMPT_TOKEN in session._rx_buf) or (READY_TOKEN in chunk):
                    prompt_seen = True
                for line in raw_lines[before:]:
                    if line.startswith(READY_TOKEN):
                        prompt_seen = True
                    parsed = parse_telemetry_line(line)
                    if parsed is not None:
                        saw_telemetry = True
                        # Canonical x-axis: sample index * nominal Ts.
                        # This avoids compressed timelines when many telemetry lines are
                        # drained from serial buffer in one host poll cycle.
                        if (ts_nominal_s is not None) and (float(ts_nominal_s) > 0.0):
                            ts = float(len(rows)) * float(ts_nominal_s)
                        else:
                            ts = time.monotonic() - t0
                            if ts < 0.0:
                                ts = 0.0
                        if ts < last_ts:
                            ts = last_ts
                        last_ts = ts
                        if len(parsed) >= 6:
                            _, pv, sp, op, y_hat, y_pred = parsed
                        else:
                            _, pv, sp, op = parsed
                            y_hat = float("nan")
                            y_pred = float("nan")
                        # Ensure unified runs always include an explicit t=0 sample for plotting
                        # and duration reporting, even if first telemetry arrives late.
                        if (not synthetic_t0_added) and (len(rows) == 0) and (ts > 1e-9):
                            rows.append((0.0, pv, sp, op, y_hat, y_pred))
                            synthetic_t0_added = True
                        rows.append((ts, pv, sp, op, y_hat, y_pred))
                    if done_tokens:
                        line_norm = line.strip()
                        elapsed_s = time.monotonic() - t0
                        done_gate_ok = (min_done_s is None) or (elapsed_s >= float(min_done_s))
                        for tok in done_tokens:
                            tok_norm = str(tok).strip()
                            if done_gate_ok and (line_norm == tok_norm):
                                done_seen = True
                                break
                    if line.startswith("# ERROR:"):
                        last_firmware_error = line
                    if show_info_lines and (not verbose) and line.startswith("# "):
                        _emit_fw(line)

            # Live plot throttle handled inside plotter.
            if rows:
                arr = np.asarray(rows, dtype=float)
                live.update(arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3], arr[:, 4], arr[:, 5])
                if live_metrics is not None:
                    live_metrics.update(arr)

            if done_seen:
                if stop_on_done and (not stop_sent):
                    session.stop_run(timeout_s=8.0)
                    stop_sent = True
                    prompt_seen = True
                if (not require_prompt_on_done) or prompt_seen:
                    break

            # Fail fast when firmware reports an error and returns to prompt
            # without producing telemetry/done token for this run.
            if prompt_seen and (last_firmware_error is not None) and (not done_seen) and (not saw_telemetry):
                raise RuntimeError("Firmware error before telemetry: %s" % last_firmware_error)

            if stop_at_s is not None and (time.monotonic() - t0) >= stop_at_s:
                session.stop_run(timeout_s=8.0)
                break

            if stdin_poll_enabled:
                try:
                    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if ready:
                        user_cmd = sys.stdin.readline().strip().lower()
                        if user_cmd:
                            _emit_lab("")
                        token = user_cmd.split()[0] if user_cmd else ""
                        if token in ("stop", "s") or (token and set(token) == {"s"}):
                            _emit_lab("stop requested, sending 'stop'...")
                            session.stop_run(timeout_s=8.0)
                            break
                        if token in ("help", "h") or (token and set(token) == {"h"}):
                            _emit_lab("run commands: stop, help, Ctrl+C")
                        elif user_cmd:
                            pass
                except Exception:
                    stdin_poll_enabled = False

            if had_io:
                time.sleep(COLLECT_LOOP_SLEEP_ACTIVE_S)
            else:
                time.sleep(COLLECT_LOOP_SLEEP_IDLE_S)
        except KeyboardInterrupt:
            interrupted = True
            _emit_lab("Ctrl+C: stop requested, sending 'stop'...")
            try:
                session.stop_run(timeout_s=8.0)
            except Exception as ex:
                _emit_lab("warning: failed to stop cleanly after Ctrl+C: %s" % ex)
            break

    telemetry = np.asarray(rows, dtype=float) if rows else np.zeros((0, 6), dtype=float)
    return telemetry, raw_lines, interrupted


def compute_metrics(
    telemetry: np.ndarray,
    settle_band_c: float,
    relay_bias_pct: Optional[float] = None,
) -> Dict[str, object]:
    m: Dict[str, object] = {}
    if telemetry.size == 0:
        m["samples"] = 0
        m["error"] = "no telemetry"
        return m

    t = telemetry[:, 0]
    pv = telemetry[:, 1]
    sp = telemetry[:, 2]
    op = telemetry[:, 3]
    e = sp - pv

    m["samples"] = int(len(t))
    m["duration_s"] = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    m["pv_mean_c"] = float(np.mean(pv))
    m["pv_std_c"] = float(np.std(pv))
    m["op_rms_pct"] = float(np.sqrt(np.mean(op * op)))

    # Noise-floor metrics (useful for baseline/no-heating runs).
    pv_centered = pv - float(np.mean(pv))
    m["noise_std_c"] = float(np.std(pv_centered))
    m["noise_rms_c"] = float(np.sqrt(np.mean(pv_centered * pv_centered)))
    m["noise_p2p_c"] = float(np.max(pv) - np.min(pv))
    m["noise_mad_c"] = float(np.median(np.abs(pv - float(np.median(pv)))))

    if len(t) > 1:
        m["IAE"] = float(np.trapezoid(np.abs(e), t))
        m["ISE"] = float(np.trapezoid(e * e, t))
        m["ITAE"] = float(np.trapezoid(np.abs(e) * (t - t[0]), t))
        m["effort_integral_abs_op"] = float(np.trapezoid(np.abs(op), t))
    else:
        m["IAE"] = 0.0
        m["ISE"] = 0.0
        m["ITAE"] = 0.0
        m["effort_integral_abs_op"] = 0.0

    sp_final = float(np.median(sp[-max(5, len(sp) // 10) :]))
    pv_final = float(np.median(pv[-max(5, len(pv) // 10) :]))
    pv0 = float(np.median(pv[: max(5, min(len(pv), int(max(1, round(5.0 / max(1e-6, np.mean(np.diff(t))) if len(t) > 1 else 1)))))]))
    amp = sp_final - pv0

    m["sp_final_c"] = sp_final
    m["pv_final_c"] = pv_final
    m["steady_state_error_c"] = float(abs(sp_final - pv_final))

    # Rise time 10-90 on PV relative to initial and final SP target.
    rise_time = None
    if abs(amp) > 1e-6:
        y10 = pv0 + 0.1 * amp
        y90 = pv0 + 0.9 * amp
        if amp > 0:
            i10 = np.where(pv >= y10)[0]
            i90 = np.where(pv >= y90)[0]
        else:
            i10 = np.where(pv <= y10)[0]
            i90 = np.where(pv <= y90)[0]
        if len(i10) > 0 and len(i90) > 0:
            t10 = t[int(i10[0])]
            t90 = t[int(i90[0])]
            if t90 >= t10:
                rise_time = float(t90 - t10)
    m["rise_time_s"] = rise_time

    # Overshoot relative to final SP.
    if abs(amp) > 1e-6:
        if amp > 0:
            ov = max(0.0, float(np.max(pv) - sp_final))
        else:
            ov = max(0.0, float(sp_final - np.min(pv)))
        m["overshoot_c"] = ov
        m["overshoot_pct"] = float(100.0 * ov / abs(amp))
    else:
        m["overshoot_c"] = 0.0
        m["overshoot_pct"] = 0.0

    # Settling time in absolute ±band around final SP (first time after which all remain in band).
    settling = None
    band = float(settle_band_c)
    in_band = np.abs(pv - sp_final) <= band
    if len(in_band) > 0:
        for i in range(len(in_band)):
            if np.all(in_band[i:]):
                settling = float(t[i] - t[0])
                break
    m["settling_time_s"] = settling
    m["settle_band_c"] = band

    # Switching count:
    # - default: ONOFF threshold at 0.5%
    # - relay tuning: split states around relay bias (high/low relay state)
    if relay_bias_pct is not None:
        on = op >= float(relay_bias_pct)
    else:
        on = op > 0.5
    if len(on) > 1:
        m["switch_count"] = int(np.sum(np.abs(np.diff(on.astype(int)))))
        m["duty_pct"] = float(np.mean(op))
    else:
        m["switch_count"] = 0
        m["duty_pct"] = float(op[0]) if len(op) else 0.0

    return m


def save_csv(path: Path, telemetry: np.ndarray):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        has_est = telemetry.ndim == 2 and telemetry.shape[1] >= 6
        if has_est:
            w.writerow(["time_s", "PV", "SP", "OP", "YH", "YP"])
            for row in telemetry:
                yh = "" if np.isnan(row[4]) else f"{row[4]:.1f}"
                yp = "" if np.isnan(row[5]) else f"{row[5]:.1f}"
                w.writerow([f"{row[0]:.2f}", f"{row[1]:.1f}", f"{row[2]:.1f}", f"{row[3]:.1f}", yh, yp])
        else:
            w.writerow(["time_s", "PV", "SP", "OP"])
            for row in telemetry:
                w.writerow([f"{row[0]:.2f}", f"{row[1]:.1f}", f"{row[2]:.1f}", f"{row[3]:.1f}"])


def save_plot(path: Path, telemetry: np.ndarray, title: str, plot_cfg: Optional[Dict[str, object]] = None):
    cfg = plot_cfg or {}
    figsize = cfg.get("figsize", [11, 7])
    if not (isinstance(figsize, (list, tuple)) and len(figsize) == 2):
        figsize = [11, 7]
    dpi = int(cfg.get("dpi", 140))
    lw = float(cfg.get("line_width", 1.6))
    show_grid = bool(cfg.get("show_grid", True))
    legend_loc = str(cfg.get("legend_loc", "best"))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(float(figsize[0]), float(figsize[1])), sharex=True)
    if telemetry.size > 0:
        t = telemetry[:, 0]
        pv = telemetry[:, 1]
        sp = telemetry[:, 2]
        op = telemetry[:, 3]
        ax1.plot(t, pv, label="PV", linewidth=lw)
        ax1.plot(t, sp, label="SP", linewidth=lw)
        if telemetry.shape[1] >= 6:
            y_hat = telemetry[:, 4]
            y_pred = telemetry[:, 5]
            if np.any(np.isfinite(y_hat)):
                ax1.plot(t, y_hat, label="Yh (estimated)", linewidth=max(1.0, 0.9 * lw), linestyle="--", color="tab:green")
            if np.any(np.isfinite(y_pred)):
                ax1.plot(t, y_pred, label="Yp (horizon-end)", linewidth=max(1.0, 0.9 * lw), linestyle="-.", color="tab:red")
        ax2.plot(t, op, label="OP", color="tab:orange", linewidth=lw)
    ax1.set_ylabel("Temperature [°C]")
    ax2.set_ylabel("Output [%]")
    ax2.set_xlabel("time [s]")
    ax1.grid(show_grid)
    ax2.grid(show_grid)
    apply_axis_config(ax1, ax2, cfg)
    h1, l1 = ax1.get_legend_handles_labels()
    if h1:
        ax1.legend(loc=legend_loc)
    h2, l2 = ax2.get_legend_handles_labels()
    if h2:
        ax2.legend(loc=legend_loc)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def parse_fopdt_summary(raw_lines: Sequence[str]) -> Optional[Dict[str, float]]:
    for line in raw_lines:
        m = FOPDT_SUMMARY_RE.search(line)
        if m:
            return {
                "K": float(m.group(1)),
                "tau_s": float(m.group(2)),
                "theta_s": float(m.group(3)),
                "rmse": float(m.group(4)),
            }
    for line in raw_lines:
        m = FOPDT_APPLIED_RE.search(line)
        if m:
            return {
                "K": float(m.group(1)),
                "tau_s": float(m.group(2)),
                "theta_s": float(m.group(3)),
                "rmse": float(m.group(4)),
            }
    return None


def parse_fopdt_model_from_log(raw_lines: Sequence[str]) -> Optional[Dict[str, object]]:
    summary = parse_fopdt_summary(raw_lines)
    if summary is None:
        return None
    out: Dict[str, object] = {
        "K": float(summary["K"]),
        "tau_s": float(summary["tau_s"]),
        "theta_s": float(summary["theta_s"]),
        "rmse": float(summary["rmse"]),
    }
    for line in raw_lines:
        m = FOPDT_STEP_RE.search(line)
        if m is not None:
            out["u0_pct"] = float(m.group(1))
            out["u1_pct"] = float(m.group(2))
            break
    for line in raw_lines:
        m = FOPDT_Y0_RE.search(line)
        if m is not None:
            out["y0"] = float(m.group(1))
            break
    for line in raw_lines:
        m = FOPDT_Y1_RE.search(line)
        if m is not None:
            out["y1"] = float(m.group(1))
            break
    for line in raw_lines:
        m = FOPDT_METHOD_RE.search(line)
        if m is not None:
            out["method"] = str(m.group(1)).upper()
            break
    return out


def write_fopdt_model_artifacts(raw_lines: Sequence[str], run_dir: Path):
    model = parse_fopdt_model_from_log(raw_lines)
    if model is None:
        return
    text = json.dumps(model, ensure_ascii=False)
    (run_dir / "model.json").write_text(text + "\n", encoding="utf-8")


def parse_tuning_summary(raw_lines: Sequence[str]) -> Optional[Dict[str, object]]:
    out: Dict[str, object] = {}
    for line in raw_lines:
        m = TUNING_RELAY_METRICS_RE.search(line)
        if m:
            out["A_c"] = float(m.group(1))
            out["d_eff_pct"] = float(m.group(2))
            out["Ku"] = float(m.group(3))
            out["Pu_s"] = float(m.group(4))
            out["pv_pp_c"] = float(m.group(5))
            break
    for line in raw_lines:
        m = TUNING_CYCLES_RE.search(line)
        if m:
            out["cycles_used"] = int(m.group(1))
            break
    for line in raw_lines:
        m = TUNING_APPLIED_SET_RE.search(line)
        if m:
            out["applied_set"] = str(m.group(1)).strip()
            break
    for line in raw_lines:
        m = TUNING_GAINS_RE.search(line)
        if m:
            out["kp"] = float(m.group(1))
            out["ki"] = float(m.group(2))
            out["kd"] = float(m.group(3))
            out["source"] = "tuning"
            break
    if "kp" not in out:
        for line in raw_lines:
            m = TUNING_PARALLEL_FORM_RE.search(line)
            if not m:
                continue
            try:
                out["kp"] = float(m.group(1))
            except Exception:
                pass
            try:
                out["ki"] = float(m.group(2))
            except Exception:
                pass
            try:
                out["kd"] = float(m.group(3))
            except Exception:
                pass
            if any(k in out for k in ("kp", "ki", "kd")):
                out["source"] = "applied_parallel_form"
            break
    if out:
        return out
    return None


def _resolve_completion_spec(
    exp: ExperimentSpec,
    params: Dict[str, object],
    duration: Optional[float],
) -> Dict[str, object]:
    c = dict(exp.completion_cfg or {})
    mode = str(c.get("mode", "")).lower().strip()
    if not mode:
        if exp.kind in ("fopdt", "tuning"):
            mode = "token"
        else:
            mode = "duration"

    out: Dict[str, object] = {"mode": mode}
    if mode == "token":
        done_tokens = c.get("done_tokens", [])
        if isinstance(done_tokens, str):
            done_tokens = [done_tokens]
        if not isinstance(done_tokens, list):
            done_tokens = []
        done_tokens = [str(t) for t in done_tokens if str(t)]
        if (not done_tokens) and exp.kind in ("fopdt", "tuning"):
            done_tokens = list(DONE_TOKENS)
        out["done_tokens"] = done_tokens
        out["require_prompt"] = bool(c.get("require_prompt", exp.kind in ("fopdt", "tuning")))
        out["stop_on_done"] = bool(c.get("stop_on_done", exp.kind == "tuning"))

        timeout_s = _safe_float(c.get("timeout_s"), None)
        if (timeout_s is None) or (timeout_s <= 0.0):
            if (duration is not None) and (float(duration) > 0.0):
                timeout_s = max(30.0, float(duration) + 30.0)
        if (timeout_s is None) or (timeout_s <= 0.0):
            kind_l = str(exp.kind).lower()
            if kind_l == "fopdt":
                timeout_s = 330.0
            elif kind_l == "tuning":
                timeout_s = 420.0
            else:
                timeout_s = max(30.0, (float(duration) if duration is not None else 120.0) + 30.0)
        out["timeout_s"] = float(max(10.0, timeout_s))
        # Token-mode runs should end on explicit firmware done markers.
        # Keep stop_at_s opt-in only (from config), otherwise disabled.
        stop_at_s = _safe_float(c.get("stop_at_s"), None)
        out["stop_at_s"] = None if (stop_at_s is None or stop_at_s <= 0.0) else float(stop_at_s)
    else:
        run_s = _safe_float(c.get("duration_s"), duration)
        if (run_s is None) or (run_s <= 0.0):
            run_s = 120.0
        out["duration_s"] = float(run_s)
        out["stop_at_s"] = float(run_s)
        timeout_s = _safe_float(c.get("timeout_s"), None)
        if (timeout_s is None) or (timeout_s <= 0.0):
            timeout_s = max(30.0, float(run_s) + 30.0)
        out["timeout_s"] = float(timeout_s)
    return out


_METRIC_LABELS = {
    "IAE": "IAE",
    "ISE": "ISE",
    "ITAE": "ITAE",
    "steady_state_error_c": "SSE",
    "rise_time_s": "rise",
    "settling_time_s": "settle",
    "overshoot_pct": "ov",
    "op_rms_pct": "RMS(OP)",
    "noise_std_c": "Noise Std Dev (sigma)",
    "noise_rms_c": "Noise RMS",
    "noise_p2p_c": "Noise Peak-to-Peak",
    "noise_mad_c": "Noise Median Abs Dev",
    "model_K": "Model K",
    "model_tau_s": "Model tau",
    "model_theta_s": "Model th",
    "model_rmse": "Model rmse",
}

_METRIC_UNITS = {
    "steady_state_error_c": "°C",
    "rise_time_s": "s",
    "settling_time_s": "s",
    "overshoot_pct": "%",
    "op_rms_pct": "%",
    "noise_std_c": "°C",
    "noise_rms_c": "°C",
    "noise_p2p_c": "°C",
    "noise_mad_c": "°C",
    "pv_mean_c": "°C",
    "pv_std_c": "°C",
    "duration_s": "s",
}


def _metric_keys_from_cfg(metrics_cfg: Optional[Dict[str, object]], field: str, fallback: List[str]) -> List[str]:
    if not isinstance(metrics_cfg, dict):
        return list(fallback)
    raw = metrics_cfg.get(field, fallback)
    if isinstance(raw, str):
        if raw.strip().lower() == "all":
            return list(fallback)
        return [raw]
    if isinstance(raw, list):
        out = []
        for k in raw:
            ks = str(k).strip()
            if ks:
                out.append(ks)
        return out if out else list(fallback)
    return list(fallback)


def filter_metrics_for_output(metrics: Dict[str, object], metrics_cfg: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    keep = set(_metric_keys_from_cfg(metrics_cfg, "compute", list(metrics.keys())))
    always = {"samples", "duration_s", "error", "sp_final_c", "pv_final_c", "settle_band_c", "fopdt", "tuning"}
    out = {}
    for k, v in metrics.items():
        if (k in keep) or (k in always):
            out[k] = v
    return out


def merge_metrics_cfg(base_cfg: Optional[Dict[str, object]], override_cfg: Optional[Dict[str, object]]) -> Dict[str, object]:
    out: Dict[str, object] = {}
    if isinstance(base_cfg, dict):
        out.update(base_cfg)
    if isinstance(override_cfg, dict):
        out.update(override_cfg)
    return out


def save_metrics_summary_plot(path: Path, metrics: Dict[str, object], metrics_cfg: Optional[Dict[str, object]] = None):
    if isinstance(metrics_cfg, dict) and (not bool(metrics_cfg.get("save_plot", True))):
        return
    lines = _build_metrics_lines(metrics, metrics_cfg)
    if not lines:
        return
    w, h = _metrics_figure_size_from_lines(lines, min_w=4.2)
    fig = plt.figure(figsize=(w, h), dpi=140)
    ax = fig.add_subplot(111)
    ax.axis("off")
    ax.set_title("Run Metrics", loc="left")
    ax.text(
        0.01,
        0.95,
        "\n".join(lines),
        va="top",
        ha="left",
        family="monospace",
        fontsize=10,
        transform=ax.transAxes,
    )
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def run_single_experiment(
    session: PicoSession,
    exp: ExperimentSpec,
    params: Dict[str, object],
    cfg: RunnerConfig,
    plot_cfg: Optional[Dict[str, object]] = None,
    suffix: str = "",
    hold_live_at_end: bool = False,
) -> RunResult:
    run_dir = build_run_dir(cfg.out_root, exp, suffix=suffix)
    if int(exp.number) >= 0:
        _emit_lab("run %d %s -> %s" % (int(exp.number), str(exp.exp_id), str(run_dir)))
    else:
        cmd_label = str(exp.run_command or exp.shortname or exp.exp_id).strip().lower()
        _emit_lab("run %s -> %s" % (cmd_label, str(run_dir)))
    live_title = str(exp.description).strip() or exp.exp_id
    params = dict(params)
    telemetry_mode = str(params.get("TELEMETRY_MODE", "NORMAL")).upper()
    telemetry_enabled = telemetry_mode != "INFO"

    effective_plot_cfg = apply_run_plot_defaults(
        plot_cfg if isinstance(plot_cfg, dict) else cfg.plot_cfg,
        params,
    )

    preflight_recipe_params(session, params, cfg.timeout_s)
    safe_apply_params(session, params, cfg.timeout_s)

    raw_lines: List[str] = []

    expect_mpc_estimates = (
        str(params.get("CONTROL_MODE", "")).upper() == "MPC"
        and str(params.get("TELEMETRY_MODE", "NORMAL")).upper() == "MPC"
    )
    live = LivePlotter(
        bool(cfg.live and telemetry_enabled),
        refresh_s=cfg.live_refresh_s,
        plot_cfg=effective_plot_cfg,
        title=live_title,
        expect_mpc_estimates=expect_mpc_estimates,
    )
    live_metrics = LiveMetricsPanel(
        enabled=bool(telemetry_enabled and (cfg.metrics_cfg or {}).get("show_window", True)),
        refresh_s=cfg.live_refresh_s,
        metrics_cfg=cfg.metrics_cfg,
        settle_band_c=cfg.settle_band_c,
        title=live_title,
    )
    telemetry = np.zeros((0, 6), dtype=float)
    run_interrupted = False
    run_ok = False
    try:
        # Avoid stale lines/tokens from a previous run affecting completion logic.
        # Prepare all host-side components before issuing run command so we do not lose early telemetry.
        session.clear_pending_io()
        run_started_at_s = time.monotonic()
        run_command = str(exp.run_command or "").lower()
        if run_command not in ("control", "tune", "model", "monitor"):
            if exp.kind == "fopdt":
                run_command = "model"
            elif exp.kind == "tuning":
                run_command = "tune"
            else:
                run_command = "control"
        session.run_command(run_command)
        duration = _safe_float(params.get("EXPERIMENT_RUN_S"), None)
        completion = _resolve_completion_spec(exp, params, duration)
        if completion["mode"] == "token":
            telemetry, raw, run_interrupted = collect_run(
                session,
                run_timeout_s=float(completion["timeout_s"]),
                stop_at_s=_safe_float(completion.get("stop_at_s"), None),
                min_done_s=None,
                done_tokens=completion.get("done_tokens", []),
                require_prompt_on_done=bool(completion.get("require_prompt", False)),
                stop_on_done=bool(completion.get("stop_on_done", False)),
                show_info_lines=True,
                live=live,
                live_metrics=live_metrics,
                verbose=cfg.verbose,
                ts_nominal_s=_safe_float(params.get("TS_S"), TS_S_DEFAULT),
                run_started_at_s=run_started_at_s,
            )
        else:
            ts_nom = _safe_float(params.get("TS_S"), TS_S_DEFAULT)
            telemetry, raw, run_interrupted = collect_run(
                session,
                run_timeout_s=float(completion["timeout_s"]),
                stop_at_s=_safe_float(completion.get("stop_at_s"), None),
                min_done_s=None,
                done_tokens=list(DONE_TOKENS),
                require_prompt_on_done=True,
                stop_on_done=False,
                show_info_lines=True,
                live=live,
                live_metrics=live_metrics,
                verbose=cfg.verbose,
                ts_nominal_s=ts_nom,
                run_started_at_s=run_started_at_s,
            )
        raw_lines.extend(raw)
        run_ok = True
    except Exception:
        # Persist whatever firmware output was already received to aid debugging.
        try:
            if raw_lines:
                (run_dir / "firmware_log.txt").write_text("\n".join(raw_lines) + "\n", encoding="utf-8")
            (run_dir / "params_sent.json").write_text(json.dumps(params, indent=2, ensure_ascii=False), encoding="utf-8")
            (run_dir / "plot_settings.json").write_text(
                json.dumps(effective_plot_cfg, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        except Exception:
            pass
        raise
    finally:
        keep_open = bool(hold_live_at_end and run_ok)
        live.close(keep_open=keep_open)
        live_metrics.close(keep_open=keep_open)

    relay_bias_pct = None
    if exp.kind == "tuning":
        relay_bias_pct = 50.0

    metrics_full = compute_metrics(telemetry, cfg.settle_band_c, relay_bias_pct=relay_bias_pct)
    if (not telemetry_enabled) and telemetry.size == 0:
        metrics_full["error"] = "telemetry disabled by TELEMETRY_MODE=INFO"
        _emit_lab("telemetry disabled by TELEMETRY_MODE=INFO (status lines only)")
    if exp.kind == "fopdt":
        f = parse_fopdt_summary(raw_lines)
        if f:
            metrics_full["fopdt"] = f
            metrics_full["model_K"] = float(f.get("K", float("nan")))
            metrics_full["model_tau_s"] = float(f.get("tau_s", float("nan")))
            metrics_full["model_theta_s"] = float(f.get("theta_s", float("nan")))
            metrics_full["model_rmse"] = float(f.get("rmse", float("nan")))
    if exp.kind == "tuning":
        a = parse_tuning_summary(raw_lines)
        if a:
            metrics_full["tuning"] = a
        else:
            _emit_lab("warning: runner could not parse tuning summary from firmware log")

    if exp.kind == "tuning":
        metrics: Dict[str, object] = {}
        for k in ("samples", "duration_s", "switch_count", "error"):
            if k in metrics_full:
                metrics[k] = metrics_full[k]
        if "tuning" in metrics_full:
            metrics["tuning"] = metrics_full["tuning"]
    else:
        metrics = filter_metrics_for_output(metrics_full, cfg.metrics_cfg)

    save_csv(run_dir / "data.csv", telemetry)
    (run_dir / "firmware_log.txt").write_text("\n".join(raw_lines) + "\n", encoding="utf-8")
    (run_dir / "params_sent.json").write_text(json.dumps(params, indent=2, ensure_ascii=False), encoding="utf-8")
    (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    (run_dir / "plot_settings.json").write_text(
        json.dumps(effective_plot_cfg, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    if exp.kind == "fopdt":
        write_fopdt_model_artifacts(raw_lines, run_dir)
    save_plot(run_dir / "plot.png", telemetry, f"{exp.exp_id} ({exp.shortname})", effective_plot_cfg)
    save_metrics_summary_plot(run_dir / "metrics_summary.png", metrics_full, cfg.metrics_cfg)
    return RunResult(
        telemetry=telemetry,
        raw_lines=raw_lines,
        metrics=metrics_full,
        params_sent=params,
        run_dir=run_dir,
    )


def _trial_suffix(params: Dict[str, object], keys: List[str]) -> str:
    out = []
    for k in keys:
        v = params.get(k)
        s = str(v).replace(".", "p").replace("-", "m")
        out.append(f"{k}_{s}")
    return "__".join(out)


def run_param_sweep(
    session: PicoSession,
    exp: ExperimentSpec,
    base_params: Dict[str, object],
    cfg: RunnerConfig,
    plot_cfg: Optional[Dict[str, object]] = None,
):
    sweep_exp = exp.sweep_cfg if isinstance(exp.sweep_cfg, dict) else {}
    grid = sweep_exp.get("grid", sweep_exp)
    if not isinstance(grid, dict) or not grid:
        raise RuntimeError("Sweep experiment '%s' has no valid sweep grid in config." % exp.exp_id)

    keys = [k for k in sorted(grid.keys()) if isinstance(grid.get(k), list) and len(grid.get(k)) > 0]
    if not keys:
        raise RuntimeError("Sweep experiment '%s' has no non-empty list parameters." % exp.exp_id)

    all_rows = []
    for values in itertools.product(*[grid[k] for k in keys]):
        params = dict(base_params)
        trial_params = {}
        for k, v in zip(keys, values):
            params[k] = v
            trial_params[k] = v
        suffix = _trial_suffix(trial_params, keys)
        rr = run_single_experiment(session, exp, params, cfg, plot_cfg=plot_cfg, suffix=suffix, hold_live_at_end=False)
        m = rr.metrics
        row = {"trial": suffix}
        row.update(trial_params)
        row.update(
            {
                "IAE": _safe_float(m.get("IAE"), float("nan")),
                "settling_time_s": _safe_float(m.get("settling_time_s"), float("nan")),
                "overshoot_pct": _safe_float(m.get("overshoot_pct"), float("nan")),
                "steady_state_error_c": _safe_float(m.get("steady_state_error_c"), float("nan")),
            }
        )
        all_rows.append(row)

    summary_dir = build_run_dir(cfg.out_root, exp, suffix="summary")
    summary_csv = summary_dir / "sweep_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        w.writeheader()
        for row in all_rows:
            w.writerow(row)

    # Best by IAE, tie-break by settling time.
    def _key(row):
        iae = row["IAE"]
        st = row["settling_time_s"]
        return (
            float("inf") if (iae is None or not math.isfinite(iae)) else iae,
            float("inf") if (st is None or not math.isfinite(st)) else st,
        )

    best = sorted(all_rows, key=_key)[0]

    fig, ax = plt.subplots(figsize=(10, 5))
    labels = [r["trial"] for r in all_rows]
    vals = [r["IAE"] for r in all_rows]
    ax.bar(range(len(labels)), vals)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("IAE")
    ax.set_title(f"{exp.exp_id} sweep (lower IAE is better)")
    ax.grid(True, axis="y")
    fig.tight_layout()
    fig.savefig(summary_dir / "sweep_compare.png", dpi=140)
    plt.close(fig)

    summary = {
        "best_trial": best,
        "num_trials": len(all_rows),
    }
    (summary_dir / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    _emit_lab("sweep summary")
    _emit_lab("best by IAE (tie settle): %s  IAE=%.3f  settle=%s"
              % (str(best["trial"]), float(best["IAE"]), str(best["settling_time_s"])))
    _emit_lab("summary saved: %s" % str(summary_dir))


def print_catalog(catalog: Sequence[ExperimentSpec]):
    kind_label = {
        "standard": "Standard",
        "fopdt": "FOPDT Identification",
        "tuning": "Tuning",
        "sweep": "Parameter Sweep",
    }
    _emit_lab("experiment catalog")
    for e in catalog:
        k = kind_label.get(str(e.kind).lower(), str(e.kind))
        _emit_lab(" %2d  %-24s  %s: %s" % (int(e.number), str(e.exp_id), str(k), str(e.description)))


def resolve_port(port_arg: str) -> str:
    if str(port_arg).lower() != "auto":
        return port_arg
    p = wait_for_autodetect_port(timeout_s=12.0, poll_s=0.25)
    if p is None:
        _emit_lab("could not auto-detect serial port")
        _emit_lab("available ports:")
        for r in list_ports():
            _emit_lab("  %s: %s" % (str(r["device"]), str(r["description"])))
        raise SystemExit(1)
    return p


def wait_for_autodetect_port(timeout_s: float = 12.0, poll_s: float = 0.25) -> Optional[str]:
    """Wait for Pico USB CDC re-enumeration (e.g., after closing Thonny)."""
    deadline = time.monotonic() + max(0.5, float(timeout_s))
    while time.monotonic() < deadline:
        p = autodetect_port()
        if p is not None:
            return p
        time.sleep(max(0.05, float(poll_s)))
    return None


def candidate_ports_for_connect(port_arg: str) -> List[str]:
    if str(port_arg).lower() != "auto":
        return [str(port_arg)]
    rows = sorted(list_ports(), key=score_port, reverse=True)
    devices = []
    for r in rows:
        dev = str(r.get("device") or "")
        sc = score_port(r)
        # Auto mode should only try likely USB CDC serial ports.
        if sc <= 0 and ("ttyacm" not in dev.lower()) and ("ttyusb" not in dev.lower()) and ("usbmodem" not in dev.lower()):
            continue
        if dev:
            devices.append(dev)
    # Keep stable order and deduplicate.
    out: List[str] = []
    seen = set()
    preferred = autodetect_port()
    if preferred:
        out.append(preferred)
        seen.add(preferred)
    for d in devices:
        if d not in seen:
            out.append(d)
            seen.add(d)
    return out


def connect_session(cfg: RunnerConfig) -> Tuple[PicoSession, str]:
    candidates = candidate_ports_for_connect(cfg.port)
    if not candidates:
        if str(cfg.port).lower() == "auto":
            _emit_lab("waiting for Pico serial port to appear...")
            port = wait_for_autodetect_port(timeout_s=12.0, poll_s=0.25)
            if port is not None:
                candidates = [port]
        if not candidates:
            port = resolve_port(cfg.port)
            candidates = [port]

    session = None
    last_err = None
    chosen_port = None
    for port in candidates:
        _emit_lab("connecting: %s @ %s" % (str(port), str(cfg.baud)))
        last_open_err = None
        for attempt in range(1, 3):
            sess = None
            try:
                _emit_lab("syncing firmware command mode (attempt %d)..." % int(attempt))
                sess = PicoSession(port=port, baud=cfg.baud, timeout_s=cfg.timeout_s, verbose=cfg.verbose)
                sess.clear_pending_io()
                sess.sync_prompt(timeout_s=max(18.0, cfg.timeout_s + 10.0))
                session = sess
                chosen_port = port
                break
            except Exception as ex:
                last_open_err = ex
                try:
                    if sess is not None:
                        sess.close()
                except Exception:
                    pass
                time.sleep(0.25)
        if session is not None:
            break
        last_err = last_open_err

    if session is None or chosen_port is None:
        raise RuntimeError(f"Failed to connect/sync command mode on candidates {candidates}: {last_err}")
    return session, chosen_port


def _build_runner_context(config_path: Path) -> Tuple[List[ExperimentSpec], RunnerConfig]:
    cfg_payload = load_or_create_config(config_path)
    runner_cfg = _runner_cfg_from_payload(cfg_payload)
    catalog = catalog_from_config(cfg_payload)
    out_root = Path(str(runner_cfg.get("out", "runs"))).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    plot_cfg = cfg_payload.get("plot", {}) if isinstance(cfg_payload.get("plot", {}), dict) else {}
    metrics_cfg = cfg_payload.get("metrics", {}) if isinstance(cfg_payload.get("metrics", {}), dict) else {}
    cfg = RunnerConfig(
        port=str(runner_cfg.get("port", "auto")),
        baud=int(runner_cfg.get("baud", BAUD_DEFAULT)),
        out_root=out_root,
        timeout_s=float(runner_cfg.get("timeout", 8.0)),
        live=bool(runner_cfg.get("live", True)),
        settle_band_c=float(runner_cfg.get("settle_band", 1.0)),
        verbose=bool(runner_cfg.get("verbose", False)),
        plot_cfg=plot_cfg,
        metrics_cfg=metrics_cfg,
    )
    return catalog, cfg


@dataclass
class SessionState:
    session: Optional[PicoSession] = None
    chosen_port: Optional[str] = None
    port_arg: Optional[str] = None
    baud: Optional[int] = None


class RunnerRuntime:
    def __init__(self, config_path: Path):
        self.config_path = Path(config_path)
        self.state = SessionState()

    def load_context(self) -> Tuple[List[ExperimentSpec], RunnerConfig]:
        return _build_runner_context(self.config_path)

    def invalidate_session(self):
        if self.state.session is not None:
            try:
                self.state.session.close()
            except Exception:
                pass
        self.state = SessionState()

    def close(self):
        self.invalidate_session()

    def ensure_connected(self, cfg: RunnerConfig) -> PicoSession:
        need_reconnect = (
            self.state.session is None
            or self.state.port_arg != str(cfg.port)
            or self.state.baud != int(cfg.baud)
        )
        if need_reconnect:
            self.invalidate_session()
            session, chosen_port = connect_session(cfg)
            self.state.session = session
            self.state.chosen_port = chosen_port
            self.state.port_arg = str(cfg.port)
            self.state.baud = int(cfg.baud)
            _emit_lab("connected: %s" % str(chosen_port))
        if self.state.session is None:
            raise RuntimeError("Session owner is not connected")
        return self.state.session


def _run_recipe_path(runtime: RunnerRuntime, choice: str):
    # Reload config right before run so experiment edits are picked up without restarting script.
    catalog, cfg = runtime.load_context()
    exp = catalog_lookup(catalog, choice)
    session = runtime.ensure_connected(cfg)
    run_one(session, cfg, exp)


@dataclass
class TerminalHostState:
    plot_enabled: bool = True
    metrics_enabled: bool = True
    telemetry_visible: bool = False


def _cfg_with_terminal_overrides(cfg: RunnerConfig, host_state: TerminalHostState) -> RunnerConfig:
    out = replace(cfg, live=bool(host_state.plot_enabled))
    mc = dict(out.metrics_cfg or {})
    mc["show_window"] = bool(host_state.metrics_enabled)
    out = replace(out, metrics_cfg=mc)
    return out


def _print_terminal_help():
    _emit_lab("help:")
    _emit_lab("  h                 help")
    _emit_lab("  c                 catalog")
    _emit_lab("  e <id|exp_id>     run experiment")
    _emit_lab("  s                 stop active firmware run")
    _emit_lab("  k                 firmware check")
    _emit_lab("  u                 host status (overrides/session/config)")
    _emit_lab("  x                 reconnect serial session")
    _emit_lab("  telemetry/plot/metrics are fixed: telemetry=off, plot=on, metrics=on")
    _emit_lab("  b                 terminal home/refresh")
    _emit_lab("  q                 quit runner")
    _emit_lab("  other input       sent directly to firmware")


def _reconnect_terminal_session(runtime: RunnerRuntime, cfg: RunnerConfig) -> Optional[PicoSession]:
    runtime.invalidate_session()
    try:
        return runtime.ensure_connected(cfg)
    except Exception as ex:
        _emit_lab("run error: %s" % ex)
        return None


def _terminal_direct_experiment(command: str) -> ExperimentSpec:
    cmd = str(command).strip().lower()
    if cmd == "model":
        kind = "fopdt"
    elif cmd == "tune":
        kind = "tuning"
    else:
        kind = "standard"
    completion_cfg: Dict[str, object] = {}
    if cmd == "monitor":
        # Monitor has no firmware FINISH token; keep host run open until explicit stop.
        completion_cfg = {
            "mode": "token",
            "done_tokens": [],
            "require_prompt": False,
            "stop_on_done": False,
            "timeout_s": 86400.0,
            "stop_at_s": None,
        }
    else:
        # For control/tune/model rely on firmware FINISH token path.
        completion_cfg = {
            "mode": "token",
            "done_tokens": list(DONE_TOKENS),
            "require_prompt": True,
            "stop_on_done": False,
            "timeout_s": 7200.0,
            "stop_at_s": None,
        }
    return ExperimentSpec(
        number=-1,
        exp_id="LAB_TERMINAL_%s" % cmd.upper(),
        shortname="terminal_%s" % cmd,
        description="Terminal: %s" % cmd.upper(),
        base_params={},
        kind=kind,
        plot_cfg={},
        metrics_cfg={},
        completion_cfg=completion_cfg,
        run_command=cmd,
    )


def _run_terminal_path(runtime: RunnerRuntime, cfg: RunnerConfig):
    session = runtime.ensure_connected(cfg)
    session.clear_pending_io()
    host_state = TerminalHostState()
    run_active = False
    while True:
        try:
            cmd = input("lab> ").strip()
        except KeyboardInterrupt:
            _emit_lab("exit requested (Ctrl+C)")
            return "back"
        if not cmd:
            continue
        if cmd.lower() in ("b", "back"):
            session.clear_pending_io()
            _emit_lab("terminal home (h=help, c=catalog, e <id>, q=quit)")
            continue
        if cmd.lower() in ("q", "quit", "exit"):
            _emit_lab("exit")
            return "quit"
        parts = [p for p in cmd.split(" ") if p]
        op = parts[0].lower()
        if op in ("h", "help"):
            _print_terminal_help()
            continue
        if op in ("c", "catalog"):
            try:
                catalog, _ = runtime.load_context()
                print_catalog(catalog)
            except Exception as ex:
                _emit_lab("run error: %s" % ex)
            continue
        if op in ("s", "stop") and len(parts) == 1:
            if not run_active:
                _emit_lab("no active run")
                continue
            try:
                _emit_lab("stop requested, sending 'stop'...")
                session.stop_run(timeout_s=8.0)
                run_active = False
            except Exception as ex:
                _emit_lab("run error: %s" % ex)
                rs = _reconnect_terminal_session(runtime, cfg)
                if rs is None:
                    return "back"
                session = rs
            continue
        if op in ("k", "check") and len(parts) == 1:
            try:
                def _line_router_check(line: str):
                    if parse_telemetry_line(line) is not None:
                        return
                    _emit_fw(line)

                session.send_cmd_stream_wait_prompt("check", timeout_s=max(8.0, cfg.timeout_s + 4.0), line_cb=_line_router_check)
            except Exception as ex:
                _emit_lab("run error: %s" % ex)
                rs = _reconnect_terminal_session(runtime, cfg)
                if rs is None:
                    return "back"
                session = rs
            continue
        if op in ("u", "status") and len(parts) == 1:
            _emit_lab("status: config=%s port=%s telemetry=off plot=on metrics=on"
                      % (
                          str(runtime.config_path.resolve()),
                          str(runtime.state.chosen_port or "n/a"),
                      ))
            continue
        if op in ("x", "reconnect") and len(parts) == 1:
            rs = _reconnect_terminal_session(runtime, cfg)
            if rs is None:
                return "back"
            session = rs
            _emit_lab("reconnected")
            continue
        if op in ("t", "telemetry", "p", "plot", "m", "metrics"):
            _emit_lab("fixed mode: telemetry=off, plot=on, metrics=on")
            continue
        if op in ("e", "experiment", "run"):
            if len(parts) < 2:
                _emit_lab("usage -> e <id|exp_id>")
                continue
            key = parts[1].strip()
            try:
                catalog, fresh_cfg = runtime.load_context()
                exp = catalog_lookup(catalog, key)
                session = runtime.ensure_connected(fresh_cfg)
                run_cfg = _cfg_with_terminal_overrides(fresh_cfg, host_state)
                run_one(session, run_cfg, exp)
            except Exception as ex:
                _emit_lab("run error: %s" % ex)
                rs = _reconnect_terminal_session(runtime, cfg)
                if rs is None:
                    return "back"
                session = rs
            continue
        try:
            first_word = cmd.split(" ", 1)[0].strip().lower()
            if first_word in ("control", "tune", "model", "monitor"):
                run_active = True
                run_cfg = _cfg_with_terminal_overrides(cfg, host_state)
                exp = _terminal_direct_experiment(first_word)
                params = {"TELEMETRY_MODE": "NORMAL"}
                run_single_experiment(
                    session,
                    exp,
                    params,
                    run_cfg,
                    plot_cfg=run_cfg.plot_cfg,
                    hold_live_at_end=bool(run_cfg.live),
                )
                run_active = False
            else:
                def _line_router(line: str):
                    if (not host_state.telemetry_visible) and (parse_telemetry_line(line) is not None):
                        return
                    _emit_fw(line)
                timeout_s = max(8.0, cfg.timeout_s + 4.0)
                session.send_cmd_stream_wait_prompt(cmd, timeout_s=timeout_s, line_cb=_line_router)
        except KeyboardInterrupt:
            _emit_lab("interrupt -> stop requested")
            try:
                session.stop_run(timeout_s=8.0)
                run_active = False
            except Exception as stop_ex:
                _emit_lab("warning: stop failed: %s" % stop_ex)
        except Exception as ex:
            _emit_lab("run error: %s" % ex)
            rs = _reconnect_terminal_session(runtime, cfg)
            if rs is None:
                return "back"
            session = rs
            run_active = False


def interactive_loop(config_path: Path):
    runtime = RunnerRuntime(config_path)
    while True:
        try:
            catalog, cfg = runtime.load_context()
        except Exception as ex:
            _emit_lab("config error: %s" % ex)
            _emit_lab("fix config file and press Enter to retry (or type q to exit)")
            ans = input("Retry? (Enter/q): ").strip().lower()
            if ans in ("q", "quit", "exit"):
                break
            continue
        if not catalog:
            _emit_lab("experiment catalog: (empty)")
            _emit_lab("no experiments configured. Add entries under 'experiments' in the config file")
            ans = input("Retry? (Enter/q): ").strip().lower()
            if ans in ("q", "quit", "exit"):
                break
            continue

        try:
            runtime.ensure_connected(cfg)
        except Exception as ex:
            _emit_lab("run error: %s" % ex)
            ans = input("Retry connection? (Enter/q): ").strip().lower()
            if ans in ("q", "quit", "exit"):
                break
            continue

        _emit_lab("terminal ready (h=help, c=catalog, e <id>, q=quit)")
        try:
            action = _run_terminal_path(runtime, cfg)
            if action == "quit":
                break
        except KeyboardInterrupt:
            _emit_lab("interrupt received; restarting terminal")
        except Exception as ex:
            _emit_lab("run error: %s" % ex)
            # If serial transport is broken, force reconnect for next selection.
            runtime.invalidate_session()
    runtime.close()


def run_one(session: PicoSession, cfg: RunnerConfig, exp: ExperimentSpec):
    # Ensure we are at command prompt before applying parameters.
    session.sync_prompt(timeout_s=max(8.0, cfg.timeout_s + 4.0))
    effective_plot_cfg = merge_plot_cfg(cfg.plot_cfg, exp.plot_cfg)
    effective_metrics_cfg = merge_metrics_cfg(cfg.metrics_cfg, exp.metrics_cfg)
    cfg_run = replace(cfg, metrics_cfg=effective_metrics_cfg)
    params = apply_overrides(exp.base_params, config_params=None)

    if exp.kind == "sweep":
        run_param_sweep(session, exp, params, cfg_run, plot_cfg=effective_plot_cfg)
    else:
        run_single_experiment(
            session,
            exp,
            params,
            cfg_run,
            plot_cfg=effective_plot_cfg,
            hold_live_at_end=bool(cfg_run.live),
        )


def main():
    config_path = _default_config_path().resolve()
    _emit_lab("config: %s" % str(config_path))
    interactive_loop(config_path)


if __name__ == "__main__":
    main()
