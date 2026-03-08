# Changelog

All notable changes to this project will be documented in this file.

The format is based on Keep a Changelog, and this project follows Semantic Versioning.

## [1.2.5] - 2026-03-06

### Added
- Added control-path disturbance parameters in firmware profile and CLI views:
  - `DIST_ENABLE`, `DIST_MODE`, `DIST_MAG_PCT`, `DIST_START_S`, `DIST_DURATION_S`
- Added disturbance formula registry entry in `docs/en/CONTROL_FORMULAS.md`.
- Added explicit disturbance defaults to control recipes in `tools/python/picopid_experiments_config.yaml` (disabled by default).

### Changed
- Control-loop status/event logging is now deferred from the control-step branch to the UI branch with a timing guard, reducing risk of missed nominal control slots while preserving telemetry output.
- Bumped firmware version to `1.2.5`.

### Documentation
- Updated docs for current runtime command flow (`control`/`tune`/`model`/`monitor`, no `set` command).
- Updated docs to reflect active model source as `MODEL_*` runtime values (no `model.json` persistence path in firmware).
- Updated telemetry naming consistency (`NORMAL` instead of `FULL`) and version tags in EN/BG quickstart/readme documents.

## [1.2.3] - 2026-03-02

### Fixed
- Unified stop/abort behavior across firmware execution paths so `stop` reliably interrupts:
  - main control loop modes (`PID`, `ONOFF`, `FUZZY`, `MPC`)
  - monitor mode
  - `FOPDT` identification mode
  - PID relay autotune preheat/cycle phases
- Added coordinated abort propagation from autotune into startup/control orchestration to ensure clean return to `READY`.

### Changed
- Standardized firmware runtime/report messages across main control, factory, FOPDT, autotune, and terminal interfaces with explicit tags:
  - `# INFO:`
  - `# PHASE:`
  - `# RESULT:`
  - `# WARNING:`
  - `# ERROR:`
- Improved experiment runner workflow:
  - reloads config/catalog before menu display and again before run start (recipe edits apply without restarting script)
  - keeps live figures open without blocking CLI (supports side-by-side run comparison)
- Bumped firmware version to `1.2.3`.

## [1.2.2] - 2026-02-27

### Changed
- Removed RP2040 watchdog support from firmware runtime and profile configuration.
- Removed watchdog-related wiring from `main.py`, `factory.py`, `autotune.py`, `fopdt.py`, and `terminal.py`.
- Simplified interactive startup flow by removing watchdog-specific prompt guards.

## [1.2.1] - 2026-02-27

### Fixed
- Fixed watchdog reset regressions by feeding WDT in pre-start terminal loop, autotune loops, and FOPDT loops.
- Fixed startup integration so `wdt` is passed through `main.py` into terminal/factory/autotune/FOPDT flows.
- Fixed watchdog validation to require `WATCHDOG_TIMEOUT_MS == 0` (disabled) or `>= 1000`.

### Changed
- Updated indicator documentation to explicitly describe absolute-band precedence and `% of |SP|` fallback behavior.
- Added gain-scheduled PID doc note clarifying possible output bump during schedule transitions.
- Tightened `FOPDT_METHOD` validation to canonical values (`SMITH`, `SK`) for clearer behavior.
- Bumped firmware version to `1.2.1`.

## [1.2.0] - 2026-02-21

### Added
- Added `firmware/factory.py` for controller construction and mode registry.
- Added `firmware/fopdt.py` for FOPDT test execution and model load/save helpers.
- Added `firmware/autotune.py` for relay autotune routines.
- Added `firmware/version.py` as an authoritative firmware version source.
- Added `ControllerBase` interface in `firmware/controllers.py` and applied it to all controllers.

### Changed
- Refactored `firmware/main.py` into an orchestrator-only role:
  - controller creation now delegates to `factory.build_controller(...)`
  - FOPDT flow now delegates to `fopdt.run_test(...)` and `fopdt.save_model(...)`
- Replaced control-mode `if/elif` orchestration with registry-driven mode dispatch in `firmware/factory.py`.
- `firmware/profile.py` now derives `ALLOWED_CONTROL_MODES` from the factory mode registry (with safe fallback).
- Startup banner now prints explicit firmware version (`PicoPID Lab v...`).

### Fixed
- Removed spurious extra blank line in `profile.validate()`.

## [1.1.2] - 2026-02-21

### Changed
- Normalized active project filenames to one-word naming across firmware, docs, terminal tooling, and MATLAB scripts.
- Renamed firmware modules:
  - `firmware/config_hw.py` -> `firmware/hardware.py`
  - `firmware/control_profiles.py` -> `firmware/profile.py`
- Renamed terminal tool:
  - `tools/terminal/pico_terminal_logger.py` -> `tools/terminal/picologger.py`
- Renamed MATLAB scripts:
  - `tools/matlab/analyze_experiment.m` -> `tools/matlab/analyze.m`
  - `tools/matlab/compare_real_vs_sim.m` -> `tools/matlab/compare.m`
  - `tools/matlab/create_pid_simulink_model.m` -> `tools/matlab/modeler.m`
- Renamed MATLAB integration documentation file:
  - `docs/matlab_integration_guide.md` -> `docs/matlabguide.md`
- Renamed conduct policy filename to one-word form:
  - `CODE_OF_CONDUCT.md` -> `codeofconduct.md`
- Updated all imports, MATLAB primary function names, and documentation references to match renamed files.

## [1.1.1] - 2026-02-21

### Added
- Added `firmware/terminal.py` to isolate terminal command parsing/help/settings from the control loop.
- Added `firmware/utils.py` as a shared utility module for common runtime helpers.

### Changed
- Refactored `firmware/main.py` to use `terminal` for pre-run command handling and in-run nonblocking command polling.
- Reduced `firmware/main.py` size by moving CLI and runtime-parameter interface code out of the core control-loop file.
- Runtime `set <PARAM> <VALUE>` now validates immediately and rolls back invalid values with a clear error.
- RP2040 GPIO drive-strength setting now uses capability checks/exception safety rather than relying on `sys.platform == "rp2"`.
- `SmithPredictorPI` dead-time history now uses typed `array('f')` storage when available (fallback to list).

### Fixed
- Eliminated duplicated `_isfinite` and `_clamp` implementations across firmware modules by centralizing helpers in `firmware/utils.py`.
- Preserved a clear runtime guard for `SMITH_PI` requirements (`TUNING_METHOD='AUTO'` and `AUTO_METHOD='FOPDT_IMC'`).
- Cleaned minor style issue around `_mf5()` placement/spacing in `firmware/controllers.py`.

## [1.0.0] - 2026-02-15

### Added
- Introduced runtime CSV logging controls in `firmware/profile.py`:
  - `LOG_ENABLE`
  - `LOG_EVERY_N`
  - `LOG_VARIABLES`
- Added validation for logging configuration in `firmware/profile.py`.
- Added repository governance and release files:
  - `.gitignore`
  - `LICENSE` (MIT)
  - `CONTRIBUTING.md`
  - `codeofconduct.md`
- Added repository layout and release-readiness checklist to `README.md`.

### Changed
- Reorganized project structure for maintainability:
  - firmware runtime code moved to `firmware/`
  - MATLAB/Python tooling moved to `tools/matlab/`
  - docs consolidated in `docs/`
  - reference data moved to `data/reference/`
  - historical backup moved to `archive/backup/`
- Integrated CSV logger into runtime paths in `firmware/main.py`:
  - main control loop output path
  - FOPDT mode output path
  - proper logger close in shutdown/finalization
- Updated command/path references across docs to match current layout.
- Replaced oversized/stale MATLAB integration guide with a concise, current version in `docs/matlabguide.md`.

### Fixed
- Fixed documented logging workflow mismatch where docs referenced CSV mode not wired in runtime.
- Fixed capture workflow path mismatches (`capture_data.py` path now consistently under `tools/matlab/`).
- Improved MATLAB R2015a compatibility in analysis/simulation scripts:
  - removed dependency on `yline`
  - removed dependency on `sgtitle`
  - added robust To Workspace signal extraction handling in `compare.m`
  - hardened Simulink block parameter setting in `modeler.m`

## [1.1.0] - 2026-02-20

### Added
- Added a unified PC-side interactive terminal/logger tool:
  - `tools/terminal/picologger.py`
  - supports command pass-through (`params`, `set`, `check`, `start`, `stop`, `restart`, `exit`)
  - supports CSV logging in the same serial session (`--csv`, `/log start`, `/log stop`)
- Added structured firmware help for runtime configuration:
  - topic help (`help`, `help settings`, `help <group>`)
  - grouped parameter views (`params active`, `params all`, `params <group>`)

### Changed
- Firmware startup/control workflow is terminal-driven:
  - waits for `start` before running
  - supports `stop` and `restart` while running
  - supports `set`/`check` parameter workflow between runs
- Runtime output format standardized for readability:
  - `PV:xx.x SP:xx.x OP:xx.x`
  - one decimal in human stream values
- Header/unit presentation updated for readability:
  - plotter header uses `PV[°C], SP[°C], OP[%]`
  - setpoint line uses `°C` and preserves user-entered numeric representation
- Documentation updated across `README.md`, `docs/en/QUICKSTART.md`, and `docs/matlabguide.md` for the terminal-first workflow.

### Removed
- Removed demo/lesson mode from active firmware and profile configuration.
- Removed legacy firmware-side CSV logging hook path:
  - deleted `firmware/logger.py`
  - removed `LOG_ENABLE`/`LOG_EVERY_N`/`LOG_VARIABLES` from active profile flow
- Removed deprecated capture script:
  - deleted `tools/matlab/capture_data.py`

### Fixed
- Fixed runtime restart behavior to reinitialize session cleanly before next `start`.
- Fixed output precision consistency in runtime/safety prints.
- Fixed multiple MATLAB tooling robustness issues:
  - settling-time correctness and descending-response handling in `analyze.m`
  - NRMSE/R² edge-case handling and clearer workspace-signal errors in `compare.m`
  - version-tolerant scope parameter setting in `modeler.m`

## [Unreleased]
- No unreleased changes.
