# Contributing

## Scope
This repository is an educational RP2040/MicroPython control lab. Keep changes student-friendly, deterministic, and safe for hardware use.

## Development Guidelines
- Keep runtime firmware changes in `firmware/`.
- Keep MATLAB tooling in `tools/matlab/`.
- Keep docs in `docs/`.
- Avoid changing behavior in multiple control modes at once unless required.
- Prefer clear parameter names and conservative defaults for thermal safety.

## Safety Requirements
- Do not remove or bypass temperature cutoff and safety hold logic.
- Document any change that affects heater output limits or safety thresholds.

## Pull Request Checklist
- [ ] Code and docs are consistent.
- [ ] Python files compile (`python3 -m py_compile firmware/*.py runner/lab.py`).
- [ ] New config options are validated in `firmware/profile.py`.
- [ ] README/docs updated for any user-visible change.
- [ ] No generated artifacts committed (`__pycache__`, CSV logs, plots, Simulink temp files).

## Release Checklist (Public GitHub)
- [ ] Firmware/runner compatibility verified (`runner/lab.py` + `runner/lab.yaml` recipes).
- [ ] Policy check passes (`python3 tools/python/check_project_policy.py`).
- [ ] Recipe catalog titles/descriptions reflect current firmware command flow (`control`/`tune`/`model`/`monitor`).
- [ ] Hardware smoke validation archived (maintainer-run on real board).

## Commit Style (recommended)
Use focused commits with imperative subjects, for example:
- `Fix CSV logging path in docs`
- `Add R2015a-compatible MATLAB plotting`
