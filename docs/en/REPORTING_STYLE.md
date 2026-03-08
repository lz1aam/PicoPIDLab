# Reporting Style (Firmware + Lab)

Scope
- Applies to runtime reporting in:
  - `firmware/*.py`
  - `runner/lab.py`

Message classes
- `# PHASE:` run stage started.
- `# INFO:` neutral context.
- `# WARNING:` non-fatal issue.
- `# ERROR:` fatal issue for current action.
- `# RESULT:` computed/reportable values.
- `# FINISH:` procedure-complete token.
- `# READY:` idle prompt token.

Block format
- Classed blocks use:
  - class header line (for example `# RESULT:`),
  - separator line `# ========================================`,
  - detail lines prefixed with `# `,
  - closing separator line.
- Do not use alternative separators.

Ownership
- Firmware-origin lines use `# ...`.
- Host-runner-origin lines use `LAB: ...`.
- Host must not emit synthetic `# READY:` lines.

Redundancy rules
- Do not repeat equivalent information in adjacent lines.
- Prefer one source line for runtime context:
  - `runtime context: telemetry=<...> Ts=<...>s run_s=<...|none>`
- Avoid host post-summary lines when firmware already reported the same result block.

Telemetry rows
- Runtime telemetry rows are not classed messages:
  - `PV:xx.x SP:xx.x OP:xx.x`
  - MPC: `PV:xx.x SP:xx.x OP:xx.x YH:xx.x YP:xx.x`

Stability
- `# FINISH: done` is a parser token and must remain stable unless firmware and host parser are updated together.
- `# READY:` semantics must remain stable for prompt synchronization.
