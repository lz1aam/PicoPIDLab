# AGENT.md — PicoPID Lab

Read this before every change. It overrides all other instructions.

---

## What this project is

An educational temperature-control lab for university students, running **MicroPython on RP2040**
(Raspberry Pi Pico). Students edit exactly one file — `firmware/config.py` — and observe
how PV/SP/OP behave. The PC runner (`runner/lab.py`) drives experiments
from YAML recipes and saves CSV + plots for MATLAB analysis.

The hardware constraints are real and non-negotiable:

- ~200 KB usable RAM. Every extra class, list, and string costs memory that cannot be reclaimed.
- No JIT. Every unnecessary function call runs slower than it looks.
- Control loop period is 250 ms (TS_S = 0.25). If your code is too slow, temperature control breaks.
- Students must be able to read the code and understand what it does. Cleverness is a bug.

Lightweight code is not a preference here. It is a hard requirement:

- Prefer fewer lines, fewer branches, fewer objects, and fewer moving parts.
- Do not add architecture layers unless they remove more code than they add.
- Do not add wrapper classes, config objects, dataclasses, registries, or adapters for firmware code unless explicitly requested.
- Derived values must be computed once at run start when possible, not recomputed inside hot control paths.
- If a simpler structure can do the job, use the simpler structure.
- Follow KISS aggressively. If a feature or refactor makes the code harder to trace, more stateful, or more layered, it is probably wrong.
- Reuse existing helpers, parsing paths, and plotting/finalization paths before adding new ones. New helpers are justified only when they remove duplication or reduce hot-path cost.
- Before adding any new function, object, or branch, check whether the same job is already done elsewhere in the project and whether that path can be reused or slightly generalized instead.
- Every feature/refactor/simplification pass must include a short cleanup check: remove dead helpers, collapse thin wrappers, and delete temporary compatibility code that is no longer needed.
- On RP2040-facing firmware, optimize for RAM first, then flash size, then aesthetics. Avoid new dicts, mirrored views, temporary containers, or rich return structures when a few scalars or a tuple will do.
- On runner/host code, keep orchestration logic and analysis/report logic unified around existing code paths when possible. Do not create special-case behavior for one artifact if the same lifecycle can be shared by all artifacts.

---

## Before writing any code, answer these three questions

**1. Is the final file shorter than before?**
Simplification tasks must produce fewer lines. If you added lines, you did not simplify.

**2. Does every value that affects algorithm behavior live in `config.py`?**
If a student cannot see it and change it, it must not exist as a parameter.

**3. Does this do exactly what was asked — no more?**
No alternatives. No "just in case" variants. No fallback paths. No wrapper classes.

**4. Did I reuse the existing code path before adding a new one?**
If not, explain why reuse would be incorrect, slower, or more memory-hungry.

**5. After the change, what can be deleted or merged?**
Every new feature and every refactor must be followed by a sweep for obsolete helpers, duplicate logic, and one-off behavior.

---

## Mandatory pre-change gate

Before any structural edit, refactor, shared-helper change, new file, or cross-file formula/reporting change,
stop and pass this gate explicitly.

You must check all five items:

1. **Which file owns this logic?**
   - Name the owner file first.
   - If two files need the same logic, prefer moving it into the owning file.
   - Do not create a new shared file by reflex.

2. **Can the existing path be reused?**
   - Check for an existing helper/path first.
   - Reuse or slightly generalize that path unless it would be slower, more memory-hungry, or conceptually wrong.

3. **Does this need a new file?**
   - New firmware files require explicit user approval first.
   - New runner/docs/tools files still require registry updates in this document in the same change.
   - If a new file is not clearly justified, do not add it.

4. **Does this respect RP2040 constraints?**
   - Prefer scalars/tuples over dicts/objects in firmware.
   - Prefer one derived computation at run start over repeated or mirrored runtime state.
   - Avoid temporary containers in low-memory or post-run firmware paths.

5. **What can be deleted after this change?**
   - Remove stale wrappers, compatibility branches, duplicated conversions, and dead helpers in the same change.

Hard rule:
- If the change touches firmware structure, control math ownership, runner/firmware interface shape,
  or adds a file, re-read this `AGENTS.md` before editing.

Common failure patterns to avoid:
- creating a helper module because it looks clean in generic software terms
- duplicating logic in `main.py` because importing the owner path feels inconvenient
- adding special-case runner behavior for one artifact when an existing lifecycle can be reused
- keeping compatibility aliases or stale code after a refactor

---

## The firmware modules and what belongs in each

| File | Role | Rule |
|---|---|---|
| `config.py` | Student configuration | Every user-facing knob and static validation lives here. |
| `main.py` | Orchestrator | Startup, dispatch, safety, telemetry, run-state transitions. |
| `control.py` | Control domain | Controllers, PID interpretation, controller construction. |
| `identify.py` | Identification/tuning | FOPDT identification, relay tuning, model tuning. |
| `cli.py` | Runtime commands | Student-facing prompt, help, params, assignments. |
| `hardware.py` | Board I/O | Thermistor, heater PWM, WS2812, shared timing helpers. |

---

## Expanded ownership map

Use this table before moving logic or creating helpers.

### Firmware owners

- `firmware/config.py`
  - owns all student-facing knobs, allowed values, and validation
  - owns curriculum-visible defaults
  - does **not** own derived runtime state, report formatting, or controller math

- `firmware/main.py`
  - owns runtime orchestration only:
    - startup flow
    - run dispatch
    - control-loop scheduling
    - safety shutdown behavior
    - telemetry emission
    - run-state transitions
  - may coordinate owned helpers from other modules
  - must not become a second owner of controller math, tuning formulas, or CLI formatting

- `firmware/control.py`
  - owns controller classes
  - owns PID-form conversion and PID interpretation math
  - owns controller construction/resolution from configured state
  - if a PID conversion is needed elsewhere, this is the first place to reuse from

- `firmware/identify.py`
  - owns FOPDT identification
  - owns relay/model tuning formulas and experiment-local steady-state logic
  - owns tuning-side computed results before they are handed back to runtime orchestration
  - does not own prompt/session behavior or runtime command flow

- `firmware/cli.py`
  - owns prompt commands, help text, params views, and machine-readable runtime snapshots
  - owns terminal/report formatting grammar
  - if runner parsing depends on firmware output, this file is the interface owner

- `firmware/hardware.py`
  - owns hardware access and low-level board helpers
  - owns timing/yield helpers that are hardware/runtime primitives
  - does not own control formulas, CLI logic, or experiment interpretation

### Runner owners

- `runner/lab.py`
  - owns host orchestration, serial command flow, run directories, plot generation, and artifact lifecycle
  - owns host-side analysis that exists only to interpret/summarize experiment results
  - should reuse one artifact lifecycle for plots/metrics whenever possible
  - should not scrape student-facing firmware formatting if a stable machine-readable firmware path can exist

- `runner/lab.yaml`
  - owns experiment recipes only
  - does not own fallback semantics that contradict firmware `config.py`
  - if a parameter is renamed/removed in firmware, this file must be updated in the same change

### Docs owners

- `README.md`
  - owns public front-page project explanation and curated showcase
  - should show stable assets/examples, not raw transient run folders

- `docs/en/CONTROL_FORMULAS.md`
  - owns canonical formula documentation for code-visible control math
  - any new control formula in firmware/runner must be documented here in the same change

- localized docs
  - own translation/adaptation of the canonical behavior
  - must not drift from EN on terminology or supported workflow

### Policy/TODO owners

- `policies/AGENTS.md`
  - owns project-specific coding constraints and file ownership rules

- `TODO/PROJECT_TODO.md`
  - owns the project task list
  - do not create parallel todo trackers

### Cross-file ownership rules

- If `main.py` and another firmware module need the same formula:
  - first move/reuse it in the owning module
  - do not create a new firmware file unless explicitly approved

- If runner and firmware need a shared interface:
  - prefer a stable firmware-owned interface in `cli.py`
  - do not make runner depend on human-only formatting unless there is no better option

- If multiple plots/artifacts need the same save/show/close behavior:
  - unify in the runner artifact lifecycle
  - do not add per-plot special behavior unless the user explicitly wants it
  - saved artifacts and real interactive plots are different lifecycles; do not collapse them into one path just to reduce helper count
  - saved artifacts must not open extra windows by default
  - keep-open behavior applies only to real interactive plots, not to saved artifact figures

---

## Firmware boundary hard rules

These are non-negotiable and override style preferences.

- `control.py`:
  - owns controller classes, PID form interpretation, and controller construction
  - no direct hardware access
  - no CLI reads
  - no control-loop scheduling or command handling
  - imports `identify.py` lazily only where model lookup is needed
- `identify.py`:
  - owns FOPDT identification, relay tuning, and model-based tuning
  - no prompt loop ownership
  - no READY/session-header ownership
  - may use shared timing helpers from `hardware.py`
- `main.py`:
  - owns runtime flow, scheduling, commands, safety, telemetry, and run-state transitions
  - may import `control.py` and `identify.py` lazily at action start to reduce boot memory
  - must not reimplement controller math or identification formulas
  - must treat `config.py` as configured input, not a scratchpad for hidden runtime state

If logic is needed by both `main.py` and another firmware module, first ask whether one of them owns it. Do not create a shared helper by reflex.

---

## Lightweight firmware design rules

Firmware code must stay brutally simple.

- Prefer plain functions, literals, tuples, and existing controller instances.
- Avoid new classes in firmware unless the class directly represents a controller or a hardware device.
- Avoid dictionaries carrying large mirrored views of `config.py`.
- Avoid creating a "runtime spec", "config model", "descriptor object", or similar rich structure for firmware unless explicitly requested.
- If reporting needs derived values, prefer a short tuple/list of lines or a tiny fixed-shape tuple over a new object graph.
- Do not write hidden derived/runtime values back into `config.py`.
- Do not keep both configured values and derived equivalents in long-lived memory unless there is a proven runtime need.
- Do not move code into a helper just to make files look more modular. Fewer total branches and lines matters more than formal layering.
- Prefer scalar math and direct assignment over helper calls that allocate dicts or descriptor objects in hot or low-memory paths.
- If a post-run/reporting path allocates noticeable temporary structures, treat that as a real memory problem, not as harmless cleanup code.

When proposing a firmware refactor, justify it in terms of RAM, hot-path cost, line count, or student readability. "Cleaner architecture" by itself is not enough.

---

## Reuse and cleanup rules for every change

These apply to firmware, runner, MATLAB helpers, and docs.

- New feature work must not leave the codebase with extra wrappers or duplicated behavior that could have been merged into an existing path.
- Refactors must reduce conceptual surface area, not only move code around.
- If a new artifact, report, or plot is introduced, check whether its save/show/finalize lifecycle can reuse the same path as existing artifacts.
- If a new parser or formatter is introduced, check whether an existing parser/formatter can be extended instead of duplicated.
- After implementing a change, perform a short sweep for:
  - dead helpers
  - wrappers that now only forward arguments
  - duplicate formatting/plotting/reporting code
  - duplicated conversions between equivalent forms
- If that sweep finds removable code, remove it in the same change.

---

## Firmware file registry (must stay complete)

Every firmware file must be listed here with an owner.
If you create a new project file, add it to this registry in the same change with an owner.
Do not leave unowned files.

| File | Owner |
|---|---|
| `firmware/hardware.py` | firmware/core |
| `firmware/control.py` | firmware/control |
| `firmware/cli.py` | firmware/runtime |
| `firmware/config.py` | firmware/curriculum |
| `firmware/identify.py` | firmware/control |
| `firmware/main.py` | firmware/runtime |

---

## Project file registry (runner + recipes)

These files are part of the same system and must be kept in sync with firmware behavior.

| File | Owner |
|---|---|
| `runner/lab.py` | tools/experiments |
| `runner/lab.yaml` | tools/experiments |
| `TODO/PROJECT_TODO.md` | project/runtime |
| `policies/RUNNER_UI_RULES.md` | tools/experiments |
| `tools/python/matlab_live_capture.py` | tools/experiments |
| `tools/matlab/lab_config.m` | tools/matlab |
| `tools/matlab/lab_runner.m` | tools/matlab |
| `tools/matlab/pico_serial_open.m` | tools/matlab |
| `tools/matlab/pico_serial_close.m` | tools/matlab |
| `tools/matlab/pico_serial_cmd.m` | tools/matlab |
| `tools/matlab/pico_serial_read_lines.m` | tools/matlab |
| `tools/matlab/pico_serial_list_ports.m` | tools/matlab |
| `tools/matlab/pico_serial_cleanup_port.m` | tools/matlab |
| `tools/matlab/pico_serial_reset.m` | tools/matlab |
| `tools/matlab/pico_parse_telemetry_line.m` | tools/matlab |
| `tools/matlab/auto_fit_pi_from_csv.m` | tools/matlab |
| `tools/matlab/mode_modeler.m` | tools/matlab |
| `tools/matlab/build_fuzzy_thermal_slx.m` | tools/matlab |
| `tools/matlab/fuzzy_ctrl_core.m` | tools/matlab |
| `tools/matlab/fuzzy_evalfis_core.m` | tools/matlab |
| `tools/matlab/run_fuzzy_thermal_octave.m` | tools/matlab |
| `tools/matlab/mpc_ctrl_core.m` | tools/matlab |
| `tools/matlab/smith_ctrl_core.m` | tools/matlab |
| `tools/matlab/exercises/ex_pid_parallel_compare.m` | tools/matlab |
| `tools/matlab/exercises/ex_fopdt_analyze.m` | tools/matlab |
| `tools/matlab/exercises/ex_autotune_review.m` | tools/matlab |
| `tools/matlab/exercises/ex_pid_live_compare.m` | tools/matlab |
| `tools/matlab/exercises/ex_mode_model_compare.m` | tools/matlab |

Rule:
- If you edit any firmware file or either experiments file, you must verify full compatibility between:
  - firmware parameter names/accepted values/validation rules, and
  - `runner/lab.py` + `runner/lab.yaml`.
- No silent drift is allowed. Update all affected files in the same change.
- All MATLAB/Simulink code in `tools/matlab` must remain compatible with MATLAB/Simulink R2015a or newer.
  Do not use APIs introduced after R2015a unless explicitly approved.
- Never add or use command-line/script arguments unless the user explicitly requested arguments.
  If unrequested arguments were added, remove them immediately.
  If arguments are truly crucial, do not keep them by default: ask permission first or redesign a no-argument solution.

---

## Documentation file registry (must stay complete)

Every project documentation file must be listed here with an owner.
If you create or rename documentation, update this registry in the same change.

| File | Owner |
|---|---|
| `README.md` | docs/core |
| `CONTRIBUTING.md` | docs/core |
| `CHANGELOG.md` | docs/core |
| `codeofconduct.md` | docs/core |
| `docs/en/CONTROL_FORMULAS.md` | docs/control |
| `docs/en/QUICKSTART.md` | docs/core |
| `docs/en/CONFIG_TUTORIAL.md` | docs/core |
| `docs/en/SETTINGS_DEPENDENCIES.md` | docs/control |
| `docs/en/REPORTING_STYLE.md` | docs/runtime |
| `docs/en/GLOSSARY.md` | docs/core |
| `docs/en/HARDWARE_DRY_RUN_CHECKLIST.md` | docs/runtime |
| `docs/bg/ФОРМУЛИ_ЗА_УПРАВЛЕНИЕ.md` | docs/control |
| `docs/bg/БЪРЗ_СТАРТ.md` | docs/core |
| `docs/bg/РЪКОВОДСТВО_ЗА_CONFIG.md` | docs/core |
| `docs/bg/ЗАВИСИМОСТИ_НА_НАСТРОЙКИТЕ.md` | docs/control |
| `docs/bg/СТИЛ_НА_ОТЧЕТИ.md` | docs/runtime |
| `docs/bg/РЕЧНИК.md` | docs/core |
| `docs/bg/ЧЕКЛИСТ_ЗА_ХАРДУЕРЕН_DRY_RUN.md` | docs/runtime |
| `docs/de/REGELUNGSFORMELN.md` | docs/control |
| `docs/de/SCHNELLSTART.md` | docs/core |
| `docs/de/CONFIG_ANLEITUNG.md` | docs/core |
| `docs/de/EINSTELLUNGSABHAENGIGKEITEN.md` | docs/control |
| `docs/de/BERICHTSSTIL.md` | docs/runtime |
| `docs/de/GLOSSAR.md` | docs/core |
| `docs/de/HARDWARE_DRY_RUN_CHECKLISTE.md` | docs/runtime |
| `docs/es/FORMULAS_DE_CONTROL.md` | docs/control |
| `docs/es/INICIO_RAPIDO.md` | docs/core |
| `docs/es/GUIA_CONFIG.md` | docs/core |
| `docs/es/DEPENDENCIAS_DE_CONFIGURACION.md` | docs/control |
| `docs/es/ESTILO_DE_REPORTES.md` | docs/runtime |
| `docs/es/GLOSARIO.md` | docs/core |
| `docs/es/CHECKLIST_DRY_RUN_HARDWARE.md` | docs/runtime |
| `docs/uk/ФОРМУЛИ_КЕРУВАННЯ.md` | docs/control |
| `docs/uk/ШВИДКИЙ_СТАРТ.md` | docs/core |
| `docs/uk/ПОСІБНИК_CONFIG.md` | docs/core |
| `docs/uk/ЗАЛЕЖНОСТІ_НАЛАШТУВАНЬ.md` | docs/control |
| `docs/uk/СТИЛЬ_ЗВІТУВАННЯ.md` | docs/runtime |
| `docs/uk/ГЛОСАРІЙ.md` | docs/core |
| `docs/uk/ЧЕКЛІСТ_АПАРАТНОГО_DRY_RUN.md` | docs/runtime |
| `docs/it/FORMULE_DI_CONTROLLO.md` | docs/control |
| `docs/it/AVVIO_RAPIDO.md` | docs/core |
| `docs/it/GUIDA_CONFIG.md` | docs/core |
| `docs/it/DIPENDENZE_CONFIGURAZIONE.md` | docs/control |
| `docs/it/STILE_REPORT.md` | docs/runtime |
| `docs/it/GLOSSARIO.md` | docs/core |
| `docs/it/CHECKLIST_DRY_RUN_HARDWARE.md` | docs/runtime |
| `docs/fr/FORMULES_DE_COMMANDE.md` | docs/control |
| `docs/fr/DEMARRAGE_RAPIDE.md` | docs/core |
| `docs/fr/GUIDE_CONFIG.md` | docs/core |
| `docs/fr/DEPENDANCES_PARAMETRES.md` | docs/control |
| `docs/fr/STYLE_DE_REPORTING.md` | docs/runtime |
| `docs/fr/GLOSSAIRE.md` | docs/core |
| `docs/fr/CHECKLIST_DRY_RUN_MATERIEL.md` | docs/runtime |

---

## Policy file registry (local-only)

These files are local governance and are excluded from public GitHub sync.
If you add/rename policy files, update this registry in the same change.

| File | Owner |
|---|---|
| `policies/AGENTS.md` | policies/core |
| `policies/PUBLISH_SCOPE.md` | policies/core |
| `policies/bg/PUBLISH_SCOPE.bg.md` | policies/core |
| `policies/RUNNER_UI_RULES.md` | policies/runtime |
| `policies/PLATFORM_PLAN.md` | policies/runtime |

---

## Implementing a formula or calculation

Do exactly what was asked. Nothing more.

- **Control-theory hard rule (non-negotiable):**
  - For any control-loop algorithm/math change (PID, ON/OFF logic, FOPDT, IMC, MPC, relay tuning, observers, filters), use the canonical textbook form only.
  - Do not invent variants, shortcuts, "improvements", blended forms, heuristic rewrites, or hidden compensations unless explicitly requested.
  - Every touched formula must be verified against a cited textbook/paper equation before finalizing the change.
  - If exact reference verification is not possible, stop and ask; do not ship the change.
  - Every control-theory formula present in code must be documented in `docs/en/CONTROL_FORMULAS.md` with:
    formula name, symbolic equation, variable definitions/units, source citation (book/paper), and code location.
  - Any formula change must update both code and `docs/en/CONTROL_FORMULAS.md` in the same commit/change.
  - No undocumented control formulas are allowed in firmware or runner logic.

- Constants in a formula are **literals with a comment**. They are not parameters.
- If the formula has one correct textbook form, implement that form. Not alternatives.
- Do not add optional parameters with defaults. An optional parameter is a hidden knob.
- Do not add helper functions unless the identical logic appears in three or more places.
- Edge cases get a single guard and a comment. Not a knob.

```python
# WRONG — turned a constant into a hidden parameter
def compute(tau, theta, scale=1.0):   # where did scale come from?

# CORRECT — textbook formula, constants are literals
def compute(tau, theta):
    lambda_s = max(tau, theta)        # IMC: lambda = max(tau, theta), Rivera-Morari
    return tau / (K * (lambda_s + theta))
```

---

## Reading profile values in algorithm code

Every value that changes algorithm behavior must be declared in `config.py`.
Algorithm code reads it directly. There are no defaults hiding in algorithm code.

```python
# WRONG — hidden default, student has no idea this is active
window = getattr(config, "STEADY_WINDOW_S", 30.0)

# WRONG — default buried in function signature
def run_test(profile, smooth_n=7):

# CORRECT — read directly; if it is missing, fail loudly
window = profile.STEADY_WINDOW_S
smooth_n = profile.FOPDT_SMOOTH_N
```

The policy linter (`tools/python/check_project_policy.py`) checks for `getattr(config, "X", default)`
patterns where `X` is not declared in `config.py`. Do not introduce new ones.

---

## Adding a knob

Do not add knobs without explicit permission.

Before adding any parameter, apply this test:

> *Can a student change this value, run the experiment, and observe a meaningful difference
> in PV/SP/OP that teaches them something about control theory?*

If yes: add it to `config.py` with a value and units comment.
If no: do not add it. Use a literal constant with a comment in the code.

**Parameters that nearly always fail this test** (do not add them):
- Rate limiters and slew caps on internal signals
- Timeout multipliers and max-cap variants of existing timeouts
- Coverage ratios and window-fraction thresholds
- Smoothing alphas for internal filters not directly observed by students
- Fallback values for sensor faults

These are implementation details. Hard-code them as named literals if needed.

---

## Deleting something

When you remove a feature, remove everything that supported it:
the algorithm code, the profile entry, the `validate()` rule, the terminal command, the log line.

- Do not leave removed code commented out.
- Do not add a compatibility alias or shim.
- Do not add a replacement unless explicitly asked.
- Count lines before and after. If the file grew, you did not delete — you refactored without permission.

---

## File operation safety (non-negotiable)

Recent failures showed that bulk shell edits can lose or corrupt files. This is forbidden.

- Do not use bulk in-place edit loops over multiple files (`xargs sed -i`, `perl -pi` loops, mass one-liners).
- Do not perform multi-file rename/move waves in one shell command.
- For critical project files (`firmware/*`, `runner/*`, `policies/*`, `tools/python/*`, `docs/*`, `README.md`), use atomic edits:
  - one file at a time with `apply_patch` (preferred), or
  - one explicit `mv/cp` command followed immediately by verification.
- After every rename/move/copy, immediately verify with:
  - `ls`/`find` that expected files exist,
  - `rg` that references are updated.
- If any expected file is missing at any point:
  - stop immediately,
  - report it to the user,
  - do not continue with further edits until user confirms recovery.

---

## Steady-state detection

Steady-state logic uses exactly these two profile parameters. No others.

```
STEADY_WINDOW_S    rolling observation window length [s]
STEADY_BAND_C      PV must stay within ±band [°C]
```

`RollingSteadyWindow` in `identify.py` implements this for identification flows. Use it. Do not reimplement it.
Do not add slope checks, variance checks, or coverage ratios — not even as internal constants.

---

## Memory and scheduling rules

These are hardware constraints, not style preferences.

- **No unbounded list growth** inside any loop that runs continuously. Pre-allocate or use a ring buffer.
- **No object allocation** inside the control loop (no `[]`, `{}`, class instantiation per cycle).
- **No `time.sleep_ms()`** in any runtime path. Use cooperative yielding/wait helpers implemented in the owning module.
- **No string formatting** inside tight loops except for telemetry output lines.
- Use `array('f', [...])` for fixed numeric buffers when available (falls back to list on non-RP2040).
- **No blocking delays of any kind** in project code. If any blocking delay is found, do not replace it immediately:
  first notify the user and get explicit approval, then replace it with a non-blocking/cooperative alternative.
- **Do not add/remove-check `isfinite(PV)` guardrails in firmware algorithms by default.**
  `inf`/`nan` sensor guards are treated as non-teaching defensive noise unless the user explicitly asks for them.
  If such checks exist in touched code, remove them without asking.

---

## Log message style

Firmware reporting uses classed event lines and classed block headers. No bare prints. No debug leftovers.

```
# PHASE:    a named execution stage is starting (presoak, relay, step, ...)
# INFO:     neutral factual statement
# WARNING:  non-fatal unexpected condition
# ERROR:    fatal — heater was forced off
# RESULT:   a computed value the student should read and record
# FINISH:   end-of-procedure marker for host synchronization
# READY:    system is idle, waiting for a command
```

Inside an INFO/RESULT block, detail lines may be contextual (`# ...`) without repeating the class prefix on every row.
Block separator policy is strict:
- use only `# ========================================` for report/section boundaries
- do not use alternative separators (`-`, `_`, mixed lengths)

### Reporting style source of truth

Firmware reporting must follow:
- `docs/en/REPORTING_STYLE.md`
- `docs/bg/СТИЛ_НА_ОТЧЕТИ.md`

These documents define message classes, ownership, redundancy rules, block format,
and READY-line policy. When changing runtime messages, keep firmware and runner
parsing compatibility aligned in the same change.

Telemetry lines (not prefixed with `#`) use the fixed format:
```
PV:xx.x SP:xx.x OP:xx.x
PV:xx.x SP:xx.x OP:xx.x YH:xx.x YP:xx.x   (MPC mode only)
```

---

## Terminology

Use these names consistently. Do not invent synonyms.

| Symbol | Meaning | Unit |
|---|---|---|
| PV | measured temperature | °C |
| SP | setpoint / target temperature | °C |
| OP | controller output to heater | % |
| Kp, Ki, Kd | parallel PID gains | %/°C, %/(°C·s), %·s/°C |
| Kc, Ti, Td | ideal/series PID parameters | same |
| Pb | proportional band | % |
| K, tau, theta | FOPDT model: gain, time constant, dead time | °C/%, s, s |
| Ku, Pu | ultimate gain and period (relay tuning) | same |

Setting names in `config.py` are UPPERCASE (`KP`, `TI_S`).
Equations and log messages use the symbols above (`Kp`, `Ti`).

---

## Documentation language style (EN + BG + DE + ES + UK + IT + FR)

All documentation edits (`docs/en/*.md`, `docs/bg/*.md`, `docs/de/*.md`, `docs/es/*.md`, `docs/uk/*.md`, `docs/it/*.md`, `docs/fr/*.md`, README-style texts) must use a textbook/academic style.
This is mandatory in all supported languages.

- Keep terminology consistent with the table above and with `docs/bg/РЕЧНИК.md`.
- Do not mix casual slang, marketing phrasing, or ad-hoc synonyms.
- Keep parameter names exactly as in code (`KP`, `TUNING_METHOD`) and symbols in academic form (`Kp`, `Ti`, `tau`).
- Prefer clear language-specific control-engineering terms over untranslated loan words unless the English term is the official symbol/abbreviation.
- For localized docs, apply extra care:
  - use standard учебник terminology (`регулирана величина`, `антинасищане`, `PID с две степени на свобода`);
  - avoid mixed-language phrases when a standard technical term exists in that language;
  - if terminology is uncertain, align to the corresponding glossary wording before finalizing.

---

## The experiment runner and YAML recipes

`runner/lab.yaml` is the source of truth for lab exercises.
`firmware/config.py` is the fallback for parameters not set in a recipe.

When a firmware parameter changes name or is removed:
- Update `config.py`.
- Update any recipe in the YAML that sets that parameter.
- Update `cli.py` if it was exposed as a `set`-able command.
- Update `validate()` in `config.py`.

Do not silently accept old parameter names. There is no backward compatibility.

Runner UX rule source:
- Host-side terminal/menu UX must follow `policies/RUNNER_UI_RULES.md`.
- If runner UX behavior/messages/commands are changed, update that file in the same change.
- GitHub publish scope and sync policy must follow `policies/PUBLISH_SCOPE.md` and `policies/bg/PUBLISH_SCOPE.bg.md`.
- `tools/` is local-only and must remain excluded from public GitHub sync.
- Any new top-level directory/file class must be classified in publish scope in the same change.

### Runner and docs enforcement rules

- Runner output ownership:
  - `LAB:` prefix is host-owned (`runner/lab.py`) only.
  - `# ...` token lines are firmware-owned only.
  - Do not mix ownership in one line.
- One event, one host line:
  - For a single host event, emit one status line only.
  - Avoid duplicated host preface lines for the same action.
- Direct terminal start:
  - `runner/lab.py` starts directly in terminal prompt after connect/sync.
  - No mandatory pre-menu mode switch unless explicitly requested by user.
- Unified run path:
  - Recipe runs and direct terminal runs (`control/tune/model/monitor`) must use one execution path in host code.
  - Plot/metrics/run-dir behavior must be identical across both entry points.
- Documentation parity (EN/BG):
  - Any new or updated `docs/en/*.md` must add/update matching files in `docs/bg/*.md`, `docs/de/*.md`, `docs/es/*.md`, `docs/uk/*.md`, `docs/it/*.md`, and `docs/fr/*.md` in the same change.
  - No partial documentation updates are allowed.
- Safe edit tooling:
  - Use `apply_patch` for file edits.
  - Do not route patch edits through shell wrappers.

---

## Change gate

Every change must state:

```
Lines removed / added / net: R / A / (A-R)
  → net must be ≤ 0 for any simplification or deletion task

Knobs added to config.py:
  → none  OR  list each with one-line justification + Effect comment confirmed

getattr(..., default) patterns introduced: none
Backward-compat aliases introduced: none
Unbounded allocations introduced in runtime loops: none
Equation source (required for control-theory algorithm/math changes): cite textbook/paper + equation/form, or `none`
Formula doc sync (required for control-theory algorithm/math changes): updated `docs/en/CONTROL_FORMULAS.md`, or `none`
```

If you cannot write `none` on the last three lines, redesign until you can.

---

## Policy linter

```bash
python3 tools/python/check_project_policy.py
```

Run it. Fix all errors before finishing. Do not modify the linter to make it pass.

---

## One rule

If a student reading `config.py` cannot see it, it must not behave like a knob.
