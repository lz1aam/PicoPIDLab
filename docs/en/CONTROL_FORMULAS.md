# Control Formulas Registry

This file is the canonical registry of control-theory formulas used in code.
For each formula: equation, symbols/units, source, and code location.

## Conventions

- External interface (telemetry/CLI/recipes):
  - PV: process value (temperature) [degC]
  - SP: setpoint [degC]
  - OP: controller output [%]
- Internal textbook symbols (formulas/comments):
  - y: process output [degC]
  - r: reference/setpoint [degC]
  - u: manipulated input to plant [%]
- Mapping:
  - y <-> PV, r <-> SP, u <-> OP
- e: control error = r - y [degC] (equivalently SP - PV)
- Ts, dt: sample time [s]
- K, tau, theta: FOPDT gain [degC/%], time constant [s], dead time [s]

## Runtime Output Disturbance

### Additive OP disturbance (control runs only)
- Equation:
  - `u_applied = clamp(u_ctrl + d_u(t), 0, 100)`
  - `d_u(t) = DIST_MAG_PCT` for active disturbance interval, else `0`
  - STEP mode active interval: `t >= DIST_START_S`
  - PULSE mode active interval: `DIST_START_S <= t < DIST_START_S + DIST_DURATION_S`
- Source:
  - D. E. Seborg, T. F. Edgar, D. A. Mellichamp, F. J. Doyle III, *Process Dynamics and Control*, 3rd ed.
  - Standard additive manipulated-variable disturbance form used in closed-loop disturbance analysis
- Code:
  - `firmware/main.py::_disturbance_is_active`
  - `firmware/main.py` control loop OP apply path

## PID Family

### Parallel PID with derivative on measurement
- Equation:
  - `u = clamp(P + I + D)`
  - `P = Kp*e`
  - `I[k] = I[k-1] + Ki*e*dt` (with AW policy)
  - `D = LPF(-Kd*d(PV)/dt)`
- Source:
  - K. J. Astrom, T. Hagglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed.
- Code:
  - `firmware/controllers.py::PIDParallelPercent.update`

### Series/Ideal conversion
- Equation:
  - `Kc_eff = Kc*(1 + Td/Ti)`
  - `Ti_eff = Ti + Td`
  - `Td_eff = (Ti*Td)/(Ti + Td)`
- Source:
  - K. J. Astrom, T. Hagglund, *PID Controllers*
- Code:
  - `firmware/builder.py::_series_to_ideal_equivalent`

### 2DOF PID
- Equation:
  - `e_p = beta*SP - PV`
  - `e_i = SP - PV`
  - `P = Kp*e_p`, `I += Ki*e_i*dt`, `D = -Kd*d(PV)/dt`
- Source:
  - K. J. Astrom, T. Hagglund, *Advanced PID Control*
- Code:
  - `firmware/controllers.py::PID2DOFPercent.update`

## Relay Tuning

### Ultimate gain from relay test
- Equation:
  - Relay outputs are fixed two-position ON/OFF: `u_high=100%`, `u_low=0%`
  - `A = PV_pp/2`
  - `d_eff = (u_high - u_low)/2`
  - `Ku = 4*d_eff/(pi*A)`
  - `Pu = mean(oscillation period)`
- Source:
  - Astrom, Hagglund (1984), DOI: `10.1016/0005-1098(84)90014-1`
- Code:
  - `firmware/tuning.py::run_relay_tuning`

### Ziegler-Nichols PID (ultimate-cycle form)
- Equation:
  - `Kp = 0.6*Ku`, `Ti = 0.5*Pu`, `Td = 0.125*Pu`
  - `Ki = Kp/Ti`, `Kd = Kp*Td`
- Source:
  - Ziegler, Nichols (1942), DOI: `10.1115/1.2899060`
- Code:
  - `firmware/tuning.py::run_relay_tuning`

### Tyreus-Luyben PID
- Equation:
  - `Kp = Ku/2.2`, `Ti = 2.2*Pu`, `Td = Pu/6.3`
  - `Ki = Kp/Ti`, `Kd = Kp*Td`
- Source:
  - Tyreus, Luyben (1992), *ISA Transactions*
- Code:
  - `firmware/tuning.py::run_relay_tuning`

### Relay-rule P/PI/PID selection
- Equation:
  - `*_P`: `Ki=0`, `Kd=0`
  - `*_PI`: `Kd=0`
  - `*_PID`: unchanged base PID set
- Source:
  - Standard controller-order reduction from base tuning family
- Code:
  - `firmware/tuning.py::_rule_terms`
  - `firmware/tuning.py::run_relay_tuning`

## Model Tuning (FOPDT)

### Ziegler-Nichols 1 (reaction-curve, open-loop)
- Equation:
  - `ZN_1_P:   Kc = tau/(K*theta)`
  - `ZN_1_PI:  Kc = 0.9*tau/(K*theta),  Ti = 3.33*theta`
  - `ZN_1_PID: Kc = 1.2*tau/(K*theta),  Ti = 2*theta,  Td = 0.5*theta`
  - Parallel mapping: `Kp=Kc`, `Ki=Kc/Ti`, `Kd=Kc*Td`
- Source:
  - J. G. Ziegler, N. B. Nichols (1942), DOI: `10.1115/1.2899060`
- Code:
  - `firmware/tuning.py::_model_rule_set`
  - `firmware/tuning.py::run_model_tuning`

### Cohen-Coon reaction-curve tuning from FOPDT
- Equation:
  - Define ratio: `rho = theta/tau`
  - `CC_P`:
    - `Kc = (1/K)*(tau/theta)*(1 + rho/3)`
    - `Ki=0`, `Kd=0`
  - `CC_PI`:
    - `Kc = (1/K)*(tau/theta)*(0.9 + rho/12)`
    - `Ti = theta*(30 + 3*rho)/(9 + 20*rho)`
    - `Kd=0`
  - `CC_PID`:
    - `Kc = (1/K)*(tau/theta)*(4/3 + rho/4)`
    - `Ti = theta*(32 + 6*rho)/(13 + 8*rho)`
    - `Td = theta*(4/(11 + 2*rho))`
  - Parallel mapping: `Kp=Kc`, `Ki=Kc/Ti`, `Kd=Kc*Td`
- Source:
  - G. H. Cohen, G. A. Coon, *Theoretical Consideration of Retarded Control*, Transactions of the ASME, 1953.
  - D. E. Seborg, T. F. Edgar, D. A. Mellichamp, F. J. Doyle III, *Process Dynamics and Control*, 3rd ed. (reaction-curve tuning table).
- Code:
  - `firmware/tuning.py::_model_rule_set`
  - `firmware/tuning.py::run_model_tuning`

## FOPDT Identification

### Steady-state detector (baseline and step-end)
- Equation:
  - Steady window condition: `span(window) >= STEADY_WINDOW_S`
  - Band condition: `PV_p2p(window) <= 2*STEADY_BAND_C`
  - Baseline steady: steady window condition AND band condition
  - Step-end steady: baseline conditions plus movement guard
    `|PV_avg_step - y0| >= 2*STEADY_BAND_C`
- Notes:
  - No fixed min/max steady-time gates are used inside model identification.
  - Stop/abort/overtemperature callbacks remain the termination guards.
- Code:
  - `firmware/model.py::_wait_for_steady`
  - `firmware/model.py::run_test`

### Process gain
- Equation:
  - `K = (y_final - y_initial)/(u1 - u0)`
- Code:
  - `firmware/model.py::run_test`

### FOPDT response model used for RMSE
- Equation:
  - `y_hat(t) = y0` for `t < theta`
  - `y_hat(t) = y0 + K*(u1-u0)*(1 - exp(-(t-theta)/tau))` for `t >= theta`
- Code:
  - `firmware/model.py::_simulate_fopdt_step`, `firmware/model.py::run_test`

## ON/OFF Controller

### Hysteresis two-position rule
- Equation:
  - ON if `PV <= SP - hyst`
  - OFF if `PV >= SP + hyst`
  - otherwise hold previous state
- Code:
  - `firmware/controllers.py::TwoPositionPercent.update`
