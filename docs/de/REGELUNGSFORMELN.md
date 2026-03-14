# Register der Regelungsformeln (DE)

Kanonsches Register der in Code verwendeten Regelungsformeln.

## Konventionen

- Extern: `PV`, `SP`, `OP`
- Intern: `y`, `r`, `u`
- Fehler: `e = r - y`
- `Ts`, `dt` in [s]
- FOPDT: `K`, `tau`, `theta`

## Ausgangsstörung im Laufbetrieb

- `u_applied = clamp(u_ctrl + d_u(t), 0, 100)`
- `d_u(t)=DIST_MAG_PCT` im aktiven Intervall, sonst `0`
- STEP: `t >= DIST_START_S`
- PULSE: `DIST_START_S <= t < DIST_START_S + DIST_DURATION_S`

Code:
- `firmware/main.py::_disturbance_is_active`
- OP-Anwendungspfad in `firmware/main.py`

## PID-Familie

### Parallel-PID (D auf Messung)

- `u = clamp(P + I + D)`
- `P = Kp*e`
- `I[k] = I[k-1] + Ki*e*dt`
- `D = LPF(-Kd*d(PV)/dt)`

Code:
- `firmware/control.py::PIDParallelPercent.update`

### SERIES/IDEAL-Umrechnung

- `Kc_eff = Kc*(1 + Td/Ti)`
- `Ti_eff = Ti + Td`
- `Td_eff = (Ti*Td)/(Ti + Td)`

Code:
- `firmware/control.py::_series_to_ideal_equivalent`

### 2DOF PID

- `e_p = beta*SP - PV`
- `e_i = SP - PV`
- `P=Kp*e_p`, `I += Ki*e_i*dt`, `D=-Kd*d(PV)/dt`

Code:
- `firmware/control.py::PID2DOFPercent.update`

## Relay-Tuning

### Grenzverstärkung

- `A = PV_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(period)`

Code:
- `firmware/identify.py::run_relay_tuning`

### ZN2 und TL (PID)

- ZN2: `Kp=0.6*Ku`, `Ti=0.5*Pu`, `Td=0.125*Pu`
- TL: `Kp=Ku/2.2`, `Ti=2.2*Pu`, `Td=Pu/6.3`
- Mapping: `Ki=Kp/Ti`, `Kd=Kp*Td`

Code:
- `firmware/identify.py::run_relay_tuning`
- `firmware/identify.py::_rule_terms`

## Modellbasiertes Tuning (FOPDT)

### ZN1

- `ZN1_P:   Kc = tau/(K*theta)`
- `ZN1_PI:  Kc = 0.9*tau/(K*theta), Ti=3.33*theta`
- `ZN1_PID: Kc = 1.2*tau/(K*theta), Ti=2*theta, Td=0.5*theta`
- Parallel: `Kp=Kc`, `Ki=Kc/Ti`, `Kd=Kc*Td`

### Cohen-Coon

- `rho = theta/tau`
- `CC_P:   Kc=(1/K)*(tau/theta)*(1+rho/3)`
- `CC_PI:  Kc=(1/K)*(tau/theta)*(0.9+rho/12)`
- `CC_PID: Kc=(1/K)*(tau/theta)*(4/3+rho/4)`
- `Ti`, `Td` gemäß implementierter Reaktionskurvenformeln

Code:
- `firmware/identify.py::_model_rule_set`
- `firmware/identify.py::run_model_tuning`

## FOPDT-Identifikation

### Stationarität

- Fenster: `span(window) >= STEADY_WINDOW_S`
- Band: `PV_p2p(window) <= 2*STEADY_BAND_C`
- Schrittende zusätzlich mit Bewegungsbedingung

Code:
- `firmware/identify.py::_wait_for_steady`
- `firmware/identify.py::run_test`

### Prozessverstärkung

- `K = (y_final - y_initial)/(u1 - u0)`

### Modellantwort

- `y_hat(t)=y0`, für `t<theta`
- `y_hat(t)=y0 + K*(u1-u0)*(1-exp(-(t-theta)/tau))`, für `t>=theta`

Code:
- `firmware/identify.py::_simulate_fopdt_step`
- `firmware/identify.py::run_test`

## ON/OFF-Regler

- EIN wenn `PV <= SP - hyst`
- AUS wenn `PV >= SP + hyst`
- sonst Zustand halten

Code:
- `firmware/control.py::TwoPositionPercent.update`
