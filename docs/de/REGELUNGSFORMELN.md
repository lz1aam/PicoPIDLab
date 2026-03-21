# Register der Regelungsformeln (DE)

Kanonsches Register der in Code verwendeten Regelungsformeln.

## Konventionen

- Extern: `PV`, `SP`, `OP`
- Intern: `y`, `r`, `u`
- Fehler: `e = r - y`
- `Ts`, `dt` in [s]
- FOPDT: `K`, `tau`, `theta`

## Ausgangsstörung im Laufbetrieb

- `u_applied = clamp(u + d_u(t), 0, 100)`
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
- `D = LPF(-Kd*dy/dt)`

Code:
- `firmware/control.py::PIDParallelPercent.update`

### SERIES/IDEAL-Umrechnung

- `Kc_eff = Kc*(1 + Td/Ti)`
- `Ti_eff = Ti + Td`
- `Td_eff = (Ti*Td)/(Ti + Td)`

Code:
- `firmware/control.py::_series_to_ideal_equivalent`

### 2DOF PID

- `e_p = beta*r - y`
- `e_i = r - y`
- `P=Kp*e_p`, `I += Ki*e_i*dt`, `D=-Kd*dy/dt`

Code:
- `firmware/control.py::PID2DOFPercent.update`

### Feedforward-PID (`FF_PID`)

- `u = clamp(u_ff + u_fb)`
- `u_fb = Kp*e + I + LPF(-Kd*dy/dt)`
- MANUAL: `u_ff = FF_BIAS_PCT + FF_GAIN_PCT_PER_C*(r - ambient)`
- FOPDT_GAIN: `u_ff = u0 + (r - ambient)/K`

Code:
- `firmware/control.py::PIDFeedForwardPercent._feedforward`
- `firmware/control.py::PIDFeedForwardPercent.update`

### Gain-Scheduled PID (`GAIN_SCHED`)

- Tabelle: `(sched_var, Kp, Ki, Kd)`
- `sched_var = y` oder `r`
- Lineare Interpolation zwischen Nachbarpunkten:
  - `Kp = Kp0 + a*(Kp1-Kp0)`
  - `Ki = Ki0 + a*(Ki1-Ki0)`
  - `Kd = Kd0 + a*(Kd1-Kd0)`
- Bei Änderung von `Ki`: `I <- I*(Ki_new/Ki_old)`

Code:
- `firmware/control.py::GainScheduledPIDPercent._interp_gains`
- `firmware/control.py::GainScheduledPIDPercent.update`

## Bode-Analyse in `tune`

- `L(jw) = C(e^{jwTs}) * G(jw)`
- Parallel / effective-ideal Rueckfuehrung:
  - `C(e^{jwTs}) = Kp + Ki*Ts/(1 - z^{-1}) + Kd*(1 - z^{-1})/Ts * alpha/(1 - (1 - alpha)z^{-1})`
- SERIES-Rueckfuehrung:
  - `C(e^{jwTs}) = Kc*(1 + Ts/(Ti*(1 - z^{-1})))*(1 + Td*(1 - z^{-1})/Ts * alpha/(1 - (1 - alpha)z^{-1}))`
- `z = e^{jwTs}`
- Hinweise:
  - Die Strecke bleibt das identifizierte kontinuierliche FOPDT-Modell `G(jw)`.
  - Der Reglerteil verwendet dieselbe diskrete D-Filter-Struktur wie die Firmware.

Code:
- `runner/lab.py::_parallel_pid_freq_response`
- `runner/lab.py::_series_pid_freq_response`
- `runner/lab.py::_pid_loop_freq_response`

## Relay-Tuning

### Grenzverstärkung

- `A = y_pp/2`
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
- Band: `y_p2p(window) <= 2*STEADY_BAND_C`
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

- EIN wenn `y <= r - hyst`
- AUS wenn `y >= r + hyst`
- sonst Zustand halten

Code:
- `firmware/control.py::TwoPositionPercent.update`

## Fuzzy-Regler

### Inkrementeller Sugeno-Fuzzy-Regler

- `e = r - y`
- `de = d(e)/dt`
- Normierung:
  - `e_n = clamp(e/FUZZY_E_SCALE_C, -1, 1)`
  - `de_n = clamp(de/FUZZY_DE_SCALE_C_PER_S, -1, 1)`
- Sugeno-Singleton-Ausgang:
  - `du_norm = sum(w_ij*c_ij)/sum(w_ij)`
- Inkrementelles Stellgesetz:
  - `u[k] = clamp(u[k-1] + FUZZY_DU_RATE_MAX*dt*du_norm)`

Code:
- `firmware/control.py::_mf5`
- `firmware/control.py::FuzzySugenoIncrementalPercent.update`

## Modellbasierte Regelung

### MPC-lite

- `alpha = exp(-dt/tau)`, `beta = 1 - alpha`
- `x[k+1] = alpha*x[k] + beta*K*(u_del[k] - u0)`
- `y_hat[k] = y0 + x[k]`
- Kostenfunktion:
  - `J = sum_i (r - y_pred[i])^2 + lambda_move*((u0-u_prev)^2 + (u1-u0)^2)`
- Verwendet 2-Move-Blocking: `u0`, `u1`, danach Halten von `u1`

Code:
- `firmware/control.py::MPCLitePercent._simulate_cost`
- `firmware/control.py::MPCLitePercent.update`

### Smith Predictor PI (`SMITH_PI`)

- `G(s) = K/(tau*s + 1) * exp(-theta*s)`
- `y_pred = y_model_nodelay + (y - y_model_delay)`
- `e = r - y_pred`
- `u = clamp(Kp*e + I)`

Code:
- `firmware/control.py::SmithPredictorPI.update`
