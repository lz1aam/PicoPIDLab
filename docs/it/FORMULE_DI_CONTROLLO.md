# Registro formule di controllo (IT)

## Convenzioni

- Esterno: `PV`, `SP`, `OP`
- Interno: `y`, `r`, `u`
- Errore: `e = r - y`
- `Ts`, `dt` in [s]
- FOPDT: `K`, `tau`, `theta`

## Disturbo su uscita

- `u_applied = clamp(u + d_u(t), 0, 100)`
- `d_u(t)=DIST_MAG_PCT` nell’intervallo attivo, altrimenti `0`
- STEP: `t >= DIST_START_S`
- PULSE: `DIST_START_S <= t < DIST_START_S + DIST_DURATION_S`

Codice:
- `firmware/main.py::_disturbance_is_active`
- percorso di applicazione OP in `firmware/main.py`

## PID parallelo

- `u = clamp(P + I + D)`
- `P = Kp*e`
- `I[k] = I[k-1] + Ki*e*dt`
- `D = LPF(-Kd*dy/dt)`

Codice:
- `firmware/control.py::PIDParallelPercent.update`

## Conversione SERIES/IDEAL

- `Kc_eff = Kc*(1 + Td/Ti)`
- `Ti_eff = Ti + Td`
- `Td_eff = (Ti*Td)/(Ti + Td)`

Codice:
- `firmware/control.py::_series_to_ideal_equivalent`

## PID 2DOF

- `e_p = beta*r - y`
- `e_i = r - y`
- `P=Kp*e_p`, `I += Ki*e_i*dt`, `D=-Kd*dy/dt`

Codice:
- `firmware/control.py::PID2DOFPercent.update`

## PID con feedforward (`FF_PID`)

- `u = clamp(u_ff + u_fb)`
- `u_fb = Kp*e + I + LPF(-Kd*dy/dt)`
- MANUAL: `u_ff = FF_BIAS_PCT + FF_GAIN_PCT_PER_C*(r - ambient)`
- FOPDT_GAIN: `u_ff = u0 + (r - ambient)/K`

Codice:
- `firmware/control.py::PIDFeedForwardPercent._feedforward`
- `firmware/control.py::PIDFeedForwardPercent.update`

## PID con gain scheduling (`GAIN_SCHED`)

- Tabella: `(sched_var, Kp, Ki, Kd)`
- `sched_var = y` oppure `r`
- Interpolazione lineare tra punti vicini:
  - `Kp = Kp0 + a*(Kp1-Kp0)`
  - `Ki = Ki0 + a*(Ki1-Ki0)`
  - `Kd = Kd0 + a*(Kd1-Kd0)`
- Quando `Ki` cambia: `I <- I*(Ki_new/Ki_old)`

Codice:
- `firmware/control.py::GainScheduledPIDPercent._interp_gains`
- `firmware/control.py::GainScheduledPIDPercent.update`

## Analisi di Bode in `tune`

- `L(jw) = C(e^{jwTs}) * G(jw)`
- Retroazione parallela / ideale effettiva:
  - `C(e^{jwTs}) = Kp + Ki*Ts/(1 - z^{-1}) + Kd*(1 - z^{-1})/Ts * alpha/(1 - (1 - alpha)z^{-1})`
- Retroazione SERIES:
  - `C(e^{jwTs}) = Kc*(1 + Ts/(Ti*(1 - z^{-1})))*(1 + Td*(1 - z^{-1})/Ts * alpha/(1 - (1 - alpha)z^{-1}))`
- `z = e^{jwTs}`
- Note:
  - Il processo resta il modello FOPDT continuo identificato `G(jw)`.
  - La parte del controllore usa la stessa struttura discreta del filtro derivativo del firmware.

Codice:
- `runner/lab.py::_parallel_pid_freq_response`
- `runner/lab.py::_series_pid_freq_response`
- `runner/lab.py::_pid_loop_freq_response`

## Relay tuning

- `A = y_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(period)`

Codice:
- `firmware/identify.py::run_relay_tuning`
- `firmware/identify.py::_rule_terms`

## Model tuning (FOPDT)

- Regole `ZN1_*`, `CC_*` con mapping parallelo (`Kp`, `Ki`, `Kd`)

Codice:
- `firmware/identify.py::_model_rule_set`
- `firmware/identify.py::run_model_tuning`

## Identificazione FOPDT

- Stazionarietà:
  - finestra: `span(window) >= STEADY_WINDOW_S`
  - banda: `y_p2p(window) <= 2*STEADY_BAND_C`
- `K = (y_final - y_initial)/(u1 - u0)`
- `y_hat(t)=y0`, per `t<theta`
- `y_hat(t)=y0 + K*(u1-u0)*(1-exp(-(t-theta)/tau))`, per `t>=theta`

Codice:
- `firmware/identify.py::_wait_for_steady`
- `firmware/identify.py::_simulate_fopdt_step`
- `firmware/identify.py::run_test`

## Controllo ON/OFF

- ON se `y <= r - hyst`
- OFF se `y >= r + hyst`
- altrimenti mantiene lo stato

Codice:
- `firmware/control.py::TwoPositionPercent.update`

## Controllo fuzzy

### Controllore fuzzy incrementale di Sugeno

- `e = r - y`
- `de = d(e)/dt`
- Normalizzazione:
  - `e_n = clamp(e/FUZZY_E_SCALE_C, -1, 1)`
  - `de_n = clamp(de/FUZZY_DE_SCALE_C_PER_S, -1, 1)`
- Uscita singleton Sugeno:
  - `du_norm = sum(w_ij*c_ij)/sum(w_ij)`
- Legge incrementale:
  - `u[k] = clamp(u[k-1] + FUZZY_DU_RATE_MAX*dt*du_norm)`

Codice:
- `firmware/control.py::_mf5`
- `firmware/control.py::FuzzySugenoIncrementalPercent.update`

## Controllo basato su modello

### MPC-lite

- `alpha = exp(-dt/tau)`, `beta = 1 - alpha`
- `x[k+1] = alpha*x[k] + beta*K*(u_del[k] - u0)`
- `y_hat[k] = y0 + x[k]`
- Costo:
  - `J = sum_i (r - y_pred[i])^2 + lambda_move*((u0-u_prev)^2 + (u1-u0)^2)`
- Usa blocco a due mosse: `u0`, `u1`, poi mantiene `u1`

Codice:
- `firmware/control.py::MPCLitePercent._simulate_cost`
- `firmware/control.py::MPCLitePercent.update`

### Smith Predictor PI (`SMITH_PI`)

- `G(s) = K/(tau*s + 1) * exp(-theta*s)`
- `y_pred = y_model_nodelay + (y - y_model_delay)`
- `e = r - y_pred`
- `u = clamp(Kp*e + I)`

Codice:
- `firmware/control.py::SmithPredictorPI.update`
