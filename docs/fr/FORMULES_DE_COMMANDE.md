# Registre des formules de commande (FR)

## Conventions

- Externe: `PV`, `SP`, `OP`
- Interne: `y`, `r`, `u`
- Erreur: `e = r - y`
- `Ts`, `dt` en [s]
- FOPDT: `K`, `tau`, `theta`

## Perturbation de sortie

- `u_applied = clamp(u + d_u(t), 0, 100)`
- `d_u(t)=DIST_MAG_PCT` dans l’intervalle actif, sinon `0`
- STEP: `t >= DIST_START_S`
- PULSE: `DIST_START_S <= t < DIST_START_S + DIST_DURATION_S`

Code:
- `firmware/main.py::_disturbance_is_active`
- chemin d’application OP dans `firmware/main.py`

## PID parallèle

- `u = clamp(P + I + D)`
- `P = Kp*e`
- `I[k] = I[k-1] + Ki*e*dt`
- `D = LPF(-Kd*dy/dt)`

Code:
- `firmware/control.py::PIDParallelPercent.update`

## Conversion SERIES/IDEAL

- `Kc_eff = Kc*(1 + Td/Ti)`
- `Ti_eff = Ti + Td`
- `Td_eff = (Ti*Td)/(Ti + Td)`

Code:
- `firmware/control.py::_series_to_ideal_equivalent`

## PID 2DOF

- `e_p = beta*r - y`
- `e_i = r - y`
- `P=Kp*e_p`, `I += Ki*e_i*dt`, `D=-Kd*dy/dt`

Code:
- `firmware/control.py::PID2DOFPercent.update`

## PID avec feedforward (`FF_PID`)

- `u = clamp(u_ff + u_fb)`
- `u_fb = Kp*e + I + LPF(-Kd*dy/dt)`
- MANUAL: `u_ff = FF_BIAS_PCT + FF_GAIN_PCT_PER_C*(r - ambient)`
- FOPDT_GAIN: `u_ff = u0 + (r - ambient)/K`

Code:
- `firmware/control.py::PIDFeedForwardPercent._feedforward`
- `firmware/control.py::PIDFeedForwardPercent.update`

## PID à gain programmé (`GAIN_SCHED`)

- Table: `(sched_var, Kp, Ki, Kd)`
- `sched_var = y` ou `r`
- Interpolation linéaire entre points voisins:
  - `Kp = Kp0 + a*(Kp1-Kp0)`
  - `Ki = Ki0 + a*(Ki1-Ki0)`
  - `Kd = Kd0 + a*(Kd1-Kd0)`
- Quand `Ki` change: `I <- I*(Ki_new/Ki_old)`

Code:
- `firmware/control.py::GainScheduledPIDPercent._interp_gains`
- `firmware/control.py::GainScheduledPIDPercent.update`

## Relay tuning

- `A = y_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(period)`

Code:
- `firmware/identify.py::run_relay_tuning`
- `firmware/identify.py::_rule_terms`

## Tuning basé modèle (FOPDT)

- règles `ZN1_*`, `CC_*` avec conversion vers `Kp`, `Ki`, `Kd`

Code:
- `firmware/identify.py::_model_rule_set`
- `firmware/identify.py::run_model_tuning`

## Identification FOPDT

- Stationnarité:
  - fenêtre: `span(window) >= STEADY_WINDOW_S`
  - bande: `y_p2p(window) <= 2*STEADY_BAND_C`
- `K = (y_final - y_initial)/(u1 - u0)`
- `y_hat(t)=y0`, pour `t<theta`
- `y_hat(t)=y0 + K*(u1-u0)*(1-exp(-(t-theta)/tau))`, pour `t>=theta`

Code:
- `firmware/identify.py::_wait_for_steady`
- `firmware/identify.py::_simulate_fopdt_step`
- `firmware/identify.py::run_test`

## Régulateur ON/OFF

- ON si `y <= r - hyst`
- OFF si `y >= r + hyst`
- sinon maintien de l’état

Code:
- `firmware/control.py::TwoPositionPercent.update`

## Commande floue

### Régulateur flou incrémental de Sugeno

- `e = r - y`
- `de = d(e)/dt`
- Normalisation:
  - `e_n = clamp(e/FUZZY_E_SCALE_C, -1, 1)`
  - `de_n = clamp(de/FUZZY_DE_SCALE_C_PER_S, -1, 1)`
- Sortie singleton Sugeno:
  - `du_norm = sum(w_ij*c_ij)/sum(w_ij)`
- Loi incrémentale:
  - `u[k] = clamp(u[k-1] + FUZZY_DU_RATE_MAX*dt*du_norm)`

Code:
- `firmware/control.py::_mf5`
- `firmware/control.py::FuzzySugenoIncrementalPercent.update`

## Commande basée modèle

### MPC-lite

- `alpha = exp(-dt/tau)`, `beta = 1 - alpha`
- `x[k+1] = alpha*x[k] + beta*K*(u_del[k] - u0)`
- `y_hat[k] = y0 + x[k]`
- Coût:
  - `J = sum_i (r - y_pred[i])^2 + lambda_move*((u0-u_prev)^2 + (u1-u0)^2)`
- Utilise un blocage à deux mouvements: `u0`, `u1`, puis maintien de `u1`

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
