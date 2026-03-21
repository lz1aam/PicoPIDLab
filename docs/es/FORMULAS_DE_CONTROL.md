# Registro de fórmulas de control (ES)

Registro canónico de fórmulas de teoría de control usadas en el código.

## Convenciones

- Externo: `PV`, `SP`, `OP`
- Interno: `y`, `r`, `u`
- Error: `e = r - y`
- `Ts`, `dt` en [s]
- FOPDT: `K`, `tau`, `theta`

## Perturbación en la salida de control

- `u_applied = clamp(u + d_u(t), 0, 100)`
- `d_u(t)=DIST_MAG_PCT` en intervalo activo, si no `0`
- STEP: `t >= DIST_START_S`
- PULSE: `DIST_START_S <= t < DIST_START_S + DIST_DURATION_S`

Código:
- `firmware/main.py::_disturbance_is_active`
- ruta de aplicación de OP en `firmware/main.py`

## Familia PID

### PID paralelo (D sobre medición)

- `u = clamp(P + I + D)`
- `P = Kp*e`
- `I[k] = I[k-1] + Ki*e*dt`
- `D = LPF(-Kd*dy/dt)`

Código:
- `firmware/control.py::PIDParallelPercent.update`

### Conversión SERIES/IDEAL

- `Kc_eff = Kc*(1 + Td/Ti)`
- `Ti_eff = Ti + Td`
- `Td_eff = (Ti*Td)/(Ti + Td)`

Código:
- `firmware/control.py::_series_to_ideal_equivalent`

### PID 2DOF

- `e_p = beta*r - y`
- `e_i = r - y`
- `P=Kp*e_p`, `I += Ki*e_i*dt`, `D=-Kd*dy/dt`

Código:
- `firmware/control.py::PID2DOFPercent.update`

### PID con feedforward (`FF_PID`)

- `u = clamp(u_ff + u_fb)`
- `u_fb = Kp*e + I + LPF(-Kd*dy/dt)`
- MANUAL: `u_ff = FF_BIAS_PCT + FF_GAIN_PCT_PER_C*(r - ambient)`
- FOPDT_GAIN: `u_ff = u0 + (r - ambient)/K`

Código:
- `firmware/control.py::PIDFeedForwardPercent._feedforward`
- `firmware/control.py::PIDFeedForwardPercent.update`

### PID con scheduling de ganancias (`GAIN_SCHED`)

- Tabla: `(sched_var, Kp, Ki, Kd)`
- `sched_var = y` o `r`
- Interpolación lineal entre puntos vecinos:
  - `Kp = Kp0 + a*(Kp1-Kp0)`
  - `Ki = Ki0 + a*(Ki1-Ki0)`
  - `Kd = Kd0 + a*(Kd1-Kd0)`
- Cuando cambia `Ki`: `I <- I*(Ki_new/Ki_old)`

Código:
- `firmware/control.py::GainScheduledPIDPercent._interp_gains`
- `firmware/control.py::GainScheduledPIDPercent.update`

## Analisis de Bode en `tune`

- `L(jw) = C(e^{jwTs}) * G(jw)`
- Realimentacion paralela / ideal efectiva:
  - `C(e^{jwTs}) = Kp + Ki*Ts/(1 - z^{-1}) + Kd*(1 - z^{-1})/Ts * alpha/(1 - (1 - alpha)z^{-1})`
- Realimentacion SERIES:
  - `C(e^{jwTs}) = Kc*(1 + Ts/(Ti*(1 - z^{-1})))*(1 + Td*(1 - z^{-1})/Ts * alpha/(1 - (1 - alpha)z^{-1}))`
- `z = e^{jwTs}`
- Notas:
  - La planta sigue siendo el modelo FOPDT continuo identificado `G(jw)`.
  - La parte del controlador usa la misma estructura discreta del filtro derivativo que el firmware.

Código:
- `runner/lab.py::_parallel_pid_freq_response`
- `runner/lab.py::_series_pid_freq_response`
- `runner/lab.py::_pid_loop_freq_response`

## Sintonía relay

### Ganancia última

- `A = y_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(periodo)`

Código:
- `firmware/identify.py::run_relay_tuning`

### ZN2 y TL (PID)

- ZN2: `Kp=0.6*Ku`, `Ti=0.5*Pu`, `Td=0.125*Pu`
- TL: `Kp=Ku/2.2`, `Ti=2.2*Pu`, `Td=Pu/6.3`
- Mapeo: `Ki=Kp/Ti`, `Kd=Kp*Td`

Código:
- `firmware/identify.py::run_relay_tuning`
- `firmware/identify.py::_rule_terms`

## Sintonía por modelo (FOPDT)

### ZN1

- `ZN1_P:   Kc = tau/(K*theta)`
- `ZN1_PI:  Kc = 0.9*tau/(K*theta), Ti=3.33*theta`
- `ZN1_PID: Kc = 1.2*tau/(K*theta), Ti=2*theta, Td=0.5*theta`
- Paralelo: `Kp=Kc`, `Ki=Kc/Ti`, `Kd=Kc*Td`

### Cohen-Coon

- `rho = theta/tau`
- `CC_P:   Kc=(1/K)*(tau/theta)*(1+rho/3)`
- `CC_PI:  Kc=(1/K)*(tau/theta)*(0.9+rho/12)`
- `CC_PID: Kc=(1/K)*(tau/theta)*(4/3+rho/4)`
- `Ti`, `Td` según fórmulas implementadas de curva de reacción

Código:
- `firmware/identify.py::_model_rule_set`
- `firmware/identify.py::run_model_tuning`

## Identificación FOPDT

### Estado estacionario

- Ventana: `span(window) >= STEADY_WINDOW_S`
- Banda: `y_p2p(window) <= 2*STEADY_BAND_C`
- Fin de escalón con condición adicional de desplazamiento

Código:
- `firmware/identify.py::_wait_for_steady`
- `firmware/identify.py::run_test`

### Ganancia del proceso

- `K = (y_final - y_initial)/(u1 - u0)`

### Respuesta del modelo

- `y_hat(t)=y0`, para `t<theta`
- `y_hat(t)=y0 + K*(u1-u0)*(1-exp(-(t-theta)/tau))`, para `t>=theta`

Código:
- `firmware/identify.py::_simulate_fopdt_step`
- `firmware/identify.py::run_test`

## Control ON/OFF

- ON si `y <= r - hyst`
- OFF si `y >= r + hyst`
- en caso contrario mantiene estado

Código:
- `firmware/control.py::TwoPositionPercent.update`

## Control difuso

### Controlador difuso incremental de Sugeno

- `e = r - y`
- `de = d(e)/dt`
- Normalización:
  - `e_n = clamp(e/FUZZY_E_SCALE_C, -1, 1)`
  - `de_n = clamp(de/FUZZY_DE_SCALE_C_PER_S, -1, 1)`
- Salida Sugeno singleton:
  - `du_norm = sum(w_ij*c_ij)/sum(w_ij)`
- Ley incremental:
  - `u[k] = clamp(u[k-1] + FUZZY_DU_RATE_MAX*dt*du_norm)`

Código:
- `firmware/control.py::_mf5`
- `firmware/control.py::FuzzySugenoIncrementalPercent.update`

## Control basado en modelo

### MPC-lite

- `alpha = exp(-dt/tau)`, `beta = 1 - alpha`
- `x[k+1] = alpha*x[k] + beta*K*(u_del[k] - u0)`
- `y_hat[k] = y0 + x[k]`
- Coste:
  - `J = sum_i (r - y_pred[i])^2 + lambda_move*((u0-u_prev)^2 + (u1-u0)^2)`
- Usa bloqueo de dos movimientos: `u0`, `u1`, y luego mantiene `u1`

Código:
- `firmware/control.py::MPCLitePercent._simulate_cost`
- `firmware/control.py::MPCLitePercent.update`

### Smith Predictor PI (`SMITH_PI`)

- `G(s) = K/(tau*s + 1) * exp(-theta*s)`
- `y_pred = y_model_nodelay + (y - y_model_delay)`
- `e = r - y_pred`
- `u = clamp(Kp*e + I)`

Código:
- `firmware/control.py::SmithPredictorPI.update`
