# Registro de fórmulas de control (ES)

Registro canónico de fórmulas de teoría de control usadas en el código.

## Convenciones

- Externo: `PV`, `SP`, `OP`
- Interno: `y`, `r`, `u`
- Error: `e = r - y`
- `Ts`, `dt` en [s]
- FOPDT: `K`, `tau`, `theta`

## Perturbación en la salida de control

- `u_applied = clamp(u_ctrl + d_u(t), 0, 100)`
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
- `D = LPF(-Kd*d(PV)/dt)`

Código:
- `firmware/controllers.py::PIDParallelPercent.update`

### Conversión SERIES/IDEAL

- `Kc_eff = Kc*(1 + Td/Ti)`
- `Ti_eff = Ti + Td`
- `Td_eff = (Ti*Td)/(Ti + Td)`

Código:
- `firmware/builder.py::_series_to_ideal_equivalent`

### PID 2DOF

- `e_p = beta*SP - PV`
- `e_i = SP - PV`
- `P=Kp*e_p`, `I += Ki*e_i*dt`, `D=-Kd*d(PV)/dt`

Código:
- `firmware/controllers.py::PID2DOFPercent.update`

## Sintonía relay

### Ganancia última

- `A = PV_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(periodo)`

Código:
- `firmware/tuning.py::run_relay_tuning`

### ZN2 y TL (PID)

- ZN2: `Kp=0.6*Ku`, `Ti=0.5*Pu`, `Td=0.125*Pu`
- TL: `Kp=Ku/2.2`, `Ti=2.2*Pu`, `Td=Pu/6.3`
- Mapeo: `Ki=Kp/Ti`, `Kd=Kp*Td`

Código:
- `firmware/tuning.py::run_relay_tuning`
- `firmware/tuning.py::_rule_terms`

## Sintonía por modelo (FOPDT)

### ZN1

- `ZN_1_P:   Kc = tau/(K*theta)`
- `ZN_1_PI:  Kc = 0.9*tau/(K*theta), Ti=3.33*theta`
- `ZN_1_PID: Kc = 1.2*tau/(K*theta), Ti=2*theta, Td=0.5*theta`
- Paralelo: `Kp=Kc`, `Ki=Kc/Ti`, `Kd=Kc*Td`

### Cohen-Coon

- `rho = theta/tau`
- `CC_P:   Kc=(1/K)*(tau/theta)*(1+rho/3)`
- `CC_PI:  Kc=(1/K)*(tau/theta)*(0.9+rho/12)`
- `CC_PID: Kc=(1/K)*(tau/theta)*(4/3+rho/4)`
- `Ti`, `Td` según fórmulas implementadas de curva de reacción

Código:
- `firmware/tuning.py::_model_rule_set`
- `firmware/tuning.py::run_model_tuning`

## Identificación FOPDT

### Estado estacionario

- Ventana: `span(window) >= STEADY_WINDOW_S`
- Banda: `PV_p2p(window) <= 2*STEADY_BAND_C`
- Fin de escalón con condición adicional de desplazamiento

Código:
- `firmware/model.py::_wait_for_steady`
- `firmware/model.py::run_test`

### Ganancia del proceso

- `K = (y_final - y_initial)/(u1 - u0)`

### Respuesta del modelo

- `y_hat(t)=y0`, para `t<theta`
- `y_hat(t)=y0 + K*(u1-u0)*(1-exp(-(t-theta)/tau))`, para `t>=theta`

Código:
- `firmware/model.py::_simulate_fopdt_step`
- `firmware/model.py::run_test`

## Control ON/OFF

- ON si `PV <= SP - hyst`
- OFF si `PV >= SP + hyst`
- en caso contrario mantiene estado

Código:
- `firmware/controllers.py::TwoPositionPercent.update`
