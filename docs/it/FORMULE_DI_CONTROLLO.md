# Registro formule di controllo (IT)

## Convenzioni

- Esterno: `PV`, `SP`, `OP`
- Interno: `y`, `r`, `u`
- Errore: `e = r - y`

## Disturbo su uscita

- `u_applied = clamp(u_ctrl + d_u(t), 0, 100)`

## PID parallelo

- `u = clamp(P + I + D)`
- `P = Kp*e`
- `I[k] = I[k-1] + Ki*e*dt`
- `D = LPF(-Kd*d(PV)/dt)`

Codice:
- `firmware/control.py::PIDParallelPercent.update`

## Relay tuning

- `A = PV_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(period)`

Codice:
- `firmware/identify.py::run_relay_tuning`

## Model tuning (FOPDT)

- Regole `ZN1_*`, `CC_*` con mapping parallelo (`Kp`, `Ki`, `Kd`)

Codice:
- `firmware/identify.py::_model_rule_set`
- `firmware/identify.py::run_model_tuning`
