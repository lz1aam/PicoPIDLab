# Registre des formules de commande (FR)

## Conventions

- Externe: `PV`, `SP`, `OP`
- Interne: `y`, `r`, `u`
- Erreur: `e = r - y`

## Perturbation de sortie

- `u_applied = clamp(u_ctrl + d_u(t), 0, 100)`

## PID parallèle

- `u = clamp(P + I + D)`
- `P = Kp*e`
- `I[k] = I[k-1] + Ki*e*dt`
- `D = LPF(-Kd*d(PV)/dt)`

Code:
- `firmware/controllers.py::PIDParallelPercent.update`

## Relay tuning

- `A = PV_pp/2`
- `d_eff = (u_high - u_low)/2`
- `Ku = 4*d_eff/(pi*A)`
- `Pu = mean(period)`

Code:
- `firmware/tuning.py::run_relay_tuning`

## Tuning basé modèle (FOPDT)

- règles `ZN_1_*`, `CC_*` avec conversion vers `Kp`, `Ki`, `Kd`

Code:
- `firmware/tuning.py::_model_rule_set`
- `firmware/tuning.py::run_model_tuning`
