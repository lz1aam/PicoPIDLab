# Glossaire des abréviations et termes techniques (FR)

## Signaux et grandeurs

- `PV` — Process Variable, grandeur réglée (température mesurée, °C)
- `SP` — Setpoint, consigne
- `OP` — Output, action de commande vers le chauffage [%]
- `y` — symbole interne de sortie de procédé (`PV`)
- `r` — symbole interne de consigne (`SP`)
- `u` — symbole interne de variable manipulée (`OP`)

## Structures de commande

- `ON/OFF` — commande tout-ou-rien avec hystérésis
- `PID` — régulateur proportionnel-intégral-dérivé
- `PI` — régulateur proportionnel-intégral
- `P` — régulateur proportionnel
- `2DOF` — PID à deux degrés de liberté
- `AW` — anti-windup
- `FF` — feedforward (préaction)
- `MPC` — commande prédictive
- `FUZZY` — commande floue (Sugeno)

## Modélisation et identification

- `FOPDT` — First-Order Plus Dead Time
- `K` — gain statique [°C/%]
- `tau` — constante de temps [s]
- `theta` — temps mort [s]

## Règles de réglage

- `ZN`, `TL`, `CC`, `Ku`, `Pu`

## Formes de paramétrage

- forme `K`: `Kp`, `Ki`, `Kd`
- forme `IDEAL`: `Kc`, `Ti`, `Td`
- `Pb` — bande proportionnelle [%]
- `SPAN` — plage thermique d'ingénierie [°C]
