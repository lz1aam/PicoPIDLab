# Glossary (Abbreviations and Terms)

This glossary defines abbreviations used across firmware, terminal output, and docs.

## Control and Signals

- `PV` - Process Variable (measured process output, here: temperature in °C)
- `SP` - Setpoint (desired target value)
- `OP` - Output (manipulated variable / controller output, here in %)
- `MV` - Manipulated Variable (industry synonym for OP)
- `y` - Textbook internal symbol for process output (maps to `PV`)
- `r` - Textbook internal symbol for reference/setpoint (maps to `SP`)
- `u` - Textbook internal symbol for manipulated input to plant (maps to `OP`/`MV`)

## Control Structures and Methods

- `ON/OFF` - Two-position control with hysteresis
- `PID` - Proportional-Integral-Derivative control
- `PI` - Proportional-Integral control
- `P` - Proportional-only control
- `2DOF` - Two-Degree-of-Freedom PID (setpoint weighting structure)
- `beta` - Setpoint-weighting factor in 2DOF PID (`0..1` in this project, configured by `PID_BETA`)
- `AW` - Anti-windup
- `AW_CLAMP` - Anti-windup by conditional integration
- `AW_BACKCALC` - Anti-windup by back-calculation
- `FF` - Feedforward (preemptive control action)
- `FF_PID` - Feedforward + PID feedback
- `GAIN_SCHED` - Gain scheduling (controller gains vary with operating point)
- `SMITH_PI` - Smith Predictor with PI regulator
- `MPC` - Model Predictive Control
- `MPC-lite` - Educational compact MPC implementation used in this project
- `FUZZY` - Fuzzy logic controller (Sugeno style in this project)

## Modeling and Identification

- `FOPDT` - First-Order Plus Dead Time model
  - Typical parameters: `K` (gain), `tau` (time constant), `theta` (dead time)
- `tau` - Dominant process time constant in FOPDT modeling
- `theta` - Apparent dead time (transport/measurement delay) in FOPDT modeling
- `IMC` - Internal Model Control (tuning/design framework)
- `lambda` - IMC closed-loop time-constant target (tuning aggressiveness parameter)
- `SK` - Sundaresan-Krishnaswamy method (FOPDT parameter estimation method)
- `BROIDA` - Broïda two-point method for FOPDT estimation (28.3% and 40.0% crossings)

## Tuning Rules

- `ZN` - Ziegler-Nichols tuning method
- `TL` - Tyreus-Luyben tuning method
- `Ku` - Ultimate gain (critical gain from relay/oscillation test)
- `Pu` - Ultimate period (oscillation period at ultimate gain)

## Parameterization Forms

- `K form` - Direct gains: `Kp`, `Ki`, `Kd`
- `IDEAL form` - Ideal/ISA style: `Kc`, `Ti`, `Td`
- `PB form` - Proportional band style: `Pb`, `Ti`, `Td`
- Active form policy - selected by `PID_ALGORITHM`: `PARALLEL` uses `K` form, `IDEAL/SERIES` use IDEAL form
- `Pb` - Proportional Band as % of configured input span
- `SPAN` - Input span in °C used together with `Pb` percentage

## PID Algorithm Forms

- `PID_ALGORITHM` - Selects internal structural composition of P/I/D action (`IDEAL`, `PARALLEL`, `SERIES`).
- `PARALLEL` - P, I, and D contributions are computed as separate additive channels and summed at the output.
- `IDEAL` - Uses controller gain `Kc` outside a grouped PI+D style expression; commonly parameterized with `Ti`, `Td` and often mapped to equivalent parallel gains.
- `SERIES` - Interacting/cascaded structure, where dynamic blocks multiply rather than simply add; this interaction can make the same nominal `Ti`, `Td` behave differently than PARALLEL/IDEAL.

## Timing / Dynamics

- `Ti` - Integral time constant
- `Td` - Derivative time constant
- `TS_S` - Sampling period in seconds
- `RMSE` - Root Mean Square Error

## Fuzzy Labels

- `NB` - Negative Big
- `NS` - Negative Small
- `Z` - Zero
- `PS` - Positive Small
- `PB` - Positive Big (fuzzy label; different context than proportional-band `Pb`)
