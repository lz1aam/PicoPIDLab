# Glosario de abreviaturas y términos técnicos (ES)

Este glosario define los términos utilizados en firmware, runner y documentación.

## Señales y variables

- `PV` - Process Variable, variable regulada (temperatura medida en °C)
- `SP` - Setpoint, valor de referencia
- `OP` - Output, señal de control al calentador [%]
- `y` - símbolo interno de libro para salida del proceso (`PV`)
- `r` - símbolo interno de libro para referencia (`SP`)
- `u` - símbolo interno de libro para variable manipulada (`OP`)

## Estructuras y métodos de control

- `ON/OFF` - control todo/nada con histéresis
- `PID` - controlador proporcional-integral-derivativo
- `PI` - controlador proporcional-integral
- `P` - controlador proporcional
- `2DOF` - PID de dos grados de libertad
- `AW` - anti-windup
- `FF` - prealimentación (feedforward)
- `MPC` - control predictivo basado en modelo
- `FUZZY` - control difuso (Sugeno)

## Modelado e identificación

- `FOPDT` - First-Order Plus Dead Time
- `K` - ganancia estática [°C/%]
- `tau` - constante de tiempo [s]
- `theta` - tiempo muerto [s]
- `SK` - método Sundaresan-Krishnaswamy
- `BROIDA` - método de dos puntos de Broïda

## Reglas de sintonía

- `ZN` - Ziegler-Nichols
- `TL` - Tyreus-Luyben
- `CC` - Cohen-Coon
- `Ku` - ganancia última
- `Pu` - periodo último

## Formas de parametrización

- forma `K`: `Kp`, `Ki`, `Kd`
- forma `IDEAL`: `Kc`, `Ti`, `Td`
- `Pb` - banda proporcional [%]
- `SPAN` - rango térmico de ingeniería [°C] para vista Pb

## Magnitudes de tiempo y calidad

- `TS_S` - periodo de muestreo [s]
- `RMSE` - error cuadrático medio

## Etiquetas difusas

- `NB` - Negative Big
- `NS` - Negative Small
- `Z` - Zero
- `PS` - Positive Small
- `PB` - Positive Big
