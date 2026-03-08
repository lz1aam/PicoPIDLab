# Glossar der Abkürzungen und Fachbegriffe (DE)

Dieses Glossar definiert die in Firmware, Runner und Dokumentation verwendeten Begriffe.

## Signale und Größen

- `PV` - Process Variable, Regelgröße (gemessene Temperatur in °C)
- `SP` - Setpoint, Sollwert
- `OP` - Output, Stellgröße zum Heizer [%]
- `y` - interner Lehrbuchsymbol für Prozessausgang (`PV`)
- `r` - interner Lehrbuchsymbol für Führungsgröße (`SP`)
- `u` - interner Lehrbuchsymbol für Stellgröße (`OP`)

## Regelungsstrukturen und Methoden

- `ON/OFF` - Zweipunktregelung mit Hysterese
- `PID` - Proportional-Integral-Differential-Regler
- `PI` - Proportional-Integral-Regler
- `P` - Proportional-Regler
- `2DOF` - PID mit zwei Freiheitsgraden
- `AW` - Anti-Windup
- `FF` - Vorsteuerung (feedforward)
- `MPC` - Model Predictive Control
- `FUZZY` - Fuzzy-Regelung (Sugeno)

## Modellierung und Identifikation

- `FOPDT` - First-Order Plus Dead Time
- `K` - statische Verstärkung [°C/%]
- `tau` - Zeitkonstante [s]
- `theta` - Totzeit [s]
- `SK` - Sundaresan-Krishnaswamy-Methode
- `BROIDA` - Broïda-Zweipunktmethode

## Tuning-Regeln

- `ZN` - Ziegler-Nichols
- `TL` - Tyreus-Luyben
- `CC` - Cohen-Coon
- `Ku` - Grenzverstärkung
- `Pu` - Grenzperiode

## Parametrisierungsformen

- `K`-Form: `Kp`, `Ki`, `Kd`
- `IDEAL`-Form: `Kc`, `Ti`, `Td`
- `Pb` - Proportionalband [%]
- `SPAN` - technischer Temperaturbereich [°C] für Pb-Darstellung

## Zeit- und Qualitätsgrößen

- `TS_S` - Abtastzeit [s]
- `RMSE` - Root Mean Square Error

## Fuzzy-Labels

- `NB` - Negative Big
- `NS` - Negative Small
- `Z` - Zero
- `PS` - Positive Small
- `PB` - Positive Big
