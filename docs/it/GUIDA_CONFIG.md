# Guida a config.py

`firmware/config.py` contiene solo i parametri didattici modificabili dallo studente.

## Flusso consigliato

1. Impostare `CONTROL_MODE`.
2. Impostare `SETPOINT_C` e, se necessario, `EXPERIMENT_RUN_S`.
3. Eseguire un esperimento.
4. Modificare un solo parametro.
5. Rieseguire e confrontare PV/SP/OP.

## PID

- `PID_VARIANT="PID"` -> `PID_AW_TYPE = NONE/CLAMP/BACKCALC`
- `2DOF`/`FF_PID` -> richiede `PID_ALGORITHM="PARALLEL"`
- `SMITH_PI` -> richiede `MODEL_*` validi e PI attivo (`Kp!=0`, `Ki>0`)

## Documenti correlati

- `docs/it/DIPENDENZE_CONFIGURAZIONE.md`
- `docs/it/AVVIO_RAPIDO.md`
