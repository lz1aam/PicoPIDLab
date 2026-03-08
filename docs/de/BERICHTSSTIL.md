# Reporting-Stil (DE)

Gültigkeitsbereich:
- `firmware/*.py`
- `runner/lab.py`

## Klassen der Laufzeitmeldungen

- `# PHASE:` Start einer klar benannten Phase
- `# INFO:` neutrale Information
- `# WARNING:` nicht-fatales unerwartetes Ereignis
- `# ERROR:` fataler Zustand, Heizer aus
- `# RESULT:` berechnetes Ergebnis
- `# FINISH:` Synchronisationsmarker für Host
- `# READY:` System ist im Leerlauf

## Blockformat

- Nur folgender Trenner ist erlaubt:
  - `# ========================================`
- Keine alternativen Trenner (`-`, `_`, gemischte Längen)

## Eigentum von Ausgaben

- Firmware-Tokens behalten das `# ...` Format.
- Host-Meldungen verwenden `LAB:`.
- Quellenwechsel (Host/Firmware) soll durch genau eine Leerzeile getrennt werden.

## Telemetrieformat

Standard:

```text
PV:xx.x SP:xx.x OP:xx.x
```

MPC:

```text
PV:xx.x SP:xx.x OP:xx.x YH:xx.x YP:xx.x
```
