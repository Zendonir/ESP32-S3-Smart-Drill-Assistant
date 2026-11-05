# Hinweise für Änderungen

- **Bei jeder Firmware-Änderung die Version in `include/Version.h` erhöhen**
  (alle vier Felder konsistent: MAJOR/MINOR/PATCH und `SDA_FW_VERSION_STR`):
  - neue Funktion → MINOR (+1, PATCH = 0)
  - Fehlerbehebung / kleine Anpassung → PATCH (+1)
- Neue Version mit einer Zeile in `CHANGELOG.md` eintragen.
- Vor dem Commit: `pio run -e sda_s3_touch_128` und `scripts/run_host_tests.sh`.
