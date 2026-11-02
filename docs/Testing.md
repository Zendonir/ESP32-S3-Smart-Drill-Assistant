# Test- und Validierungskonzept

## 1. Automatisierte Tests (Host)

```bash
pio test -e native                # PlatformIO + Unity
scripts/run_host_tests.sh         # g++ -Werror, AddressSanitizer + UBSan
```

71 Testfälle in 8 Suiten prüfen die hardwareunabhängige Bibliothek `lib/DrillCore`.
Datengrundlage ist ein deterministischer synthetischer IMU-Generator
(`test/support/SyntheticImu.h`) mit Rauschen, Bias, Skalen- und Offsetfehlern,
Vibration und Sample-Verlust.

| Suite | Inhalt |
|---|---|
| `test_math` | Quaternionen (Achse/Winkel, Matrix-Hin-und-Rück, Komposition, exp/log, zwei Vektoren), Matrixinverse, Achspermutation (Parsen, Spiegelung abgelehnt), Anzeigebasis, Jacobi-Eigenzerlegung, kleinste Quadrate |
| `test_calibration` | Gyro-Bias und -Rauschen, Neustart bei Bewegung, Ablehnung zu hohen Rauschens, 6 Lagen Offset/Skala mit Schiefstand, erweiterte 3×3-Kreuzkopplung, falsche Lage, Temperaturinterpolation (kein Extrapolieren, „inaktiv“ bei einem Punkt), Plausibilität, Montage-Rollfit, zu geringe Abdeckung, Ständerverfahren |
| `test_fusion` | Konvergenz aller drei Filter, reine Gyro-Integration, **Azimut bleibt von der Accel-Korrektur unberührt**, Driftrate ohne Korrektur, Wachsen und Schrumpfen der Unsicherheit, Filterwechsel |
| `test_orientation` | Wasserwaage flach und Kante, relative Neigung, senkrechtes Ziel (2D, Richtung), waagerechtes Ziel (nur Elevation, Seite frei), 45° mit Hemisphärenwechsel, Winkelkorrektur, Richtungsziel seitlich, Fläche eingeschränkt vs. 3D, relativer Azimut |
| `test_detectors` | Stillstand nach Haltezeit, schnell/langsam, langsames Kriechen ist kein Stillstand, Vibrationsstufen mit Rückstufung, Accel-Vertrauen, Zeitschätzer (±3 % ODR, Überlauf, Verlust, Duplikat, Lücke), Fallback auf Host-Zeit, Werkzeugzustand |
| `test_display` | Zielhysterese und Haltezeit, ungültig ist nie grün, automatische Skala, Randbegrenzung des Markers, One-Euro, Zahlenhysterese, Präzisionsstatistik, Drifterkennung, Ringpuffer |
| `test_settings` | Defaults gültig, Reparatur ungültiger Werte (NaN, Bereich, Hostname, Passwortlänge, Achsabbildung), Idempotenz, Profilvalidierung, Akkumodell |
| `test_pipeline` | Gesamtkette: statische Genauigkeit (< 0,05° synthetisch), schnelle Drehung, Vibration (keine grüne Freigabe, Orientierung bleibt), lange Vibration lässt σ wachsen, festhängende Daten ergeben Sensorfehler, Sample-Verlust-Flag, Gyro-Kalibrierung, Online-Bias, Präzisionsmessung, Referenzdrift, Montagekalibrierung |

Die synthetischen Genauigkeiten belegen nur die **Korrektheit der Algorithmen**, nicht
die Leistung des realen Sensors.

## 2. Hardware-Testverfahren (manuell, mit Protokoll)

Hilfsmittel:

* stabile Unterlage (Granitplatte oder Werkbank)
* Präzisionswasserwaage oder digitaler Referenz-Neigungsmesser (Genauigkeit ≤ 0,05°)
* Sinuslineal oder Präzisionswinkel 45°/90°
* V-Nut
* Bohrständer
* für Test 6: Thermometer bzw. Klimaschrank oder ein warmer und ein kalter Raum

Messwerte stammen aus dem Präzisionsmodus (3 s) oder aus `GET /api/orientation`.
Vor jeder Serie wird das Gerät 10 min eingeschaltet (Warmlauf).

| Nr. | Prüfung | Durchführung | Auswertung |
|---|---|---|---|
| 1 | Wiederholbarkeit 0° | 10× abheben und auf dieselbe Anschlagposition auflegen, je eine Präzisionsmessung | Std der 10 Werte (Gerät zeigt „Wdh. σ“). Entwicklungsziel < 0,2° |
| 2 | Wiederholbarkeit 45° | wie 1 auf dem 45°-Sinuslineal | Std; Mittelwert gegen die Referenz (absolute Abweichung) |
| 3 | nahe 90° | Achse senkrecht im Ständer, Wiederholung 10× | Std. Prüfen, dass keine Sprünge bei ±90° auftreten |
| 4 | Rückkehr auf Ausrichtung | Nullpunkt setzen, Gerät stark bewegen, zurück an den Anschlag (10×) | Restabweichung relativ zum Nullpunkt |
| 5 | Langzeitdrift 30 min | ruhig liegen lassen, `/api/orientation` alle 10 s protokollieren | Spanne und Trend der Neigung. Azimut-Drift des 3D-Nullpunkts getrennt auswerten |
| 6 | Temperatur | Messung bei T1, Gerät 1 h bei T2 (ΔT ≥ 10 K), Messung bei T2, mit und ohne 2. Kalibrierpunkt | Neigungsänderung/K; Wirkung der Kompensation |
| 7 | Neustart | Messung, Neustart, Messung (10×) | Differenz. Die Kalibrierung muss geladen sein (`selfTest.calibrationLoaded`) |
| 8 | langsame Bewegung | Gerät mit ca. 2 °/s über 20° kippen (Sinuslineal) | Verzögerung und Glätte der Anzeige, keine Sprünge |
| 9 | schnelle Bewegung | 90°-Schwenk in < 0,5 s, danach ruhig | Einschwingzeit bis < 0,1° Abweichung vom Ruhewert |
| 10 | Vibration | Bohrmaschine (ohne Bohren, dann mit) laufen lassen | Zustand „Vibration“ bzw. „Unsicher“, keine grüne Freigabe, Ruhewert nach Abschalten in < 2 s |
| 11 | Fusion nach abrupten Bewegungen | Schläge oder Stöße auf das Werkzeug, Fallenlassen auf eine weiche Unterlage | Kein Einfrieren, Rückkehr zum Ruhewert. Die Zähler für Clipping und Sensorfehler sind plausibel |
| 12 | Kalibrierdaten nach Neustart | Kalibrierdaten anzeigen, Neustart, erneut anzeigen | identisch |
| 13 | Referenzdrift Raumrichtungen | Fläche 3D erfassen, Gerät 5 min in 6 Lagen ruhen lassen bzw. ummontieren, zurück auf die Fläche | angezeigte σ_ψ gegen die tatsächliche Azimutabweichung (σ_ψ muss den Fehler überdecken) |

Ergänzend:

* **Achsprüfung** nach [Hardware.md](Hardware.md#verifikation-am-board-inbetriebnahme-checkliste)
* **OTA-Test**:
  1. Version A per USB flashen.
  2. Version B per OTA einspielen; sie läuft nach dem Neustart, und nach 30 s steht im
     Log „firmware image confirmed“.
  3. Eine Version C mit absichtlich ausgelöstem Selbsttestfehler einspielen (z. B.
     `app.boot.imu = false` erzwingen). Erwartung: Rollback auf B.
  4. Strom während des Uploads trennen. Erwartung: Das Gerät startet mit B.
* **Energie**: Dimmen, Display aus, Deep-Sleep und Aufwachen mit BOOT. Stromaufnahme in
  jedem Zustand mit einem USB-Messgerät dokumentieren.

## 3. Kennwerte, die zu ermitteln und zu dokumentieren sind

Diese Größen sind getrennt anzugeben. Keine ist durch die Firmware garantiert.

| Größe | Methode |
|---|---|
| Anzeigeauflösung | 0,1° bzw. 0,01° (Einstellung), rein numerisch |
| Rauschen | Std im Präzisionsmodus (Ruhe) |
| Wiederholbarkeit | Tests 1–4, 7 |
| Kurzzeitstabilität | Spanne über 60 s in Ruhe |
| Langzeitdrift | Test 5 |
| absolute Genauigkeit | Mittelwert gegen die externe Referenz, Tests 2/3 |
| Referenzgenauigkeit | Test 13 |
| dynamische Genauigkeit | Tests 8/9 gegen das Sinuslineal, Verzögerung in ms |

Die geschätzten Unsicherheiten (σ) der Firmware sind Modellwerte und **ersetzen diese
Messungen nicht**. Mit den Messungen lassen sich die Modellparameter überprüfen
(`tempCoeffAssumedDpsPerK`, Skalenfehler).

## 4. Status

* Firmware-Build (`sda_s3_touch_128`): erfolgreich mit PlatformIO 6.1, espressif32 6.9.0
  und Arduino-ESP32 2.0.17.
* Host-Tests: 71/71 bestanden.
* **Hardwaretests 1–13: noch nicht durchgeführt.** Es lag keine Hardware vor. Ergebnisse
  bitte im Format der Tabelle oben unter `docs/results/` ablegen.
