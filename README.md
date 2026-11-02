# Smart Drill Assistant

Firmware für einen digitalen Bohrassistenten, Neigungsmesser und eine elektronische
Wasserwaage auf dem **Waveshare ESP32-S3-Touch-LCD-1.28** (rundes 240×240-Display
GC9A01A, Touch CST816S, IMU QMI8658).

Das Gerät wird an eine Bohrmaschine oder einen Akkuschrauber montiert und zeigt auf
einer kreisförmigen Zielanzeige, wie weit die Bohrachse vom Sollwinkel abweicht. Es
arbeitet vollständig offline. WLAN, Weboberfläche, REST-API und OTA sind optional.

> **Stand der Verifikation:** Die Firmware wird mit PlatformIO vollständig gebaut, und die
> hardwareunabhängige Messbibliothek ist mit 74 Unit-Tests auf dem Host geprüft.
> **Auf echter Hardware ist sie noch nicht getestet.** Pinbelegung, Achsorientierung des
> QMI8658, Farbinvertierung des Displays, Touch-Orientierung und Akku-Spannungsteiler
> müssen am konkreten Board geprüft werden, siehe [docs/Hardware.md](docs/Hardware.md)
> und das Testkonzept in [docs/Testing.md](docs/Testing.md).

## Funktionen

| Bereich | Umfang |
|---|---|
| Messsystem | QMI8658 mit ca. 224 Hz (gemessene ODR), Zeitstempel über den Sample-Zähler des Sensors, Erkennung verlorener Samples, Clipping, festhängender Daten und Bus-Fehler mit I²C-Recovery |
| Sensorfusion | Mahony (Standard), Komplementärfilter und Madgwick, zur Laufzeit umschaltbar. Die Korrekturverstärkung kommt aus einem Unsicherheitsmodell (Kalman-artig) und hängt vom Vertrauen in den Beschleunigungssensor ab |
| Adaptive Filterung | Stillstands-, Bewegungs- und Vibrationserkennung. Bei Vibration wird der Beschleunigungssensor geringer gewichtet, die steigende Unsicherheit wird angezeigt, und ein grünes Signal ist dann gesperrt. Im Stillstand wird der Gyro-Bias nachgeführt. Anzeigeglättung mit One-Euro-Filter |
| Qualität | Stabil / Bewegung / Vibration / Kalibrierung erforderlich / Sensor gestört / Orientierung eingeschränkt / Referenz evtl. gedriftet, jeweils aus Messgrößen abgeleitet |
| Kalibrierung | Gyro-Offset und -Rauschen, 6-Lagen-Kalibrierung des Beschleunigungssensors (Offset und Skala, optional volle 3×3-Matrix), Temperaturtabellen mit Interpolation, Montagekalibrierung (V-Nut-Rollverfahren oder Bohrständer), Profile |
| Modi | Bohrassistent, Wasserwaage (Dosen- und Kantenlibelle), Winkelmesser mit Verlauf, Nullpunkt (Neigung oder 3D), Referenzfläche (nur Neigung oder 3D), Präzisionsmodus, Winkel-Hold mit Speichern |
| UI | LVGL 8.3 für das runde Display mit sicherer Anzeigezone, Schnellmenü, Gesten, Touchsperre, komplettem Einstellungsbaum, Kalibrier-Assistenten und Diagnoseseite |
| Netzwerk | WLAN aus/AP/Client mit AP-Fallback, mDNS, REST-API mit Basic-Auth und Sperre nach Fehlversuchen, WebSocket-Livedaten mit Token, eingebettete Weboberfläche |
| Updates | USB-C und OTA (Upload über die Weboberfläche). Zwei OTA-Slots und Bootloader-Rollback: Ein neues Image bestätigt sich erst nach bestandenem Selbsttest und 30 s stabilem Lauf |
| Energie | Akkuspannung (geglättet, Ladestand nur als „ca.“), Warnstufen, Flash-Schreibsperre bei kritischem Akku, Dimmen und Abschalten des Displays, Deep-Sleep mit Aufwachen über die BOOT-Taste |

## Grenzen der Hardware

* Der QMI8658 ist ein 6-Achsen-Sensor. Die **Neigung gegen die Schwerkraft** ist driftfrei
  messbar. Die **Drehung um die Lotrichtung (Azimut)** ist nur relativ über das Gyroskop
  verfügbar und driftet. Die Firmware kennzeichnet alle gyrobasierten Größen, zeigt deren
  geschätzte Unsicherheit und gibt kein grünes Signal, solange diese Unsicherheit nicht
  in die Toleranz passt.
* Beim Bohren in eine senkrechte Wand ist nur die **Höhenneigung** gravitationsbasiert
  bestimmbar. Die seitliche Richtung zur Wand ist ohne Referenz unbestimmt, die Anzeige
  zeigt dann „Seite frei“.
* Die Anzeige mit 0,01° ist **Auflösung, keine Genauigkeit**. Wiederholbarkeit und absolute
  Genauigkeit müssen nach [docs/Testing.md](docs/Testing.md) mit einer externen Referenz
  ermittelt werden.
* Die Vibrationsanalyse ist bandbegrenzt (interner Tiefpass des Sensors bei ca. 30 Hz,
  224 Hz Abtastung). Sie ist ein Störindikator und kein Vibrationsspektrum. Den
  Motorzustand der Bohrmaschine kann sie nicht sicher erkennen.

## Bauen und Flashen

Voraussetzungen: VS Code mit PlatformIO oder PlatformIO Core ≥ 6.1.

```bash
git clone <repo> && cd ESP32-S3-Smart-Drill-Assistant
pio run -e sda_s3_touch_128                 # bauen
pio run -e sda_s3_touch_128 -t upload       # über USB-C flashen
pio device monitor -b 115200                # Log über USB-Serial (CDC)
pio test -e native                          # Unit-Tests auf dem PC
```

Ohne PlatformIO lassen sich die Tests mit `scripts/run_host_tests.sh` ausführen
(g++ mit Address- und UB-Sanitizer).

Falls der Upload nicht startet: **BOOT** gedrückt halten, **RESET** kurz drücken,
**BOOT** loslassen (ROM-Bootloader), dann erneut hochladen.

Ein OTA-Update geht so: WLAN am Gerät aktivieren (Einstellungen › System › WLAN),
`http://<ip>/` öffnen, mit `admin` und dem Web-Passwort vom Gerät anmelden, unter
**System** die Datei `.pio/build/sda_s3_touch_128/firmware.bin` hochladen.

Die Bibliotheken sind auf exakte Versionen festgelegt, siehe [docs/Dependencies.md](docs/Dependencies.md).

## Bedienung

| Geste | Wirkung |
|---|---|
| Tippen | Schnellmenü: Sollwinkel, Toleranz, Skala, Hold, Nullpunkt/Speichern, Sperre, Profil, Menü |
| Wischen links/rechts | Modus wechseln (bei Vibration oder Touchsperre ignoriert) |
| Lange drücken | Nullpunkt setzen, Fläche erfassen oder Präzisionsmessung starten (je nach Modus). Bei aktivem Hold wird der Wert gespeichert |
| Doppeltippen | Hold an/aus |
| BOOT kurz | konfigurierbar (Hold, Nullpunkt, nächster Modus, Touchsperre) |
| BOOT lang | Touchsperre an/aus. Entsperren alternativ durch 2,5 s Halten auf dem Display |

Typischer Ablauf: Gerät einschalten → das zuletzt aktive Profil wird geladen →
Sollwinkel im Schnellmenü wählen (z. B. 90° für senkrechtes Bohren) → Bohrer ausrichten,
bis der Punkt im Zentrum einrastet und die Anzeige grün wird (Ring gefüllt, Haken).
Bei starker Vibration wird die Anzeige als eingeschränkt markiert.

Farbe und Form gehören immer zusammen:

* **grün**: gefüllter Ring und Haken
* **gelb**: nah am Ziel
* **orange/rot**: deutliche Abweichung, Pfeil am Rand, sobald die Abweichung über die Skala hinausgeht
* **grau**: Messung ungültig, Punkt hohl, Zahlen durch „--“ ersetzt

Ein Farbenblind-Modus ist wählbar.

### Winkelbohren (z. B. 45°)

Im Schnellmenü wählst du unter **Soll** den Winkel und unter **Bezug** die Bedeutung des Winkels.
Bezug und Winkel werden pro Profil gespeichert. Dieselbe Einstellung gibt es unter Menü › Messung › Winkelbezug und in der Weboberfläche.

| Bezug | Bedeutung | Messprinzip |
|---|---|---|
| zur Horizontalen | 45° = Bohrachse 45° gegen die Waagerechte, 90° = senkrecht | absolut, Schwerkraft, driftfrei |
| zur Vertikalen | 45° = 45° gegen das Lot, 0° = senkrecht, 90° = waagerecht | absolut, Schwerkraft, driftfrei |
| Nullpunkt + oben / unten | Bohrer an der Bezugsachse ausrichten (z. B. senkrecht zur Wand), **lange drücken**, dann um X° nach oben bzw. unten neigen | relativ, Schwerkraft, driftfrei |
| Nullpunkt + links / rechts | wie oben, dann um X° seitlich schwenken | relativ, **Gyroskop, driftet** |

Bei den absoluten Bezügen und bei „oben/unten“ ist die seitliche Richtung frei (Anzeige „Seite frei“).
„Links/rechts“ lässt sich mit einem 6-Achsen-Sensor nicht absolut messen, deshalb gibt es diese Richtung nur relativ zu einem Nullpunkt.
In diesem Fall zeigt die Anzeige die geschätzte Drift (±°) und das Alter des Nullpunkts. Grün gibt es nur, solange die geschätzte Drift in die Toleranz passt; sonst neu nullen.
Ist die Bezugsachse fast senkrecht (näher als 10° am Lot), ist „links/rechts“ nicht definiert, und das Gerät meldet das.

Erstinbetriebnahme: Menü › Kalibrierung › **Gyroskop**, dann **Sensor 6 Lagen**,
dann **Montage** (V-Nut oder Ständer). Details stehen in [docs/Calibration.md](docs/Calibration.md).

## Projektstruktur

```
include/            BoardConfig.h (Pins), AppConfig.h (Tasks/Raten), Version.h, lv_conf.h
lib/DrillCore/src/  hardwareunabhängige Mess-Mathematik (unit-getestet)
  MathTypes, LinearAlgebra, CoordinateTransforms, SensorCalibration, MountCalibration,
  SensorFusion, MotionAnalysis, MeasurementQuality, Orientation, ReferenceManager,
  DisplayLogic, Statistics, TimingEstimator, BatteryModel, Settings, MeasurementPipeline
src/hardware/       I2CBus, IMUDriver (QMI8658), TouchDriver (CST816S), DisplayDriver
                    (GC9A01A/LovyanGFX + LVGL), BatteryManager, BootButton
src/measurement/    MeasurementService (IMU- und Fusions-Task, Kommandos, Snapshots)
src/ui/             UIManager, MainScreen (alle Modi), MenuScreen, SettingsScreen,
                    CalibrationScreen, DiagnosticsScreen, Theme, fonts/ (Latin-1)
src/network/        NetworkManager (WLAN/mDNS), WebApi (REST, WebSocket, OTA)
src/storage/        SettingsManager (NVS), ProfileManager (LittleFS), CalibrationStorage (NVS), ConfigJson
src/system/         SystemTask (Akku/Energie), OtaGuard, CpuMonitor, Log, SharedState, AppContext
data/web/           Weboberfläche (wird beim Build gzip-komprimiert eingebettet)
test/               Unity-Tests mit synthetischen Sensordatensätzen
docs/               Hardware, Calibration, Mathematics, API, Testing, Architecture, Dependencies
```

Weitere Dokumente:

* [docs/Architecture.md](docs/Architecture.md): Tasks, Datenfluss, Thread-Sicherheit
* [docs/Mathematics.md](docs/Mathematics.md): Koordinatensysteme, Fusion, Grenzen
* [docs/API.md](docs/API.md): REST-API und WebSocket

## Lizenz

Eigener Code: MIT. Die Bibliotheken behalten ihre eigenen Lizenzen
(LVGL MIT, LovyanGFX FreeBSD, ArduinoJson MIT, arduinoWebSockets LGPL-2.1, Arduino-ESP32 LGPL-2.1).
Die Schriften sind aus Montserrat (OFL) und FontAwesome (OFL/MIT) mit `lv_font_conv` erzeugt.
