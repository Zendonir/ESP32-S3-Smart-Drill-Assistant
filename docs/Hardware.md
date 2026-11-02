# Hardware

## Board

**Waveshare ESP32-S3-Touch-LCD-1.28.** Nicht zu verwechseln mit dem
„ESP32-S3-LCD-1.28“ ohne Touch, das eine andere Pinbelegung hat.

| Komponente | Typ | Anbindung |
|---|---|---|
| SoC | ESP32-S3R2, Dual-Core LX7 240 MHz, 2 MB Quad-PSRAM im Gehäuse | – |
| Flash | 16 MB (QIO) | – |
| Display | 1,28″ rund, 240×240, GC9A01A | SPI (SPI2, DMA) |
| Touch | CST816S, Einzelberührung | I²C 0x15 |
| IMU | QMI8658 (3× Beschleunigung, 3× Gyro, Temperatur, Sample-Zähler) | I²C 0x6B (0x6A als Fallback) |
| Akku | 3,7-V-Li-Ion/LiPo, integrierte Ladeschaltung | ADC GPIO1 über Spannungsteiler |
| USB | USB-C am nativen USB des ESP32-S3 (CDC/JTAG) | – |

## Pinbelegung (`include/BoardConfig.h`)

| Funktion | GPIO | Funktion | GPIO |
|---|---|---|---|
| I²C SDA (Touch + IMU) | 6 | LCD RESET | 14 |
| I²C SCL (Touch + IMU) | 7 | LCD Backlight (PWM) | 2 |
| LCD DC | 8 | Touch RESET | 13 |
| LCD CS | 9 | Touch INT | 5 |
| LCD CLK | 10 | QMI8658 INT1 | 4 |
| LCD MOSI | 11 | QMI8658 INT2 | 3 |
| LCD MISO | 12 | Akku-ADC | 1 |
| BOOT-Taste | 0 | | |

INT1 und INT2 des QMI8658 sind reserviert, werden aber nicht verwendet. Die Firmware
liest datengetrieben über den Sample-Zähler des Sensors, unabhängig von der
Interrupt-Konfiguration des Chips (siehe [Mathematics.md](Mathematics.md#2-sensordatenverarbeitung)).

## Quellen und was davon geprüft ist

| Angabe | Quelle | Status |
|---|---|---|
| Pinbelegung | Projektvorgabe; deckt sich mit den Waveshare-Demos für die Touch-Variante | **am Board prüfen** |
| QMI8658-Register, Adresse 0x6B, CTRL1 = 0x60, Little-Endian-Daten, Temperatur/256 | Offizieller Waveshare-Treiber `github.com/waveshareteam/Waveshare-ESP32-components/sensor/qmi8658` | aus Quellcode übernommen |
| Soft-Reset 0x60 ← 0xB0, Statusregister 0x4D | QMI8658A/C-Datenblatt | nach Datenblatt |
| CTRL5-Tiefpass (Bit 6:5/4 Gyro, 2:1/0 Accel) | Datenblatt | nach Datenblatt |
| 6DOF-ODR 224,2 Hz bei ODR-Code 5 | Datenblatt (gyrotaktbasierte Raten) | **wird zur Laufzeit gemessen** und unter Diagnose angezeigt |
| GC9A01A-Initialisierung | LovyanGFX 1.1.16 `Panel_GC9A01` | Bibliothek |
| Akku-Teiler Faktor 3,0 | Waveshare-Beispielcode (2:1-Teiler) | **am Schaltplan prüfen**, im Menü einstellbar |

Die Waveshare-Dokumentation (`docs.waveshare.com`, `waveshare.com/wiki`) war aus der
Entwicklungsumgebung nicht erreichbar. Deshalb sind Schaltplan und Boardrevisionen hier
nicht gegengeprüft. Bitte vor dem ersten Einsatz die folgenden Punkte durchgehen.

## Verifikation am Board (Inbetriebnahme-Checkliste)

1. **Boardtyp.** Auf der Rückseite steht „ESP32-S3-Touch-LCD-1.28“, und ein
   Touchcontroller ist vorhanden.
2. **Serielles Log** (`pio device monitor`). Es muss `QMI8658 at 0x6B`,
   `CST816S chip 0xB?` und `GC9A01 240x240` erscheinen. Erkennt die Firmware ein Bauteil
   nicht, meldet sie das im Startbildschirm und im Log.
3. **Displayfarben.** Der Hintergrund ist schwarz und Grün ist grün. Sind die Farben
   invertiert, `kLcdInvertColors` in `BoardConfig.h` umschalten.
4. **Touch.** Diagnose › „Touch x/y“ zeigt die Koordinate unter dem Finger
   (oben links ≈ 0/0). Stimmt sie nicht, `kTouchSwapXY`/`kTouchInvertX/Y` anpassen.
5. **IMU-Achsen.** Das Gerätekoordinatensystem ist X = rechts, Y = oben,
   Z = aus dem Display heraus. Diagnose › „a raw“:
   * Display flach nach oben → `a raw ≈ 0 0 +1`
   * auf die untere Kante gestellt (oben = oben) → `≈ 0 +1 0`
   * auf die linke Kante gestellt (rechte Kante oben) → `≈ +1 0 0`

   Weicht das ab, unter Einstellungen › System › „Achsen S→D“ die passende
   Zuordnung wählen (alle 24 echten Drehungen stehen zur Auswahl). Danach die
   6-Lagen-Kalibrierung wiederholen.
6. **Abtastrate.** Diagnose › „Rate“ liegt bei etwa 220–230 Hz, und die Zahl der
   verlorenen Samples steigt im Ruhebetrieb nicht.
7. **Akku.** Die Akkuspannung mit einem Multimeter messen, mit Einstellungen › System ›
   Akku vergleichen und bei Bedarf den „Akku-Teiler“ korrigieren.
8. **SPI-Takt.** Bei Bildfehlern `kLcdSpiFreqHz` von 80 MHz auf 40 MHz senken.

## Stromversorgung und Akku

* Versorgung über USB-C oder einen 1S-Li-Ion/LiPo-Akku über den Akkustecker des Boards.
  **Polarität, Steckertyp und Ladestrom** stehen im Schaltplan der vorliegenden Revision.
  Waveshare-Boards verwenden typischerweise MX1.25-Stecker, die Polarität ist aber nicht
  bei allen Akkuherstellern gleich. Vor dem Anschließen nachmessen.
* Die Firmware misst nur die Spannung. Ob gerade geladen wird, ist ohne Statuspin nicht
  erkennbar. Über 4,30 V wird „USB“ angezeigt.
* Der Ladestand ist eine **Schätzung** aus einer typischen Leerlaufspannungskurve,
  gerundet auf 5 %. Unter Last oder beim Laden ist er ungenau.
* Unter 3,50 V (einstellbar) erscheint eine Warnung. Unter 3,35 V (kritisch) wird das
  Schreiben in den Flash gesperrt, und nach 30 s geht das Gerät in Deep-Sleep.

## Energiesparen

| Zustand | Auslöser | Aufwecken |
|---|---|---|
| Dimmen (20 %) | keine Berührung und keine Bewegung für `dimAfterSec` | Berührung, Bewegung, BOOT |
| Display aus (Panel-Sleep, Backlight aus) | `offAfterSec` | Berührung (CST816S bleibt aktiv), Bewegung, BOOT |
| Deep-Sleep | Menü „Ausschalten“, `sleepAfterMin` Inaktivität oder kritischer Akku | **BOOT-Taste (GPIO0, ext0)** |

Vor dem Deep-Sleep werden ausstehende Einstellungen gespeichert, der QMI8658 wird
abgeschaltet (CTRL7 = 0, Oszillator aus), der CST816S in Deep-Sleep versetzt und das
Display ausgeschaltet. Das ESP32-S3 kann über ext0 an RTC-GPIO0 aufwachen. Einen
Aufweck-Interrupt über den Touchcontroller (GPIO5) nutzt die Firmware bewusst nicht,
weil der CST816S im Deep-Sleep nur per Reset aufwacht.

## Flash-Partitionierung (`partitions.csv`)

| Name | Typ | Offset | Größe |
|---|---|---|---|
| nvs | NVS (Einstellungen, Kalibrierung) | 0x9000 | 20 KB |
| otadata | OTA | 0xE000 | 8 KB |
| app0 | OTA-Slot 0 | 0x10000 | 6 MB |
| app1 | OTA-Slot 1 | 0x610000 | 6 MB |
| spiffs | LittleFS (Profile) | 0xC10000 | 3,9 MB |
| coredump | Core-Dump | 0xFF0000 | 64 KB |

NVS und LittleFS bleiben bei Firmware-Updates (OTA oder `pio run -t upload`) erhalten.
Nur `pio run -t erase` löscht sie.

## Erweiterungsschnittstellen

`sda::IOrientationSensor` (`lib/DrillCore/src/ImuTypes.h`) abstrahiert Sensoren:
`begin()`, `readSample()`, `isHealthy()`, `info()`, `setPowerDown()`. Weitere Sensoren
wie Präzisions-Inklinometer, eine zweite IMU oder eine drahtlose Referenzeinheit
implementieren diese Schnittstelle und liefern `ImuSample` mit eigener `sourceId`. Die
Pipeline kennt keine Chipdetails. Freie GPIOs sind im Schaltplan zu bestimmen.
Nicht verwendet werden die IMU-Interrupts GPIO3/4, die für Wake-on-Motion vorgesehen sind.
