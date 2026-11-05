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
3. **Displayfarben.** Einstellungen › System › **Farbtest** zeigt Balken in Schwarz, Grau 12 %, Grau 38 %, Weiß, Rot, Grün und Blau.
   Mit bloßem Auge prüfen (Kamerafotos verfälschen dunkle Töne):
   * Schwarz muss schwarz sein und Weiß weiß. Sind die beiden vertauscht, `kLcdInvertColors` in `BoardConfig.h` umschalten.
   * Rot und Blau müssen stimmen. Sind sie vertauscht, `cfg.rgb_order` in `src/hardware/DisplayDriver.cpp` umschalten.
   * Grau 12 % muss deutlich dunkler sein als Grau 38 %.
4. **Touch.** Diagnose › „Touch x/y“ zeigt die Koordinate unter dem Finger
   (oben links ≈ 0/0). Stimmt sie nicht, `kTouchSwapXY`/`kTouchInvertX/Y` anpassen.
5. **IMU-Achsen.** Der Standard ist `-Y-X-Z`. Diese Zuordnung wurde an einem echten ESP32-S3-Touch-LCD-1.28 mit dem Achsen-Assistenten ermittelt.
   Bei einer anderen Boardrevision oder wenn der Punkt in die falsche Richtung läuft, den Assistenten erneut ausführen: Menü › Kalibrierung › **Achsen erkennen**.
   1. Gerät mit dem Display nach oben flach hinlegen.
   2. Danach aufrecht hinstellen, sodass der Text auf dem Display lesbar ist.

   Die Firmware bestimmt daraus, wie der QMI8658 auf der Platine sitzt (alle 24 möglichen Lagen) und zeigt die Zuordnung an, z. B. `-Y+X+Z`.
   Eine bereits vorhandene Kalibrierung wird beim Speichern in das korrigierte Koordinatensystem umgerechnet und bleibt gültig.
   Gesetzte Nullpunkte werden gelöscht.
   Wird später die Displayrotation geändert, muss der Assistent erneut laufen, denn „oben“ ist die angezeigte Oberkante.
   Manuell geht es auch: Einstellungen › System › „Achsen S→D“.
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
* Ladeerkennung (heuristisch): Ist ein USB-Host angeschlossen (Start-of-Frame-Aktivität am USB-Serial/JTAG-Controller), gilt der Akku als „lädt“, ab 4,15 V als „voll (USB)“.
  Ohne Datenverbindung (Netzteil) wird Laden an einer steigenden Spannung erkannt: mindestens 6 mV/min über ein 2-Minuten-Fenster.
  Ein Ladestatus-Pin ist der Firmware nicht bekannt.
  Falls der Schaltplan einen CHRG/STDBY-Ausgang an einem GPIO zeigt, sollte er stattdessen ausgewertet werden.
* Die Firmware misst ansonsten nur die Spannung. Ob gerade geladen wird, ist ohne Statuspin nicht
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
| Deep-Sleep („Aus“) | BOOT 3 s halten, Menü „Ausschalten“, `sleepAfterMin` Inaktivität oder kritischer Akku (nicht während des Ladens) | **BOOT-Taste (GPIO0, ext0)**, standardmäßig 1 s halten |

**Ladeerkennung im Aus-Zustand.** Mit der Einstellung „Ladeanzeige: Auch wenn aus“ wacht das Gerät per RTC-Timer alle 30 s auf.
Es misst die Akkuspannung (32 ADC-Werte, ohne Display, WLAN und Sensoren) und vergleicht sie mit der vorigen Messung im RTC-Speicher.
Steigt die Spannung um mindestens 60 mV oder liegt sie über 4,30 V, startet es mit der Ladeanzeige; sonst schläft es nach ca. 0,1 s weiter.
Die ersten zwei Prüfungen nach dem Ausschalten zählen nicht, weil die Zellspannung nach dem Wegfall der Last ohnehin ansteigt.
Der zusätzliche mittlere Strombedarf hängt von Boot-Dauer und Board-Ruhestrom ab und ist noch zu messen; grob sind es einige zehn µA.
Ein Ladegerät, das bereits beim Ausschalten steckt, löst keinen Sprung aus. Für diesen Fall zeigt das Gerät beim Ausschalten direkt die Ladeanzeige.

**USB im Aus-Zustand:** Im Tiefschlaf ist der USB-Serial/JTAG-Controller abgeschaltet, deshalb meldet sich das Gerät nicht am PC. Zum Flashen das Gerät einschalten oder BOOT+RESET (Download-Modus) verwenden. Wird das Gerät ausgeschaltet, während es bereits am USB hängt, bleibt es wach (Ladeanzeige), und USB bleibt erreichbar.

**Ein/Aus-Taste.** Das Board hat keinen eigenen Einschalter, deshalb dient BOOT als Ein/Aus-Taste.
„Aus“ ist Deep-Sleep, keine Trennung vom Akku.
Vor dem Schlafen wartet die Firmware, bis die Taste losgelassen ist (sonst würde ext0 sofort wieder wecken).
Außerdem hält sie den Backlight-Pin über `gpio_hold` auf LOW, damit die Hintergrundbeleuchtung im Schlaf nicht durch einen offenen Pin glimmt.
Nach dem Aufwachen wird die Taste 1 s lang geprüft; wird sie früher losgelassen, schläft das Gerät sofort wieder ein.
Den Ruhestrom im Aus-Zustand (ESP32-S3, Spannungsregler, Ladeschaltung, IMU im Power-Down, Touch im Deep-Sleep) bitte mit einem Strommessgerät ermitteln.
Er hängt von der Boardbeschaltung ab und ist hier nicht verifiziert.
Für lange Lagerung empfiehlt sich ein **Schiebeschalter in der Akkuleitung** (Plusleitung zwischen Akku und Stecker), der den Akku wirklich trennt.
Geladen werden kann dann nur in Stellung „Ein“.

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
