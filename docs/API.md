# REST-API und WebSocket

Die API ist nur erreichbar, wenn WLAN aktiv ist (Einstellungen › System › WLAN). Im
AP-Modus heißt das Netz `SmartDrill-XXXX` (Passwort am Gerät), und das Gerät hat die IP
`192.168.4.1`. Im Client-Modus ist es über die vom Router vergebene IP oder über
`http://<hostname>.local/` (mDNS) erreichbar.

## Sicherheit

**Access Point / Captive Portal (Standard):** Der Access Point `SmartDrill-XXXX` ist **offen**.
Nach dem Verbinden öffnen Smartphones und Laptops automatisch die Portalseite: Alle DNS-Anfragen werden auf das Gerät umgeleitet, die Prüf-URLs von Android, Apple, Windows und Firefox per 302.
Geräte, die direkt mit diesem Access Point verbunden sind, brauchen **keinen Login**; das gilt auch für OTA-Updates.
Dahinter stehen zwei Überlegungen:
* Den AP schaltet man nur am Gerät selbst ein (System › WLAN). Wer das kann, hat das Gerät ohnehin in der Hand.
* Die Ausnahme gilt nur für Clients im AP-Subnetz. Im Heimnetz (Client-Modus) bleibt der Login immer Pflicht.

Abschaltbar am Gerät unter System › WLAN: „AP mit Passwort“ (WPA2 mit dem angezeigten AP-Passwort) und „Login im AP“.
Hinweis: Solange der offene AP läuft, kann jeder in Funkreichweite die Einstellungen ändern und Firmware einspielen. AP nach dem Einrichten wieder ausschalten.


* **Alle `/api/*`-Endpunkte** verlangen HTTP-Basic-Auth mit Benutzer `admin` und dem
  Web-Passwort. Das Passwort wird beim ersten Start zufällig erzeugt und am Gerät unter
  System › WLAN › Web-Passwort angezeigt. Ändern lässt es sich über
  `PUT /api/config {"system":{"adminPassword":"..."}}` (8–16 Zeichen).
* Nach 5 Fehlversuchen ist die Anmeldung 30 s gesperrt (HTTP 429).
* WebSocket (Port 81): Verbindung mit `ws://<host>:81/?t=<token>`. Den Token liefert das
  authentifizierte `GET /api/token`. Er wird bei jedem Start neu erzeugt. Es sind höchstens
  3 Clients gleichzeitig verbunden. Der Kanal ist nur lesend.
* Die Übertragung läuft über unverschlüsseltes HTTP. Das Gerät ist für das lokale Netz
  gedacht und sollte nicht ins Internet freigegeben werden.
* Firmware-Updates erfordern Auth und werden bei schwachem oder kritischem Akku
  abgelehnt.

## Fehlerformat

```json
{"error": {"code": "invalid_value", "message": "Ungültiger Wert oder Typ"}}
```

| HTTP | code | Bedeutung |
|---|---|---|
| 400 | `invalid_json` | Body fehlt, ist kein JSON-Objekt oder größer als 12 KB |
| 400 | `update_failed` | Firmware-Upload fehlerhaft (Text mit Grund) |
| 401 | `unauthorized` | Auth fehlt oder ist falsch |
| 404 | `not_found` | unbekannter Endpunkt oder Profilindex |
| 409 | `full`, `last_profile`, `calibration_failed` | Konflikt mit dem Zustand |
| 422 | `invalid_value` | Wert außerhalb des Bereichs oder falscher Typ |
| 429 | `locked` | Auth-Sperre nach Fehlversuchen |
| 503 | `busy`, `write_blocked` | Messsystem belegt, Flash-Schreiben gesperrt (Akku) |

Alle Eingaben werden geprüft: Typprüfung je Feld, Bereichsprüfung mit Reparatur über
`validateSettings`/`validateProfile`, zu lange Strings werden abgelehnt statt
abgeschnitten. Bei `PUT /api/config` werden nur die übergebenen Felder geändert.

## Endpunkte

| Methode | Pfad | Beschreibung |
|---|---|---|
| GET | `/` | Weboberfläche (gzip, ohne Auth; alle Daten kommen aus der API) |
| GET | `/api/status` | `battery.chargeState`, `chargeStateName`, `trendMvPerMin`, `usbHost`; außerdem Firmware, Partition, Bestätigung des Images, Laufzeit, Heap, CPU-Schätzung, Modus, Qualität, Akku, WLAN, Selbsttest |
| GET | `/api/imu` | kalibrierte und rohe Accel/Gyro-Werte (Geräteframe), Temperatur, Rate, verlorene Samples, Fehlerzähler, Vibration, Bewegung, Werkzeugzustand |
| GET | `/api/orientation` | `deviation.angleReference`, `targetValid`, `needsZero`, `targetUsesGyro`, `targetElevationDeg`, `problem`; dazu Quaternion, Up-Vektor, Wasserwaage, Bohrachse (Elevation, Neigung, Rolle, Azimut *driftend*), Abweichung, geschätzte Unsicherheit, Referenz, Präzisionsergebnis |
| GET | `/api/config` | Einstellungen (ohne Passwörter, `passwordSet`) |
| PUT | `/api/config` | Einstellungen teilweise ändern |
| GET | `/api/calibration` | Status des laufenden Verfahrens und Kalibrierdaten |
| DELETE | `/api/calibration` | Sensorkalibrierung löschen |
| POST | `/api/calibration/gyro` | Gyro-Kalibrierung starten (Gerät ruhig). Antwort 202 |
| POST | `/api/calibration/accel` | `{"action":"start","extended":false}` / `{"action":"pose","pose":0..5}` / `{"action":"solve","save":true}` / `{"action":"cancel"}` |
| POST | `/api/calibration/save` | aktuelle Kalibrierung (z. B. nach Gyro) dauerhaft speichern |
| POST | `/api/reference` | `{"type":"tilt"\|"3d"\|"axis"\|"surface"\|"surface3d"\|"clear"}`. `axis` = Bezugsachse für Winkelbohren (3D-Nullpunkt) |
| GET | `/api/profiles` | alle Profile und `active` |
| POST | `/api/profiles` | `{"name":"...","from":idx}`: neues Profil als Kopie. Antwort 201 `{"index":n}` |
| PUT | `/api/profiles/{i}` | Profil teilweise ändern (Felder wie in GET, `mount.axis` wird normalisiert) |
| DELETE | `/api/profiles/{i}` | Profil löschen (nicht das letzte) |
| POST | `/api/profiles/{i}/activate` | Profil aktivieren |
| POST | `/api/mode` | `{"mode":0..5}`: 0 Bohren, 1 Waage, 2 Winkel, 3 Nullpunkt, 4 Referenz, 5 Präzision |
| POST | `/api/hold` | Hold umschalten |
| POST | `/api/target` | `{"angle":0..90,"reference":0..5,"tolerance":0.05..10}` für das aktive Profil. `reference`: 0 zur Horizontalen, 1 zur Vertikalen, 2 Nullpunkt + oben, 3 Nullpunkt + unten, 4 Nullpunkt + links (Gyro), 5 Nullpunkt + rechts (Gyro). Das Feld `angleReference` im Profil hat dieselbe Bedeutung |
| GET | `/api/wifi/scan` | WLAN-Suche (asynchron): `202 {"scanning":true}`, später `{"networks":[{"ssid","rssi","open"}]}` |
| GET | `/api/token` | `{"token":"...","port":81}` |
| GET | `/api/log` | letzte ca. 2 KB Log (text/plain) |
| GET | `/api/diagnostics` | alle Diagnosedaten (CPU, UI-FPS, Touch, I2C, IMU-Timing, Fusion, Speicher, Akku, WLAN, Einstellungen, letztes Log) als JSON für Fehlerberichte |
| GET | `/api/export` | Einstellungen, Profile und Kalibrierung als JSON (ohne Passwörter) |
| POST | `/api/import` | `{"settings":{},"profiles":{},"calibration":{},"importCalibration":true}`: Erst werden alle Teile geprüft, dann wird übernommen |
| POST | `/api/update` | multipart/form-data mit `firmware.bin`. Nach Erfolg folgt ein Neustart. Das neue Image bestätigt sich nach Selbsttest und 30 s Laufzeit, sonst Rollback |
| POST | `/api/reboot` | Neustart (ausstehende Einstellungen werden vorher gespeichert) |

### Einstellungen (Auszug, vollständig in `src/storage/ConfigJson.cpp`)

```json
{
  "version": 1,
  "measurement": {"filterStrength": 2, "fusion": 0, "precisionWindowSec": 3, "holdTimeSec": 0.4,
                  "hemisphere": 0, "zeroKind": 0, "reference3D": false, "customAngleDeg": 22.5,
                  "tempCoeffAssumedDpsPerK": 0.02, "onlineBiasTracking": true, "startupGyroCal": 1, "lastMode": 0},
  "display": {"brightness": 80, "dimAfterSec": 90, "offAfterSec": 600, "rotation": 0, "theme": 0,
              "colorMode": 0, "largeFont": 0, "decimals": 1, "touchLockAuto": false,
              "showHistory": true, "historyWindowSec": 15},
  "system": {"wifiMode": 0, "ssid": "", "hostname": "smartdrill", "mdns": true, "sleepAfterMin": 30,
             "bootButton": 0, "powerOnHold": true, "apSecured": false, "apLogin": false, "axisMap": "+X+Y+Z", "logLevel": 2, "batteryDividerFactor": 3.0,
             "lowBatteryV": 3.5, "criticalBatteryV": 3.35, "activeProfile": 0, "liveRateHz": 10,
             "passwordSet": false}
}
```

Nur schreibbar (werden nie ausgelesen): `system.password`, `system.apPassword`,
`system.adminPassword`.

### Beispiel

```bash
curl -u admin:PASSWORT http://192.168.4.1/api/orientation
curl -u admin:PASSWORT -X PUT -H 'Content-Type: application/json' \
     -d '{"system":{"wifiMode":2,"ssid":"Werkstatt","password":"geheim123"}}' http://192.168.4.1/api/config
curl -u admin:PASSWORT -F firmware=@.pio/build/sda_s3_touch_128/firmware.bin http://192.168.4.1/api/update
```

## WebSocket-Livedaten

Die Daten werden in der eingestellten Rate (`liveRateHz`, 1–25 Hz, Standard 10 Hz)
als JSON gesendet. Das Erzeugen geschieht ohne Heap per `snprintf` im Netzwerk-Task
(Kern 0) und beeinflusst die Messung auf Kern 1 nicht.

| Feld | Bedeutung |
|---|---|
| `t` | ms seit Start |
| `mode`, `hold` | Betriebsmodus, Hold aktiv |
| `q`, `qn` | Qualitätszustand (Zahl, Kurztext) |
| `dev` | [x, y, gesamt] Abweichung in ° (Anzeige, geglättet) |
| `ts` | Zielzustand 0 ungültig, 1 weit, 2 nah, 3 innen, 4 im Ziel |
| `target`, `tol` | Sollwinkel, Toleranz |
| `val` | Hauptwert des Modus (z. B. Neigung) |
| `tilt` | [X, Y] Wasserwaage in ° |
| `acc`, `gyr` | kalibriert, g bzw. °/s (Geräteframe) |
| `temp` | Sensortemperatur °C |
| `vib`, `vl` | Vibrations-RMS mg, Stufe 0–2 |
| `motion` | 0 Stillstand, 1 langsam, 2 schnell |
| `sig`, `hsig` | geschätzte Neigungsunsicherheit und Referenz-Azimutunsicherheit in ° |
| `rate`, `lost` | gemessene Abtastrate, verlorene Samples |
| `bat`, `pct` | Akkuspannung, geschätzter Ladestand (−1 = unbekannt) |
| `gcal`, `acal` | Gyro- bzw. Accel-Kalibrierung vorhanden |
| `chg` | Ladezustand: 0 unbekannt, 1 Akkubetrieb, 2 lädt (abgeleitet), 3 voll an USB, 4 kein Akku |
| `cal` | `{p: Verfahren, pr: Fortschritt, mask: akzeptierte Lagen, det: erkannte Lage, act: aktive Lage, seq: Ergebniszähler}` |
| `fps` | UI-Bildrate |
