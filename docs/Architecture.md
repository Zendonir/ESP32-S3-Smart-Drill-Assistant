# Softwarearchitektur

## Schichten

```
┌───────────────────────────── src/ (Gerät) ─────────────────────────────┐
│ ui/  UIManager · MainScreen · MenuScreen · SettingsScreen ·             │
│      CalibrationScreen · DiagnosticsScreen · Theme · fonts              │
│ network/ NetworkManager (WLAN, mDNS) · WebApi (REST, WebSocket, OTA)    │
│ storage/ SettingsManager (NVS) · ProfileManager (LittleFS) ·            │
│          CalibrationStorage (NVS) · ConfigJson                          │
│ system/  SystemTask (Akku, Energie, Taste, Flash-Debounce) · OtaGuard · │
│          CpuMonitor · Log · SharedState · WriteGuard · AppContext       │
│ measurement/ MeasurementService (IMU-Task, Fusions-Task, Kommandos)     │
│ hardware/ I2CBus · IMUDriver (QMI8658) · TouchDriver (CST816S) ·        │
│           DisplayDriver (GC9A01A + LVGL) · BatteryManager · BootButton  │
├────────────────── lib/DrillCore (hardwareunabhängig) ──────────────────┤
│ MeasurementPipeline ← SensorFusion · SensorCalibration · MountCalibration│
│ MotionAnalysis · MeasurementQuality · Orientation · ReferenceManager ·  │
│ DisplayLogic · Statistics · TimingEstimator · BatteryModel · Settings · │
│ CoordinateTransforms · LinearAlgebra · MathTypes · ImuTypes             │
└─────────────────────────────────────────────────────────────────────────┘
```

`lib/DrillCore` bindet weder Arduino, ESP-IDF noch FreeRTOS ein. Es wird unverändert auf
dem Gerät und in den Host-Tests übersetzt.

## Tasks

| Task | Kern | Prio | Periode | Aufgabe |
|---|---|---|---|---|
| imu | 1 | 20 | datengetrieben (~224 Hz, Poll 1 ms) | Sensor lesen, Zeitstempel, Fehler/Timeout, Bus-Recovery, Queue |
| fusion | 1 | 19 | pro Sample | Kommandos, `MeasurementPipeline::process`, Snapshot veröffentlichen, Lastmessung |
| ui | 1 | 5 | 40 FPS (25 ms) | LVGL, Touch, Gesten, Modi, Menüs |
| net | 0 | 3 | 2 ms bei aktivem WLAN | WebServer, WebSocket (gedrosselt), WLAN-Automat |
| system | 0 | 2 | 10 ms / 500 ms | Taste, Akku, Dimmen, Deep-Sleep, verzögertes Speichern, OTA-Bestätigung, CPU-Last |
| log | 0 | 1 | bei Bedarf | Ringpuffer → USB-Serial |

Der WLAN-Stack des ESP-IDF läuft auf Kern 0. Messung und UI laufen ungestört auf Kern 1,
wobei die Messung Vorrang vor der UI hat.

## Kommunikation und Thread-Sicherheit

* **Messdaten:** Der Fusions-Task schreibt `OrientationSnapshot` (POD) in
  `SharedState<T>`, einen Spinlock-geschützten memcpy. Die Leser (UI, Netzwerk, System)
  bekommen stets eine konsistente Kopie.
* **Kommandos an die Messung:** FreeRTOS-Queue `MeasurementService::Command`.
  Synchrone Anfragen (Lösen der Kalibrierung, Kalibrierung lesen oder setzen) laufen über
  eine Request-Mutex, einen Staging-Puffer und ein binäres Semaphor mit Timeout.
  Die Pipeline wird **ausschließlich** im Fusions-Task benutzt.
* **Kommandos an die UI:** Queue `UiCommand`, die einzige Schnittstelle der anderen
  Tasks zu LVGL. Nach `setup()` ruft nur der UI-Task LVGL auf. LVGL ist nicht
  threadsicher.
* **I²C:** `I2CBus` mit Mutex (Prioritätsvererbung) pro Transaktion. Touch (UI-Task)
  und IMU (IMU-Task) teilen den Bus.
* **SPI:** Nur der UI-Task greift zu (LovyanGFX-DMA). Display-Sleep und -Wake laufen
  über `UiCommand`.
* **Einstellungen und Profile:** Manager mit Mutex und Versionszähler. Die Tasks merken
  Änderungen am Versionszähler und lesen dann eine Kopie.
* **Flash:** Schreiben ist um 4 s entprellt und wird nur ausgeführt, wenn sich der
  Inhalt geändert hat (Hash). Bei kritischem Akku sperrt `WriteGuard` den Zugriff.
  Profile werden über eine temporäre Datei und Umbenennen geschrieben, also atomar.

## Speicher und Laufzeit

* Im Messpfad gibt es keine Heap-Allokation. Alle Puffer der Pipeline (Präzisionsfenster
  2400 Werte, Kalibrierakkumulatoren) sind statisch.
* Für LVGL stehen 64 KB Pool und 2 × 19,2 KB DMA-Puffer im internen RAM bereit. Der
  WLAN- und LWIP-Speicher liegt bevorzugt im PSRAM (`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP`).
* Die Diagnose zeigt die Last des Fusions-Tasks und die maximale Rechenzeit pro Sample.
  Die CPU-Last pro Kern ist eine Schätzung über den Idle-Hook.
* Das Logging blockiert nicht. Formatiert wird im Aufrufer, danach geht der Text über
  einen Ringpuffer an einen Task mit niedriger Priorität. Ist der Puffer voll, wird die
  Meldung verworfen und gezählt.

## Erweiterbarkeit

* **Neue Sensoren:** `IOrientationSensor` implementieren und Samples mit eigener
  `sourceId` liefern. Für die Sensor-Fusion mehrerer Quellen genügt eine neue
  Pipeline-Stufe, z. B. eine Gewichtung über die Neigungsunsicherheit. UI und API
  arbeiten nur auf `OrientationSnapshot`.
* **Neue Fusionsfilter:** von `IFusionFilter` ableiten und in `FusionBank` aufnehmen.
* **Akustik, Vibrationsmotor, externe Taster:** Sie reagieren auf `TargetTracker::justLocked()`
  bzw. `UiCommand` (z. B. `ButtonShort`).
* **Home Assistant / Bluetooth:** Die REST-API und die WebSocket-Livedaten liefern alle
  Größen. Ein BLE-GATT-Dienst kann als weiterer Leser von `SharedState` hinzukommen.
