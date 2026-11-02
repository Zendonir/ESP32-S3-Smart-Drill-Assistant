# Abhängigkeiten und Versionen

Alle Versionen sind in `platformio.ini` exakt festgelegt.

| Komponente | Version | Quelle | Zweck |
|---|---|---|---|
| PlatformIO Core | 6.2.0 (geprüft), ≥ 6.1 | pypi | Build-System |
| Plattform espressif32 | 6.9.0 | PlatformIO-Registry | Toolchain und Integration |
| Arduino-ESP32 | 2.0.17 (`framework-arduinoespressif32` 3.20017) | Espressif | Framework, ESP-IDF 4.4.7 |
| Toolchain | xtensa-esp32s3-elf GCC 8.4.0 (esp-2021r2-patch5) | Espressif | Compiler |
| LVGL | 8.3.11 (`git+https://github.com/lvgl/lvgl.git#v8.3.11`) | GitHub-Tag | GUI |
| LovyanGFX | 1.1.16 (`#1.1.16`) | GitHub-Tag | GC9A01A-Treiber, SPI-DMA |
| ArduinoJson | 7.2.1 (`#v7.2.1`) | GitHub-Tag | JSON (Einstellungen, API) |
| arduinoWebSockets | 2.6.1 (`#2.6.1`) | GitHub-Tag | WebSocket-Server |
| Unity | 2.6.x | PlatformIO (Test-Framework) | Unit-Tests |
| Eingebaute Core-Bibliotheken | WiFi, WiFiClientSecure, WebServer, Update, Preferences, FS, LittleFS, ESPmDNS | Arduino-ESP32 2.0.17 | – |
| lv_font_conv | 1.5.2 (nur zum Erzeugen der Schriften) | npm | Montserrat Medium + FontAwesome → `src/ui/fonts/*.c` |

Die Bibliotheken werden als Git-Tags eingebunden statt über Registry-Namen. Das macht
die Versionen eindeutig reproduzierbar.

## Kompatibilität

* LVGL 8.3 statt 9.x: Die Konfiguration (`lv_conf.h`), die Treiberschnittstelle
  (`lv_disp_drv_t`/`lv_indev_drv_t`) und die Widget-API sind auf 8.3 ausgelegt.
* LovyanGFX 1.1.16 unterstützt Arduino-ESP32 2.0.x und den ESP32-S3 mit SPI-DMA.
  Übergabe aus LVGL: `LV_COLOR_16_SWAP = 1` und `pushImageDMA(..., lgfx::swap565_t*)`,
  also DMA ohne Konvertierung.
* Arduino-ESP32 2.0.17 hat im sdkconfig `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=1`.
  Darauf baut der OTA-Rollback auf (`src/system/OtaGuard.cpp`, Override von
  `verifyRollbackLater()`).
* Für die I²C-Ansteuerung nutzt die Firmware direkt den Legacy-Treiber von ESP-IDF 4.4
  (`i2c_master_write_read_device`), nicht `Wire`. Das ergibt deterministische
  Timeouts und eine eigene Bus-Recovery.
* Wegen der Makro-Bedingungen in arduinoWebSockets findet der Library Dependency Finder
  `<WiFi.h>` nicht. Die Include-Pfade von WiFi/WiFiClientSecure werden daher in
  `build_flags` gesetzt.

## Schriften neu erzeugen

```bash
npx lv_font_conv@1.5.2 --bpp 4 --size 18 --font Montserrat-Medium.ttf -r 0x20-0x7E,0xA0-0xFF,0x2022,0x2026 \
  --font FontAwesome5-Solid+Brands+Regular.woff -r <LVGL-Symbolliste> --format lvgl \
  --lv-include lvgl.h --force-fast-kern-format -o src/ui/fonts/sda_font_18.c
```

Die TTF- und WOFF-Dateien sowie die Symbolliste stehen im LVGL-Repository unter
`scripts/built_in_font/`. Die Kopfzeile jeder `.c`-Datei enthält die genauen Optionen.
