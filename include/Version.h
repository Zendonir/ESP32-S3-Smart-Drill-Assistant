// SPDX-License-Identifier: MIT
#pragma once

#define SDA_FW_NAME "SmartDrillAssistant"
#define SDA_FW_VERSION_MAJOR 1
#define SDA_FW_VERSION_MINOR 5
#define SDA_FW_VERSION_PATCH 0
#define SDA_FW_VERSION_STR "1.5.0"
// git commit of the build, injected by scripts/embed_web.py
#ifndef SDA_FW_COMMIT
#define SDA_FW_COMMIT "unknown"
#endif
#define SDA_FW_BUILD_DATE __DATE__ " " __TIME__
#define SDA_HW_NAME "Waveshare ESP32-S3-Touch-LCD-1.28"
