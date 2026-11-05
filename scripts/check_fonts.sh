#!/usr/bin/env bash
# Renders text with all firmware fonts using LVGL on the host with the firmware's
# lv_conf.h and fails if glyphs are not drawn. Usage: scripts/check_fonts.sh [lvgl-src-dir] [ppm-out-dir]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/.pio/fontcheck"
mkdir -p "$BUILD"
LVGL="${1:-}"
if [ -z "$LVGL" ]; then
  LVGL="$BUILD/lvgl"
  [ -d "$LVGL" ] || git clone -q --depth 1 -b v8.3.11 https://github.com/lvgl/lvgl.git "$LVGL"
fi
# same configuration as the firmware, only the Arduino tick source is replaced
sed -e 's/#define LV_TICK_CUSTOM 1/#define LV_TICK_CUSTOM 0/' "$ROOT/include/lv_conf.h" > "$BUILD/lv_conf.h"
SRCS=$(find "$LVGL/src" -name '*.c')
gcc -O1 -w -DLV_CONF_INCLUDE_SIMPLE -DLV_LVGL_H_INCLUDE_SIMPLE -I"$BUILD" -I"$LVGL" -I"$LVGL/src" \
    $SRCS "$ROOT"/src/ui/fonts/*.c "$ROOT/tools/render_check/render_check.c" -o "$BUILD/render_check" -lm
"$BUILD/render_check" ${2:-}
