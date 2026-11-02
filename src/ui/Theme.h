// SPDX-License-Identifier: MIT
// Visual design for the round 240x240 display: colours, fonts and geometry.
//
// Safe zone: everything essential lies inside a circle of radius 104 px around the
// centre (the panel's visible circle has r = 120 px; bezels hide a few pixels and
// text near the rim is clipped by the circle).
#pragma once

#include <lvgl.h>

#include "DisplayLogic.h"
#include "MeasurementQuality.h"

namespace theme {

constexpr int kCx = 120;
constexpr int kCy = 120;
constexpr int kSafeRadius = 104;
constexpr int kTargetRadius = 100;   ///< radius of the deviation scale circle

struct Palette {
    lv_color_t bg, fg, dim, grid, accent;
    lv_color_t ok, near, far, invalid, hold, warn;
};

void apply(uint8_t themeIdx, uint8_t colorMode);
const Palette& pal();
lv_color_t targetColor(sda::TargetState s);
lv_color_t qualityColor(sda::QualityState q);

inline const lv_font_t* small() { return &sda_font_14; }
inline const lv_font_t* medium() { return &sda_font_18; }
inline const lv_font_t* large() { return &sda_font_24; }
inline const lv_font_t* huge() { return &sda_font_44; }

/// Creates a plain, non-scrollable, non-clickable container.
lv_obj_t* plain(lv_obj_t* parent);
/// Creates a circle object (border only or filled).
lv_obj_t* circle(lv_obj_t* parent, int diameter, lv_color_t color, int border, bool filled);
/// Label helper.
lv_obj_t* label(lv_obj_t* parent, const lv_font_t* font, lv_color_t color, const char* text);
/// Round screen background with theme colour.
lv_obj_t* screen();

}  // namespace theme
