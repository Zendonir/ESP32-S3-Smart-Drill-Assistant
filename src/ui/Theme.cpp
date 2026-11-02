// SPDX-License-Identifier: MIT
#include "Theme.h"

namespace theme {
namespace {
Palette g_pal;
}

void apply(uint8_t themeIdx, uint8_t colorMode) {
    Palette p;
    p.bg = lv_color_hex(0x000000);
    p.fg = lv_color_hex(themeIdx == 1 ? 0xFFFFFF : 0xE8EAED);
    p.dim = lv_color_hex(themeIdx == 1 ? 0xC0C0C0 : 0x8A9099);
    p.grid = lv_color_hex(themeIdx == 1 ? 0x606060 : 0x30343A);
    p.accent = lv_color_hex(0x4FC3F7);
    p.hold = lv_color_hex(0x5C9DFF);
    p.invalid = lv_color_hex(0x7A7F87);
    if (colorMode == 1) {
        // colour-blind friendly: blue = ok, yellow = near, orange-red = far
        p.ok = lv_color_hex(0x2F9BFF);
        p.near = lv_color_hex(0xFFD23F);
        p.far = lv_color_hex(0xFF6B1A);
    } else {
        p.ok = lv_color_hex(0x2BD96B);
        p.near = lv_color_hex(0xFFD000);
        p.far = lv_color_hex(0xFF5A1F);
    }
    p.warn = lv_color_hex(0xFFA000);
    g_pal = p;
}

const Palette& pal() { return g_pal; }

lv_color_t targetColor(sda::TargetState s) {
    switch (s) {
        case sda::TargetState::Locked: return g_pal.ok;
        case sda::TargetState::Inside: return g_pal.near;  // inside, hold time not yet over
        case sda::TargetState::Near: return g_pal.near;
        case sda::TargetState::Far: return g_pal.far;
        case sda::TargetState::Invalid: return g_pal.invalid;
    }
    return g_pal.invalid;
}

lv_color_t qualityColor(sda::QualityState q) {
    switch (q) {
        case sda::QualityState::Stable: return g_pal.ok;
        case sda::QualityState::Motion: return g_pal.dim;
        case sda::QualityState::Vibration: return g_pal.warn;
        case sda::QualityState::CalibrationRequired: return g_pal.warn;
        case sda::QualityState::SensorFault: return g_pal.far;
        case sda::QualityState::OrientationLimited: return g_pal.far;
        case sda::QualityState::ReferenceDrift: return g_pal.warn;
    }
    return g_pal.dim;
}

lv_obj_t* plain(lv_obj_t* parent) {
    lv_obj_t* o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(o, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    return o;
}

lv_obj_t* circle(lv_obj_t* parent, int d, lv_color_t color, int border, bool filled) {
    lv_obj_t* o = plain(parent);
    lv_obj_set_size(o, d, d);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(o, border, 0);
    lv_obj_set_style_border_color(o, color, 0);
    lv_obj_set_style_border_opa(o, border > 0 ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(o, color, 0);
    lv_obj_set_style_bg_opa(o, filled ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    return o;
}

lv_obj_t* label(lv_obj_t* parent, const lv_font_t* font, lv_color_t color, const char* text) {
    lv_obj_t* l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_label_set_text(l, text ? text : "");
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(l, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    return l;
}

lv_obj_t* screen() {
    lv_obj_t* s = lv_obj_create(nullptr);
    lv_obj_remove_style_all(s);
    lv_obj_set_size(s, 240, 240);
    lv_obj_set_style_bg_color(s, g_pal.bg, 0);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
    return s;
}

}  // namespace theme
