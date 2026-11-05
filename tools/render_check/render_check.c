/* SPDX-License-Identifier: MIT
 * Host check: renders text with every firmware font through LVGL (using the
 * firmware's lv_conf.h) into an off-screen buffer and verifies that glyphs are
 * actually drawn. Catches font/config mismatches (e.g. compressed fonts with
 * LV_USE_FONT_COMPRESSED 0), which compile fine but show no text on the device.
 * Optionally writes a PPM image per font for visual inspection.
 */
#include <stdio.h>
#include <string.h>

#include "lvgl.h"

#define W 240
#define H 60

static lv_color_t buf[W * H];
static lv_color_t fb[W * H];
static int flushed;

static void flush_cb(lv_disp_drv_t* d, const lv_area_t* a, lv_color_t* px) {
    for (int y = a->y1; y <= a->y2; ++y)
        for (int x = a->x1; x <= a->x2; ++x) fb[y * W + x] = *px++;
    flushed = 1;
    lv_disp_flush_ready(d);
}

static lv_obj_t* scr;

static int render(const lv_font_t* font, const char* text, const char* ppm) {
    lv_obj_clean(scr);
    lv_obj_t* l = lv_label_create(scr);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_white(), 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    memset(fb, 0, sizeof(fb));
    lv_obj_invalidate(scr);
    lv_refr_now(NULL);
    int lit = 0;
    for (int i = 0; i < W * H; ++i)
        if (lv_color_brightness(fb[i]) > 128) ++lit;
    if (ppm) {
        FILE* f = fopen(ppm, "wb");
        if (f) {
            fprintf(f, "P6\n%d %d\n255\n", W, H);
            for (int i = 0; i < W * H; ++i) {
                lv_color32_t c;
                c.full = lv_color_to32(fb[i]);
                fputc(c.ch.red, f);
                fputc(c.ch.green, f);
                fputc(c.ch.blue, f);
            }
            fclose(f);
        }
    }
    return lit;
}

int main(int argc, char** argv) {
    const char* outdir = argc > 1 ? argv[1] : NULL;
    lv_init();
    static lv_disp_draw_buf_t db;
    lv_disp_draw_buf_init(&db, buf, NULL, W * H);
    static lv_disp_drv_t dd;
    lv_disp_drv_init(&dd);
    dd.hor_res = W;
    dd.ver_res = H;
    dd.flush_cb = flush_cb;
    dd.draw_buf = &db;
    lv_disp_drv_register(&dd);
    scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_scr_load(scr);

    struct { const lv_font_t* f; const char* name; const char* text; int minPx; } cases[] = {
        {&sda_font_14, "14", "Größe ±0,5° " LV_SYMBOL_OK LV_SYMBOL_BATTERY_FULL LV_SYMBOL_CHARGE, 80},
        {&sda_font_18, "18", "Kalibrierung äöü ß " LV_SYMBOL_SETTINGS, 100},
        {&sda_font_24, "24", "BOHREN 45° " LV_SYMBOL_POWER, 150},
        {&sda_font_44, "44", "-12.5°", 300},
    };
    int fail = 0;
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        char path[256];
        if (outdir) snprintf(path, sizeof(path), "%s/font_%s.ppm", outdir, cases[i].name);
        const int lit = render(cases[i].f, cases[i].text, outdir ? path : NULL);
        const int ok = lit >= cases[i].minPx;
        printf("font %s: %d lit pixels %s\n", cases[i].name, lit, ok ? "OK" : "FAIL (text not rendered)");
        if (!ok) fail = 1;
    }
    if (!flushed) {
        printf("FAIL: nothing flushed\n");
        fail = 1;
    }
    return fail;
}
