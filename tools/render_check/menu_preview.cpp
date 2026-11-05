// Host preview of the menu layout (MenuScreen + Theme are pure LVGL).
// Renders a WLAN-like menu into a PPM image for visual inspection.
#include <cstdio>

#include "../../src/ui/MenuScreen.h"
#include "../../src/ui/Theme.h"
#include "lvgl.h"

static lv_color_t buf[240 * 240], fb[240 * 240];
static void flush(lv_disp_drv_t* d, const lv_area_t* a, lv_color_t* p) {
    for (int y = a->y1; y <= a->y2; ++y)
        for (int x = a->x1; x <= a->x2; ++x) fb[y * 240 + x] = *p++;
    lv_disp_flush_ready(d);
}

int main(int argc, char** argv) {
    lv_init();
    static lv_disp_draw_buf_t db;
    lv_disp_draw_buf_init(&db, buf, nullptr, 240 * 240);
    static lv_disp_drv_t dd;
    lv_disp_drv_init(&dd);
    dd.hor_res = dd.ver_res = 240;
    dd.flush_cb = flush;
    dd.draw_buf = &db;
    lv_disp_drv_register(&dd);
    theme::apply(0, 0);
    static MenuScreen m;
    m.open("WLAN",
           {MenuItem::info("AP-Passwort", [] { return std::string("Xk7pRqbbdx7"); }),
            MenuItem::info("Web-Login", [] { return std::string("admin"); }),
            MenuItem::info("Web-Passwort", [] { return std::string("9yCHwDKXaQ"); }),
            MenuItem::info("Hostname", [] { return std::string("smartdrill.local"); }),
            MenuItem::submenu("Untermenü", [] {})},
           [] {});
    // scroll so that the first data rows are in view
    lv_obj_scroll_by(lv_obj_get_child(m.obj(), 1), 0, argc > 2 ? -atoi(argv[2]) : -40, LV_ANIM_OFF);
    lv_refr_now(nullptr);
    FILE* f = fopen(argc > 1 ? argv[1] : "menu.ppm", "wb");
    fprintf(f, "P6\n240 240\n255\n");
    for (int i = 0; i < 240 * 240; ++i) {
        lv_color32_t c;
        c.full = lv_color_to32(fb[i]);
        const int x = i % 240 - 120, y = i / 240 - 120;
        const bool in = x * x + y * y <= 120 * 120;  // round panel mask
        fputc(in ? c.ch.red : 40, f);
        fputc(in ? c.ch.green : 40, f);
        fputc(in ? c.ch.blue : 40, f);
    }
    fclose(f);
    return 0;
}
