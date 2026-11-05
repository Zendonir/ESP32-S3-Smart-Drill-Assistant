// SPDX-License-Identifier: MIT
#include "DisplayDriver.h"

#include <Arduino.h>
#include <esp_heap_caps.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lvgl.h>

#include "../system/LoadEvents.h"
#include "../system/Log.h"
#include "AppConfig.h"
#include "BoardConfig.h"
#include "TouchDriver.h"

static const char* TAG = "lcd";

namespace {

class LGFX_GC9A01 : public lgfx::LGFX_Device {
public:
    LGFX_GC9A01() {
        {
            auto cfg = bus_.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = board::kLcdSpiFreqHz;
            cfg.freq_read = 16000000;
            cfg.spi_3wire = false;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = board::kLcdClk;
            cfg.pin_mosi = board::kLcdMosi;
            cfg.pin_miso = board::kLcdMiso;
            cfg.pin_dc = board::kLcdDc;
            bus_.config(cfg);
            panel_.setBus(&bus_);
        }
        {
            auto cfg = panel_.config();
            cfg.pin_cs = board::kLcdCs;
            cfg.pin_rst = board::kLcdRst;
            cfg.pin_busy = -1;
            cfg.memory_width = board::kLcdWidth;
            cfg.memory_height = board::kLcdHeight;
            cfg.panel_width = board::kLcdWidth;
            cfg.panel_height = board::kLcdHeight;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.readable = false;
            cfg.invert = board::kLcdInvertColors;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;
            panel_.config(cfg);
        }
        {
            auto cfg = light_.config();
            cfg.pin_bl = board::kLcdBacklight;
            cfg.invert = false;
            cfg.freq = board::kBacklightPwmFreq;
            cfg.pwm_channel = board::kBacklightPwmChannel;
            light_.config(cfg);
            panel_.setLight(&light_);
        }
        setPanel(&panel_);
    }

private:
    lgfx::Bus_SPI bus_;
    lgfx::Panel_GC9A01 panel_;
    lgfx::Light_PWM light_;
};

// File local driver state (LVGL callbacks are C style)
LGFX_GC9A01 g_lcd;
lv_disp_draw_buf_t g_drawBuf;
lv_disp_drv_t g_dispDrv;
lv_indev_drv_t g_indevDrv;
lv_color_t* g_buf1 = nullptr;
lv_color_t* g_buf2 = nullptr;
TouchDriver* g_touch = nullptr;
volatile bool g_pressed = false;
volatile int16_t g_tx = 0, g_ty = 0;
volatile uint32_t g_flushes = 0;
uint8_t g_rot = 0;

void flushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* px) {
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    g_lcd.startWrite();
    // LV_COLOR_16_SWAP = 1: buffer is already in panel byte order -> pure DMA
    g_lcd.pushImageDMA(area->x1, area->y1, w, h, reinterpret_cast<lgfx::swap565_t*>(&px->full));
    g_lcd.endWrite();
    g_flushes = g_flushes + 1;
    lv_disp_flush_ready(drv);
}

void mapTouch(int16_t nx, int16_t ny, int16_t& x, int16_t& y) {
    const int16_t W = board::kLcdWidth, H = board::kLcdHeight;
    if (board::kTouchSwapXY) { const int16_t t = nx; nx = ny; ny = t; }
    if (board::kTouchInvertX) nx = static_cast<int16_t>(W - 1 - nx);
    if (board::kTouchInvertY) ny = static_cast<int16_t>(H - 1 - ny);
    switch (g_rot & 3) {
        case 0: x = nx; y = ny; break;
        case 1: x = ny; y = static_cast<int16_t>(W - 1 - nx); break;
        case 2: x = static_cast<int16_t>(W - 1 - nx); y = static_cast<int16_t>(H - 1 - ny); break;
        default: x = static_cast<int16_t>(H - 1 - ny); y = nx; break;
    }
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= W) x = W - 1;
    if (y >= H) y = H - 1;
}

void touchCb(lv_indev_drv_t*, lv_indev_data_t* data) {
    int16_t nx = 0, ny = 0;
    bool pressed = false;
    if (g_touch) g_touch->read(nx, ny, pressed);
    int16_t x, y;
    mapTouch(nx, ny, x, y);
    g_pressed = pressed;
    g_tx = x;
    g_ty = y;
    data->point.x = x;
    data->point.y = y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

}  // namespace

bool DisplayDriver::begin(uint8_t rotation) {
    rotation_ = rotation & 3;
    g_rot = rotation_;
    ok_ = g_lcd.init();
    if (!ok_) {
        LOGE(TAG, "GC9A01 init failed");
        return false;
    }
    g_lcd.setRotation(rotation_);
    g_lcd.fillScreen(TFT_BLACK);
    setBrightness(brightness_);
    LOGI(TAG, "GC9A01 %dx%d, SPI %lu MHz", board::kLcdWidth, board::kLcdHeight,
         static_cast<unsigned long>(board::kLcdSpiFreqHz / 1000000));
    return true;
}

bool DisplayDriver::initLvgl(TouchDriver* touch) {
    g_touch = touch;
    lv_init();
    const size_t px = static_cast<size_t>(board::kLcdWidth) * appcfg::kLvglBufferLines;
    // DMA capable internal RAM for both draw buffers
    g_buf1 = static_cast<lv_color_t*>(heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    g_buf2 = static_cast<lv_color_t*>(heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!g_buf1 || !g_buf2) {
        LOGE(TAG, "LVGL draw buffer allocation failed");
        return false;
    }
    lv_disp_draw_buf_init(&g_drawBuf, g_buf1, g_buf2, px);
    lv_disp_drv_init(&g_dispDrv);
    g_dispDrv.hor_res = board::kLcdWidth;
    g_dispDrv.ver_res = board::kLcdHeight;
    g_dispDrv.flush_cb = flushCb;
    g_dispDrv.draw_buf = &g_drawBuf;
    lv_disp_drv_register(&g_dispDrv);

    lv_indev_drv_init(&g_indevDrv);
    g_indevDrv.type = LV_INDEV_TYPE_POINTER;
    g_indevDrv.read_cb = touchCb;
    g_indevDrv.long_press_time = appcfg::kLongPressMs;
    g_indevDrv.gesture_limit = 40;          // px of travel before a swipe is recognised
    g_indevDrv.gesture_min_velocity = 6;    // px per read period
    g_indevDrv.scroll_limit = 12;
    lv_indev_drv_register(&g_indevDrv);
    return true;
}

void DisplayDriver::setRotation(uint8_t rotation) {
    rotation_ = rotation & 3;
    g_rot = rotation_;
    g_lcd.setRotation(rotation_);
    lv_obj_invalidate(lv_scr_act());
}

void DisplayDriver::setBrightness(uint8_t percent) {
    if (percent > 100) percent = 100;
    if (percent != brightness_) load_events::mark();
    brightness_ = percent;
    if (!sleeping_) g_lcd.setBrightness(static_cast<uint8_t>(percent * 255 / 100));
}

void DisplayDriver::sleep() {
    if (sleeping_) return;
    load_events::mark();
    g_lcd.setBrightness(0);
    g_lcd.sleep();
    sleeping_ = true;
}

void DisplayDriver::wake() {
    if (!sleeping_) return;
    load_events::mark();
    g_lcd.wakeup();
    sleeping_ = false;
    setBrightness(brightness_);
    lv_obj_invalidate(lv_scr_act());
}

bool DisplayDriver::touchPressed() const { return g_pressed; }
int16_t DisplayDriver::touchX() const { return g_tx; }
int16_t DisplayDriver::touchY() const { return g_ty; }
uint32_t DisplayDriver::flushCount() const { return g_flushes; }
