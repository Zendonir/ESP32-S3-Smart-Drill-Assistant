// SPDX-License-Identifier: MIT
// Technical diagnostics page (scrollable text, refreshed at 2 Hz).
#pragma once

#include <lvgl.h>

#include <functional>

#include "../system/AppContext.h"

class DiagnosticsScreen {
public:
    void open(AppContext& app, std::function<void()> onBack, float* uiFps);
    void update();
    bool isOpen() const { return scr_ != nullptr; }
    void close();

private:
    AppContext* app_ = nullptr;
    lv_obj_t* scr_ = nullptr;
    lv_obj_t* text_ = nullptr;
    std::function<void()> onBack_;
    float* fps_ = nullptr;
    static void gesture(lv_event_t* e);
};
