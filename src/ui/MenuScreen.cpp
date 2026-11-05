// SPDX-License-Identifier: MIT
#include "MenuScreen.h"

#include <cmath>
#include <cstdio>

#include "Theme.h"

MenuItem MenuItem::submenu(const std::string& l, std::function<void()> open) {
    MenuItem m;
    m.kind = Kind::Submenu;
    m.label = l;
    m.action = std::move(open);
    m.value = [] { return std::string(LV_SYMBOL_RIGHT); };
    return m;
}
MenuItem MenuItem::act(const std::string& l, std::function<void()> fn, std::function<std::string()> v) {
    MenuItem m;
    m.kind = Kind::Action;
    m.label = l;
    m.action = std::move(fn);
    m.value = std::move(v);
    return m;
}
MenuItem MenuItem::toggle(const std::string& l, std::function<bool()> g, std::function<void(bool)> s) {
    MenuItem m;
    m.kind = Kind::Toggle;
    m.label = l;
    m.getBool = g;
    m.setBool = std::move(s);
    m.value = [g] { return std::string(g() ? "Ein" : "Aus"); };
    return m;
}
MenuItem MenuItem::choice(const std::string& l, const std::string& opts, std::function<int()> g,
                          std::function<void(int)> s) {
    MenuItem m;
    m.kind = Kind::Choice;
    m.label = l;
    m.options = opts;
    m.getIndex = g;
    m.setIndex = std::move(s);
    m.value = [opts, g] {
        const int idx = g();
        size_t start = 0;
        for (int i = 0; i < idx; ++i) {
            start = opts.find('\n', start);
            if (start == std::string::npos) return std::string("?");
            ++start;
        }
        const size_t end = opts.find('\n', start);
        return opts.substr(start, end == std::string::npos ? std::string::npos : end - start);
    };
    return m;
}
MenuItem MenuItem::number(const std::string& l, float mn, float mx, float st, int dec, const std::string& unit,
                          std::function<float()> g, std::function<void(float)> s) {
    MenuItem m;
    m.kind = Kind::Number;
    m.label = l;
    m.minV = mn;
    m.maxV = mx;
    m.step = st;
    m.decimals = dec;
    m.unit = unit;
    m.getNum = g;
    m.setNum = std::move(s);
    m.value = [g, dec, unit] {
        char b[32];
        snprintf(b, sizeof(b), "%.*f%s", dec, double(g()), unit.c_str());
        return std::string(b);
    };
    return m;
}
MenuItem MenuItem::info(const std::string& l, std::function<std::string()> v) {
    MenuItem m;
    m.kind = Kind::Info;
    m.label = l;
    m.value = std::move(v);
    return m;
}
MenuItem MenuItem::confirm(const std::string& l, const std::string& q, std::function<void()> fn) {
    MenuItem m;
    m.kind = Kind::Confirm;
    m.label = l;
    m.confirmText = q;
    m.action = std::move(fn);
    return m;
}

// ---------------------------------------------------------------------------
void MenuScreen::open(const std::string& title, std::vector<MenuItem> items, BackFn onBack) {
    lv_obj_t* old = scr_;
    scr_ = nullptr;
    items_ = std::move(items);
    onBack_ = std::move(onBack);
    valueLabels_.clear();
    modal_ = nullptr;
    const theme::Palette& p = theme::pal();

    scr_ = theme::screen();
    lv_obj_add_event_cb(scr_, gestureEvent, LV_EVENT_GESTURE, this);

    lv_obj_t* t = theme::label(scr_, theme::medium(), p.accent, title.c_str());
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 14);

    list_ = lv_obj_create(scr_);
    lv_obj_remove_style_all(list_);
    lv_obj_set_size(list_, 184, 186);
    lv_obj_align(list_, LV_ALIGN_TOP_MID, 0, 42);
    lv_obj_set_flex_flow(list_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_, 4, 0);
    lv_obj_set_style_pad_bottom(list_, 40, 0);  // last items can be scrolled into the wide part
    lv_obj_set_scrollbar_mode(list_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(list_, LV_DIR_VER);
    lv_obj_add_flag(list_, LV_OBJ_FLAG_SCROLL_MOMENTUM);

    auto addRow = [&](const char* text, lv_event_cb_t cb, void* ud, intptr_t idx) -> lv_obj_t* {
        lv_obj_t* b = lv_btn_create(list_);
        lv_obj_set_size(b, 184, LV_SIZE_CONTENT);
        lv_obj_set_style_min_height(b, 38, 0);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x16191D), 0);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x2A3038), LV_STATE_PRESSED);
        lv_obj_set_style_radius(b, 10, 0);
        lv_obj_set_style_shadow_width(b, 0, 0);
        lv_obj_set_style_pad_hor(b, 10, 0);
        lv_obj_set_style_pad_ver(b, 6, 0);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_set_user_data(b, reinterpret_cast<void*>(idx));
        lv_obj_add_flag(b, LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_t* l = theme::label(b, theme::medium(), p.fg, text);
        lv_obj_set_width(l, 120);
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
        return b;
    };

    addRow(LV_SYMBOL_LEFT " Zurück", backEvent, this, -1);
    for (size_t i = 0; i < items_.size(); ++i) {
        lv_obj_t* row = addRow(items_[i].label.c_str(), itemEvent, this, static_cast<intptr_t>(i));
        lv_obj_t* v = theme::label(row, theme::small(), p.dim, "");
        lv_obj_align(v, LV_ALIGN_RIGHT_MID, 0, 0);
        valueLabels_.push_back(v);
        if (items_[i].kind == MenuItem::Kind::Info) lv_obj_clear_flag(row, LV_OBJ_FLAG_CLICKABLE);
    }
    refresh();
    lv_scr_load(scr_);
    if (old) lv_obj_del_async(old);  // may be called from an event of the old screen
}

void MenuScreen::close() {
    if (!scr_) return;
    lv_obj_t* s = scr_;
    scr_ = nullptr;
    list_ = modal_ = nullptr;
    valueLabels_.clear();
    items_.clear();
    // deleting the active screen is not allowed: caller loads another screen first
    lv_obj_del_async(s);
}

void MenuScreen::refresh() {
    if (!scr_) return;
    for (size_t i = 0; i < items_.size() && i < valueLabels_.size(); ++i) {
        if (items_[i].value) lv_label_set_text(valueLabels_[i], items_[i].value().c_str());
    }
}

void MenuScreen::backEvent(lv_event_t* e) {
    auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
    BackFn fn = self->onBack_;  // copy: the callback may rebuild this menu
    if (fn) fn();
}

void MenuScreen::gestureEvent(lv_event_t* e) {
    auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
    if (self->modal_) return;
    if (lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT && self->onBack_) {
        BackFn fn = self->onBack_;
        fn();
    }
}

void MenuScreen::itemEvent(lv_event_t* e) {
    auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
    lv_obj_t* btn = lv_event_get_current_target(e);
    const intptr_t idx = reinterpret_cast<intptr_t>(lv_obj_get_user_data(btn));
    self->activate(static_cast<int>(idx));
}

void MenuScreen::activate(int idx) {
    if (idx < 0 || idx >= static_cast<int>(items_.size())) return;
    MenuItem& m = items_[idx];
    switch (m.kind) {
        case MenuItem::Kind::Submenu:
        case MenuItem::Kind::Action:
            if (m.action) {
                auto fn = m.action;  // copy: the menu may be rebuilt inside
                fn();
            }
            refresh();
            break;
        case MenuItem::Kind::Toggle:
            if (m.getBool && m.setBool) m.setBool(!m.getBool());
            refresh();
            break;
        case MenuItem::Kind::Choice: openChoice(idx); break;
        case MenuItem::Kind::Number: openNumber(idx); break;
        case MenuItem::Kind::Confirm: openConfirm(idx); break;
        case MenuItem::Kind::Info: break;
    }
}

lv_obj_t* MenuScreen::modalBase() {
    closeModal();
    modal_ = lv_obj_create(scr_);
    lv_obj_remove_style_all(modal_);
    lv_obj_set_size(modal_, 240, 240);
    lv_obj_set_style_bg_color(modal_, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(modal_, LV_OPA_COVER, 0);
    lv_obj_clear_flag(modal_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(modal_, LV_OBJ_FLAG_CLICKABLE);  // swallow clicks behind
    return modal_;
}

lv_obj_t* MenuScreen::modalButton(lv_obj_t* parent, const char* text, int x, int y, int w, lv_event_cb_t cb) {
    lv_obj_t* b = lv_btn_create(parent);
    lv_obj_set_size(b, w, 44);
    lv_obj_align(b, LV_ALIGN_CENTER, x, y);
    lv_obj_set_style_radius(b, 22, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2A3038), 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, this);
    lv_obj_t* l = theme::label(b, theme::large(), theme::pal().fg, text);
    lv_obj_center(l);
    return b;
}

void MenuScreen::closeModal() {
    if (modal_) lv_obj_del(modal_);
    modal_ = nullptr;
    roller_ = nullptr;
    editLabel_ = nullptr;
    editIdx_ = -1;
}

void MenuScreen::openChoice(int idx) {
    MenuItem& m = items_[idx];
    lv_obj_t* base = modalBase();
    editIdx_ = idx;
    lv_obj_t* t = theme::label(base, theme::small(), theme::pal().accent, m.label.c_str());
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 22);
    roller_ = lv_roller_create(base);
    lv_roller_set_options(roller_, m.options.c_str(), LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(roller_, 3);
    lv_obj_set_width(roller_, 184);
    lv_obj_set_style_text_align(roller_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(roller_, theme::medium(), 0);
    lv_obj_set_style_bg_color(roller_, lv_color_hex(0x101214), 0);
    lv_obj_set_style_text_color(roller_, theme::pal().fg, 0);
    lv_obj_set_style_bg_color(roller_, theme::pal().accent, LV_PART_SELECTED);
    lv_obj_set_style_text_color(roller_, lv_color_hex(0x000000), LV_PART_SELECTED);
    lv_obj_align(roller_, LV_ALIGN_CENTER, 0, -10);
    if (m.getIndex) lv_roller_set_selected(roller_, static_cast<uint16_t>(m.getIndex()), LV_ANIM_OFF);
    modalButton(base, LV_SYMBOL_CLOSE, -40, 72, 64, [](lv_event_t* e) {
        static_cast<MenuScreen*>(lv_event_get_user_data(e))->closeModal();
    });
    modalButton(base, LV_SYMBOL_OK, 40, 72, 64, [](lv_event_t* e) {
        auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
        if (self->editIdx_ >= 0 && self->roller_) {
            MenuItem& it = self->items_[self->editIdx_];
            const int sel = lv_roller_get_selected(self->roller_);
            auto setter = it.setIndex;
            self->closeModal();
            if (setter) setter(sel);
        } else {
            self->closeModal();
        }
        self->refresh();
    });
}

void MenuScreen::updateEditLabel() {
    if (!editLabel_ || editIdx_ < 0) return;
    const MenuItem& m = items_[editIdx_];
    lv_label_set_text_fmt(editLabel_, "%.*f%s", m.decimals, double(editValue_), m.unit.c_str());
}

void MenuScreen::openNumber(int idx) {
    MenuItem& m = items_[idx];
    lv_obj_t* base = modalBase();
    editIdx_ = idx;
    editValue_ = m.getNum ? m.getNum() : m.minV;
    lv_obj_t* t = theme::label(base, theme::small(), theme::pal().accent, m.label.c_str());
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 26);
    editLabel_ = theme::label(base, theme::large(), theme::pal().fg, "");
    lv_obj_align(editLabel_, LV_ALIGN_CENTER, 0, -26);
    updateEditLabel();
    auto stepper = [](lv_event_t* e) {
        auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
        if (self->editIdx_ < 0) return;
        const MenuItem& it = self->items_[self->editIdx_];
        lv_obj_t* b = lv_event_get_current_target(e);
        const intptr_t dir = reinterpret_cast<intptr_t>(lv_obj_get_user_data(b));
        // long press repeat accelerates (x5)
        const float mult = lv_event_get_code(e) == LV_EVENT_LONG_PRESSED_REPEAT ? 5.0f : 1.0f;
        float v = self->editValue_ + float(dir) * it.step * mult;
        v = std::round(v / it.step) * it.step;
        if (v < it.minV) v = it.minV;
        if (v > it.maxV) v = it.maxV;
        self->editValue_ = v;
        self->updateEditLabel();
    };
    lv_obj_t* minus = modalButton(base, LV_SYMBOL_MINUS, -50, 18, 72, stepper);
    lv_obj_set_user_data(minus, reinterpret_cast<void*>(intptr_t(-1)));
    lv_obj_add_event_cb(minus, stepper, LV_EVENT_LONG_PRESSED_REPEAT, this);
    lv_obj_t* plus = modalButton(base, LV_SYMBOL_PLUS, 50, 18, 72, stepper);
    lv_obj_set_user_data(plus, reinterpret_cast<void*>(intptr_t(1)));
    lv_obj_add_event_cb(plus, stepper, LV_EVENT_LONG_PRESSED_REPEAT, this);
    modalButton(base, LV_SYMBOL_CLOSE, -40, 74, 64, [](lv_event_t* e) {
        static_cast<MenuScreen*>(lv_event_get_user_data(e))->closeModal();
    });
    modalButton(base, LV_SYMBOL_OK, 40, 74, 64, [](lv_event_t* e) {
        auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
        if (self->editIdx_ >= 0) {
            auto setter = self->items_[self->editIdx_].setNum;
            const float v = self->editValue_;
            self->closeModal();
            if (setter) setter(v);
        }
        self->refresh();
    });
}

void MenuScreen::openConfirm(int idx) {
    MenuItem& m = items_[idx];
    lv_obj_t* base = modalBase();
    editIdx_ = idx;
    lv_obj_t* t = theme::label(base, theme::medium(), theme::pal().warn, m.confirmText.c_str());
    lv_obj_set_width(t, 170);
    lv_label_set_long_mode(t, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t, LV_ALIGN_CENTER, 0, -30);
    modalButton(base, LV_SYMBOL_CLOSE, -40, 50, 64, [](lv_event_t* e) {
        static_cast<MenuScreen*>(lv_event_get_user_data(e))->closeModal();
    });
    modalButton(base, LV_SYMBOL_OK, 40, 50, 64, [](lv_event_t* e) {
        auto* self = static_cast<MenuScreen*>(lv_event_get_user_data(e));
        if (self->editIdx_ >= 0) {
            auto fn = self->items_[self->editIdx_].action;
            self->closeModal();
            if (fn) fn();
        }
        self->refresh();
    });
}
