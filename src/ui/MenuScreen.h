// SPDX-License-Identifier: MIT
// Generic, data driven menu for the round display: a scrollable list in the safe
// zone, plus modal editors (choice roller, numeric +/- editor, confirmation).
// Menus are built on demand and destroyed when left (saves LVGL memory).
#pragma once

#include <lvgl.h>

#include <functional>
#include <string>
#include <vector>

struct MenuItem {
    enum class Kind { Submenu, Action, Toggle, Choice, Number, Info, Confirm };
    Kind kind = Kind::Action;
    std::string label;
    std::function<std::string()> value;      ///< text right of the label (optional)
    std::function<void()> action;            ///< Submenu/Action/Confirm
    // Toggle
    std::function<bool()> getBool;
    std::function<void(bool)> setBool;
    // Choice
    std::string options;                     ///< "\n" separated
    std::function<int()> getIndex;
    std::function<void(int)> setIndex;
    // Number
    std::function<float()> getNum;
    std::function<void(float)> setNum;
    float minV = 0, maxV = 100, step = 1;
    int decimals = 0;
    std::string unit;
    std::string confirmText;

    static MenuItem submenu(const std::string& l, std::function<void()> open);
    static MenuItem act(const std::string& l, std::function<void()> fn, std::function<std::string()> v = nullptr);
    static MenuItem toggle(const std::string& l, std::function<bool()> g, std::function<void(bool)> s);
    static MenuItem choice(const std::string& l, const std::string& opts, std::function<int()> g,
                           std::function<void(int)> s);
    static MenuItem number(const std::string& l, float mn, float mx, float st, int dec, const std::string& unit,
                           std::function<float()> g, std::function<void(float)> s);
    static MenuItem info(const std::string& l, std::function<std::string()> v);
    static MenuItem confirm(const std::string& l, const std::string& question, std::function<void()> fn);
};

class MenuScreen {
public:
    using BackFn = std::function<void()>;
    /// Builds and loads a menu screen; `onBack` is called on "Zurück" / swipe right.
    void open(const std::string& title, std::vector<MenuItem> items, BackFn onBack);
    void close();
    bool isOpen() const { return scr_ != nullptr; }
    /// Re-evaluates value texts (call periodically, e.g. 2 Hz).
    void refresh();
    lv_obj_t* obj() const { return scr_; }

private:
    lv_obj_t* scr_ = nullptr;
    lv_obj_t* list_ = nullptr;
    lv_obj_t* modal_ = nullptr;
    std::vector<MenuItem> items_;
    std::vector<lv_obj_t*> valueLabels_;
    BackFn onBack_;
    // editor state
    int editIdx_ = -1;
    float editValue_ = 0;
    lv_obj_t* editLabel_ = nullptr;
    lv_obj_t* roller_ = nullptr;

    static void itemEvent(lv_event_t* e);
    static void backEvent(lv_event_t* e);
    static void gestureEvent(lv_event_t* e);
    void activate(int idx);
    void openChoice(int idx);
    void openNumber(int idx);
    void openConfirm(int idx);
    void closeModal();
    void updateEditLabel();
    lv_obj_t* modalBase();
    lv_obj_t* modalButton(lv_obj_t* parent, const char* text, int x, int y, int w, lv_event_cb_t cb);
};
