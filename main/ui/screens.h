#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_SRC_MAIN = 1,
    SCREEN_ID_SRC_TIME = 2,
    SCREEN_ID_SRC_SETTING = 3,
    SCREEN_ID_SRC_INFO = 4,
    _SCREEN_ID_LAST = 4
};

typedef struct _objects_t {
    lv_obj_t *src_main;
    lv_obj_t *src_time;
    lv_obj_t *src_setting;
    lv_obj_t *src_info;
    lv_obj_t *speed_after_adjust;
    lv_obj_t *speed_unit;
    lv_obj_t *gps_render_loading_indicator;
    lv_obj_t *speed_adjust_main;
    lv_obj_t *sat_num;
    lv_obj_t *signal_streng;
    lv_obj_t *hour_minute;
    lv_obj_t *date;
    lv_obj_t *second;
    lv_obj_t *rtc_sync_icon;
    lv_obj_t *spd_adj_setting;
    lv_obj_t *btn_inc;
    lv_obj_t *fix_info;
    lv_obj_t *hdop_info;
    lv_obj_t *sat_info;
    lv_obj_t *lat_info;
    lv_obj_t *long_info;
} objects_t;

extern objects_t objects;

void create_screen_src_main();
void tick_screen_src_main();

void create_screen_src_time();
void tick_screen_src_time();

void create_screen_src_setting();
void tick_screen_src_setting();

void create_screen_src_info();
void tick_screen_src_info();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/