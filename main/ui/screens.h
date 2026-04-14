#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_GPSLOST = 1,
    SCREEN_ID_MAIN = 2,
    SCREEN_ID_TIME = 3,
    SCREEN_ID_DEBUG = 4,
    SCREEN_ID_SETTING = 5,
    _SCREEN_ID_LAST = 5
};

typedef struct _objects_t {
    lv_obj_t *gpslost;
    lv_obj_t *main;
    lv_obj_t *time;
    lv_obj_t *debug;
    lv_obj_t *setting;
    lv_obj_t *gps_lost_text;
    lv_obj_t *sats_num;
    lv_obj_t *singal_bar;
    lv_obj_t *sat_img;
    lv_obj_t *speed_compensate;
    lv_obj_t *nmea_indicator;
    lv_obj_t *render_indicator;
    lv_obj_t *speed;
    lv_obj_t *speed_unit;
    lv_obj_t *hour_minute;
    lv_obj_t *second;
    lv_obj_t *rtc_sync_icon;
    lv_obj_t *gps_fix;
    lv_obj_t *gps_sat;
    lv_obj_t *gps_hdop;
    lv_obj_t *gps_lat;
    lv_obj_t *gps_long;
    lv_obj_t *spd_adj_setting;
    lv_obj_t *up;
    lv_obj_t *spd_setting_label;
} objects_t;

extern objects_t objects;

void create_screen_gpslost();
void tick_screen_gpslost();

void create_screen_main();
void tick_screen_main();

void create_screen_time();
void tick_screen_time();

void create_screen_debug();
void tick_screen_debug();

void create_screen_setting();
void tick_screen_setting();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/