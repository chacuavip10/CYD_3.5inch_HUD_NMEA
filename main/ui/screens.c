#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;

//
// Event handlers
//

lv_obj_t *tick_value_change_obj;

//
// Screens
//

void create_screen_gpslost() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.gpslost = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_spinner_create(parent_obj);
            lv_obj_set_pos(obj, 173, 30);
            lv_obj_set_size(obj, 135, 130);
            lv_obj_set_style_arc_width(obj, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // gps_lost_text
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_lost_text = obj;
            lv_obj_set_pos(obj, 60, 220);
            lv_obj_set_size(obj, 360, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "Finding satelites...");
        }
    }
    
    tick_screen_gpslost();
}

void tick_screen_gpslost() {
}

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // sats_num
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.sats_num = obj;
            lv_obj_set_pos(obj, 421, 272);
            lv_obj_set_size(obj, 43, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "30");
        }
        {
            // singal_bar
            lv_obj_t *obj = lv_image_create(parent_obj);
            objects.singal_bar = obj;
            lv_obj_set_pos(obj, 416, 11);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_image_set_src(obj, &img_signal_ex_48px);
        }
        {
            // sat_img
            lv_obj_t *obj = lv_image_create(parent_obj);
            objects.sat_img = obj;
            lv_obj_set_pos(obj, 379, 268);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_image_set_src(obj, &img_satlite_white_40px);
        }
        {
            // speed_compensate
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.speed_compensate = obj;
            lv_obj_set_pos(obj, 12, 272);
            lv_obj_set_size(obj, 36, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "+5");
        }
        {
            // nmea_indicator
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.nmea_indicator = obj;
            lv_obj_set_pos(obj, 11, 11);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "\\");
        }
        {
            // render_indicator
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.render_indicator = obj;
            lv_obj_set_pos(obj, 39, 11);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "/");
        }
        {
            // speed
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.speed = obj;
            lv_obj_set_pos(obj, 123, 92);
            lv_obj_set_size(obj, 234, 137);
            lv_obj_set_style_text_font(obj, &ui_font_jb_130, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffbe00), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "999");
        }
        {
            // speed_unit
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.speed_unit = obj;
            lv_obj_set_pos(obj, 363, 178);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "km/h");
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
}

void create_screen_time() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.time = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // hour_minute
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.hour_minute = obj;
            lv_obj_set_pos(obj, 18, 85);
            lv_obj_set_size(obj, 390, 137);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_130, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffbe00), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "19:59");
        }
        {
            // second
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.second = obj;
            lv_obj_set_pos(obj, 425, 85);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "59");
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 150, 236);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "14/04/2026");
        }
        {
            // rtc_sync_icon
            lv_obj_t *obj = lv_image_create(parent_obj);
            objects.rtc_sync_icon = obj;
            lv_obj_set_pos(obj, 419, 9);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_image_set_src(obj, &img_sync_48px);
        }
    }
    
    tick_screen_time();
}

void tick_screen_time() {
}

void create_screen_debug() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.debug = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // gps_fix
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_fix = obj;
            lv_obj_set_pos(obj, 56, 35);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "FIX :");
        }
        {
            // gps_sat
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_sat = obj;
            lv_obj_set_pos(obj, 56, 87);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "SAT :");
        }
        {
            // gps_hdop
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_hdop = obj;
            lv_obj_set_pos(obj, 56, 144);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "HDOP:");
        }
        {
            // gps_lat
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_lat = obj;
            lv_obj_set_pos(obj, 56, 195);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "LAT :");
        }
        {
            // gps_long
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_long = obj;
            lv_obj_set_pos(obj, 56, 250);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "LONG:");
        }
    }
    
    tick_screen_debug();
}

void tick_screen_debug() {
}

void create_screen_setting() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.setting = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // spd_adj_setting
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.spd_adj_setting = obj;
            lv_obj_set_pos(obj, 201, 92);
            lv_obj_set_size(obj, 78, 137);
            lv_obj_set_style_text_font(obj, &ui_font_jb_130, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "0");
        }
        {
            // up
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.up = obj;
            lv_obj_set_pos(obj, 35, 129);
            lv_obj_set_size(obj, 133, 63);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "+");
                }
            }
        }
        {
            lv_obj_t *obj = lv_button_create(parent_obj);
            lv_obj_set_pos(obj, 313, 129);
            lv_obj_set_size(obj, 133, 63);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "-");
                }
            }
        }
        {
            // spd_setting_label
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.spd_setting_label = obj;
            lv_obj_set_pos(obj, 132, 33);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb_30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "Adjust speed");
        }
    }
    
    tick_screen_setting();
}

void tick_screen_setting() {
}

typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_gpslost,
    tick_screen_main,
    tick_screen_time,
    tick_screen_debug,
    tick_screen_setting,
};
void tick_screen(int screen_index) {
    if (screen_index >= 0 && screen_index < 5) {
        tick_screen_funcs[screen_index]();
    }
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen(screenId - 1);
}

//
// Fonts
//

ext_font_desc_t fonts[] = {
    { "jb_130", &ui_font_jb_130 },
    { "jb_30", &ui_font_jb_30 },
#if LV_FONT_MONTSERRAT_8
    { "MONTSERRAT_8", &lv_font_montserrat_8 },
#endif
#if LV_FONT_MONTSERRAT_10
    { "MONTSERRAT_10", &lv_font_montserrat_10 },
#endif
#if LV_FONT_MONTSERRAT_12
    { "MONTSERRAT_12", &lv_font_montserrat_12 },
#endif
#if LV_FONT_MONTSERRAT_14
    { "MONTSERRAT_14", &lv_font_montserrat_14 },
#endif
#if LV_FONT_MONTSERRAT_16
    { "MONTSERRAT_16", &lv_font_montserrat_16 },
#endif
#if LV_FONT_MONTSERRAT_18
    { "MONTSERRAT_18", &lv_font_montserrat_18 },
#endif
#if LV_FONT_MONTSERRAT_20
    { "MONTSERRAT_20", &lv_font_montserrat_20 },
#endif
#if LV_FONT_MONTSERRAT_22
    { "MONTSERRAT_22", &lv_font_montserrat_22 },
#endif
#if LV_FONT_MONTSERRAT_24
    { "MONTSERRAT_24", &lv_font_montserrat_24 },
#endif
#if LV_FONT_MONTSERRAT_26
    { "MONTSERRAT_26", &lv_font_montserrat_26 },
#endif
#if LV_FONT_MONTSERRAT_28
    { "MONTSERRAT_28", &lv_font_montserrat_28 },
#endif
#if LV_FONT_MONTSERRAT_30
    { "MONTSERRAT_30", &lv_font_montserrat_30 },
#endif
#if LV_FONT_MONTSERRAT_32
    { "MONTSERRAT_32", &lv_font_montserrat_32 },
#endif
#if LV_FONT_MONTSERRAT_34
    { "MONTSERRAT_34", &lv_font_montserrat_34 },
#endif
#if LV_FONT_MONTSERRAT_36
    { "MONTSERRAT_36", &lv_font_montserrat_36 },
#endif
#if LV_FONT_MONTSERRAT_38
    { "MONTSERRAT_38", &lv_font_montserrat_38 },
#endif
#if LV_FONT_MONTSERRAT_40
    { "MONTSERRAT_40", &lv_font_montserrat_40 },
#endif
#if LV_FONT_MONTSERRAT_42
    { "MONTSERRAT_42", &lv_font_montserrat_42 },
#endif
#if LV_FONT_MONTSERRAT_44
    { "MONTSERRAT_44", &lv_font_montserrat_44 },
#endif
#if LV_FONT_MONTSERRAT_46
    { "MONTSERRAT_46", &lv_font_montserrat_46 },
#endif
#if LV_FONT_MONTSERRAT_48
    { "MONTSERRAT_48", &lv_font_montserrat_48 },
#endif
};

//
// Color themes
//

uint32_t active_theme_index = 0;

//
//
//

void create_screens() {

// Set default LVGL theme
    lv_display_t *dispp = lv_display_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_display_set_theme(dispp, theme);
    
    // Initialize screens
    // Create screens
    create_screen_gpslost();
    create_screen_main();
    create_screen_time();
    create_screen_debug();
    create_screen_setting();
}