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

void create_screen_src_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.src_main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            // main_prev_scr
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.main_prev_scr = obj;
            lv_obj_set_pos(obj, 1, 0);
            lv_obj_set_size(obj, 100, 226);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // main_next_scr
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.main_next_scr = obj;
            lv_obj_set_pos(obj, 379, 0);
            lv_obj_set_size(obj, 100, 226);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
        }
        {
            // speed_after_adjust
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.speed_after_adjust = obj;
            lv_obj_set_pos(obj, 84, 111);
            lv_obj_set_size(obj, 273, 115);
            lv_obj_set_style_text_font(obj, &ui_font_jb150, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffbe00), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "999");
        }
        {
            // speed_unit
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.speed_unit = obj;
            lv_obj_set_pos(obj, 357, 177);
            lv_obj_set_size(obj, 72, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "km/h");
        }
        {
            // gps_render_loading_indicator
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_render_loading_indicator = obj;
            lv_obj_set_pos(obj, 30, 19);
            lv_obj_set_size(obj, 138, 32);
            lv_obj_set_style_text_font(obj, &ui_font_braille30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "⠏");
        }
        {
            // sat_num
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.sat_num = obj;
            lv_obj_set_pos(obj, 275, 271);
            lv_obj_set_size(obj, 190, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "SAT:0");
        }
        {
            // signal_bar_icon
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.signal_bar_icon = obj;
            lv_obj_set_pos(obj, 370, 13);
            lv_obj_set_size(obj, 83, 46);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_text_font(obj, &ui_font_font_awesome_icon_40, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "X");
        }
        {
            // signal_streng
            lv_obj_t *obj = lv_image_create(parent_obj);
            objects.signal_streng = obj;
            lv_obj_set_pos(obj, 405, 11);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_image_set_src(obj, &img_no_signal_48px);
        }
        {
            // btn_inc
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.btn_inc = obj;
            lv_obj_set_pos(obj, 1, 236);
            lv_obj_set_size(obj, 167, 84);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // speed_adjust_main
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.speed_adjust_main = obj;
                    lv_obj_set_pos(obj, -36, 8);
                    lv_obj_set_size(obj, 36, 34);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "+0");
                }
            }
        }
    }
    
    tick_screen_src_main();
}

void tick_screen_src_main() {
}

void create_screen_src_time() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.src_time = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            // time_prev_scr
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.time_prev_scr = obj;
            lv_obj_set_pos(obj, 1, 0);
            lv_obj_set_size(obj, 100, 226);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
        }
        {
            // time_next_scr
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.time_next_scr = obj;
            lv_obj_set_pos(obj, 379, 0);
            lv_obj_set_size(obj, 100, 226);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
        }
        {
            // hour_minute
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.hour_minute = obj;
            lv_obj_set_pos(obj, 15, 55);
            lv_obj_set_size(obj, 450, 133);
            lv_obj_set_style_text_font(obj, &ui_font_jb150, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffbe00), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "20:59");
        }
        {
            // date
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.date = obj;
            lv_obj_set_pos(obj, 51, 247);
            lv_obj_set_size(obj, 366, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "Chủ nhật, 15/04/2026");
        }
        {
            // second
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.second = obj;
            lv_obj_set_pos(obj, 380, 188);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb61, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xa0a0a0), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "59");
        }
        {
            // icon_sync_rtc
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.icon_sync_rtc = obj;
            lv_obj_set_pos(obj, 426, 263);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_text_font(obj, &ui_font_font_awesome_icon_40, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "X");
        }
    }
    
    tick_screen_src_time();
}

void tick_screen_src_time() {
}

void create_screen_src_info() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.src_info = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            // info_prev_scr
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.info_prev_scr = obj;
            lv_obj_set_pos(obj, 1, 0);
            lv_obj_set_size(obj, 100, 218);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
        }
        {
            // info_next_scr
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.info_next_scr = obj;
            lv_obj_set_pos(obj, 379, 0);
            lv_obj_set_size(obj, 100, 218);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_PRESSED);
        }
        {
            // fix_info
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.fix_info = obj;
            lv_obj_set_pos(obj, 65, 22);
            lv_obj_set_size(obj, 344, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "FIX :");
        }
        {
            // hdop_info
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.hdop_info = obj;
            lv_obj_set_pos(obj, 65, 83);
            lv_obj_set_size(obj, 344, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "HDOP:");
        }
        {
            // sat_info
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.sat_info = obj;
            lv_obj_set_pos(obj, 65, 149);
            lv_obj_set_size(obj, 344, 32);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "SAT :");
        }
        {
            // lat_info
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.lat_info = obj;
            lv_obj_set_pos(obj, 65, 207);
            lv_obj_set_size(obj, 344, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "LAT :");
        }
        {
            // long_info
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.long_info = obj;
            lv_obj_set_pos(obj, 65, 264);
            lv_obj_set_size(obj, 344, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &ui_font_jb30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "LONG:");
        }
        {
            // gps_render_loading_indicator_1
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.gps_render_loading_indicator_1 = obj;
            lv_obj_set_pos(obj, 311, 23);
            lv_obj_set_size(obj, 138, 32);
            lv_obj_set_style_text_font(obj, &ui_font_braille30, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "⠏");
        }
    }
    
    tick_screen_src_info();
}

void tick_screen_src_info() {
}

typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_src_main,
    tick_screen_src_time,
    tick_screen_src_info,
};
void tick_screen(int screen_index) {
    if (screen_index >= 0 && screen_index < 3) {
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
    { "jb30", &ui_font_jb30 },
    { "jb150", &ui_font_jb150 },
    { "Braille30", &ui_font_braille30 },
    { "jb61", &ui_font_jb61 },
    { "font_awesome_icon_40", &ui_font_font_awesome_icon_40 },
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
    create_screen_src_main();
    create_screen_src_time();
    create_screen_src_info();
}