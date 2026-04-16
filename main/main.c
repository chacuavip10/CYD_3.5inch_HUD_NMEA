/**
 * ============================================================================
 *  GPS HUD – ESP32 + ATGM336H
 *  ESP-IDF 5.x / 6.x
 *
 *  DATA FLOW:
 *      UART → NMEA FSM → double buffer → notify →
 *          ├── RTC task (sync time)
 *          ├── UI task  (update screen)
 *          └── DEBUG (optional)
 *
 *  DESIGN GOALS:
 *      - Lock-free GPS data sharing
 *      - LVGL thread-safe via mutex
 *      - No blocking in ISR
 *      - Event-driven UI + smooth rendering
 * ============================================================================
 */

/* ========================================================================== */
/*                              INCLUDES                                       */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "gps_nmea.h"
#include "gps_rtc.h"
#include "lvgl.h"
#include "ui/ui.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "images.h"

/* ========================================================================== */
/*                              CONFIGURATION                                  */
/* ========================================================================== */

#define DEBUG_TASK 0
static const char *TAG = "CYD_3.5inch_GPS_HUD";

// GPIO & UART
#define GPS_UART_PORT UART_NUM_2
#define GPS_RX_GPIO GPIO_NUM_21
#define LED_RED GPIO_NUM_4
#define LED_GREEN GPIO_NUM_17
#define LED_BLUE GPIO_NUM_16
#define PIN_NUM_BK_LIGHT GPIO_NUM_27
#define GPS_BAUD_RATE 115200
#define UART_RX_RING_BUF 2048
#define UART_READ_BUF_SZ 256
#define UART_READ_TIMEOUT_MS 10
#define GPS_COM_TIMEOUT_MS 2000

// Event flags
#define EVT_GPS_UPDATE (1 << 0)
#define EVT_RTC_SYNC (1 << 1)

// Task handles
static TaskHandle_t gps_task_handle;
static TaskHandle_t rtc_task_handle;
static TaskHandle_t ui_lvgl_task_handle;
#if DEBUG_TASK
static TaskHandle_t dbg_task_handle;
#endif

/* ========================================================================== */
/*                              LVGL CONFIG                                    */
/* ========================================================================== */

static SemaphoreHandle_t s_lvgl_mutex;
#define LVGL_LOCK() xSemaphoreTake(s_lvgl_mutex, portMAX_DELAY)
#define LVGL_UNLOCK() xSemaphoreGive(s_lvgl_mutex)

// LCD SPI
#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)
#define PIN_NUM_SCLK GPIO_NUM_14
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_LCD_DC GPIO_NUM_2
#define PIN_NUM_LCD_RST GPIO_NUM_NC
#define PIN_NUM_LCD_CS GPIO_NUM_15
#define PIN_NUM_TOUCH_CS GPIO_NUM_33
#define PIN_NUM_TOUCH_INT GPIO_NUM_36

// Display
#define LCD_HOR_RES 320
#define LCD_VER_RES 480
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

// LVGL settings
#define LVGL_DRAW_BUF_LINES 60
#define LVGL_TICK_PERIOD_MS 1
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 5
#define LVGL_TASK_STACK_SIZE (8 * 1024)
#define LVGL_TASK_PRIORITY 5

/* ========================================================================== */
/*                              GPS SHARED BUFFER                              */
/* ========================================================================== */

typedef struct
{
    gps_data_t buf[2];
    volatile uint8_t index;
} gps_shared_t;

static gps_shared_t g_gps = {0};

static inline void gps_read_latest(gps_data_t *out)
{
    uint8_t idx1, idx2;
    do
    {
        idx1 = __atomic_load_n(&g_gps.index, __ATOMIC_ACQUIRE);
        *out = g_gps.buf[idx1];
        idx2 = __atomic_load_n(&g_gps.index, __ATOMIC_ACQUIRE);
    } while (idx1 != idx2);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  HDOP Helper                                                                */
/* ─────────────────────────────────────────────────────────────────────────── */
typedef enum
{
    SIG_NOSIGNAL = 0,
    SIG_BAD,
    SIG_MODERATE,
    SIG_GOOD,
    SIG_EXCELLENT
} signal_level_t;

static inline signal_level_t hdop_to_level(float hdop)
{
    if (hdop > 5.0f)
        return SIG_BAD;
    if (hdop > 2.5f)
        return SIG_MODERATE;
    if (hdop > 1.5f)
        return SIG_GOOD;
    return SIG_EXCELLENT;
}

static const void *signal_img_table[] = {
    &img_no_signal_48px,       // SIG_NOSIGNAL
    &img_signal_poor_48px,     // SIG_BAD
    &img_signal_moderate_48px, // SIG_MODERATE
    &img_signal_good_48px,     // SIG_GOOD
    &img_signal_ex_48px        // SIG_EXCELLENT
};

bool time_equal(local_time_t *a, local_time_t *b)
{
    return (a->year == b->year) &&
           (a->month == b->month) &&
           (a->day == b->day) &&
           (a->hour == b->hour) &&
           (a->minute == b->minute) &&
           (a->second == b->second) &&
           (a->valid == b->valid);
}

static void format_lat(double lat, char *buf, size_t len)
{
    char hemi = (lat >= 0) ? 'N' : 'S';
    lat = fabs(lat);

    int deg = (int)lat;
    double min = (lat - deg) * 60.0;

    snprintf(buf, len, "LAT : %02d°%06.3f'%c", deg, min, hemi);
}

static void format_lon(double lon, char *buf, size_t len)
{
    char hemi = (lon >= 0) ? 'E' : 'W';
    lon = fabs(lon);

    int deg = (int)lon;
    double min = (lon - deg) * 60.0;

    snprintf(buf, len, "LONG: %03d°%06.3f'%c", deg, min, hemi);
}

#if DEBUG_TASK
/* ─────────────────────────────────────────────────────────────────────────── */
/*  Hiển thị dữ liệu GPS – thời gian lấy từ RTC local                         */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief In dữ liệu GPS ra console.
 *
 * Thời gian/ngày tháng: đọc từ RTC local (GMT+7), KHÔNG từ gps_data_t.
 * Vị trí, tốc độ, chất lượng: lấy trực tiếp từ gps_data_t.
 *
 * @param gps  Dữ liệu GPS mới nhất.
 */
static void print_gps_data(const gps_data_t *gps)
{
    /* Đọc thời gian local GMT+7 từ RTC system */
    local_time_t lt;
    gps_rtc_get_local_time(&lt);

    ESP_LOGI(TAG, "--------------------------------------");

    /* ── Thời gian từ RTC local (GMT+7) ─────────────────────────────── */
    if (lt.valid)
    {
        ESP_LOGI("DEBUG", "Time (GMT+7): %02d:%02d:%02d",
                 lt.hour, lt.minute, lt.second);
        ESP_LOGI("DEBUG", "Date (GMT+7): %02d/%02d/%04d",
                 lt.day, lt.month, lt.year);
    }
    else
    {
        /*
         * RTC chưa được sync lần nào (chờ GPS fix lần đầu).
         * Hiển thị UTC thô từ GPS làm fallback tạm thời.
         */
        ESP_LOGI("DEBUG", "Time (UTC) : %02d:%02d:%02d  [waiting for RTC sync]",
                 gps->hour, gps->minute, gps->second);
        ESP_LOGI("DEBUG", "Date (UTC) : %02d/%02d/%04d",
                 gps->day, gps->month, gps->year);
    }
    if (gps->valid)
    {
        /* ── Trạng thái fix ──────────────────────────────────────────────── */
        ESP_LOGI("DEBUG", "Fix valid  : YES");

        /* ── Vị trí ──────────────────────────────────────────────────────── */
        ESP_LOGI("DEBUG", "Latitude   : %.7f %s",
                 gps->latitude >= 0 ? gps->latitude : -gps->latitude,
                 gps->latitude >= 0 ? "N" : "S");
        ESP_LOGI("DEBUG", "Longitude  : %.7f %s",
                 gps->longitude >= 0 ? gps->longitude : -gps->longitude,
                 gps->longitude >= 0 ? "E" : "W");

        /* ── Chất lượng tín hiệu ─────────────────────────────────────────── */
        ESP_LOGI("DEBUG", "Satellites : %d", gps->satellites);
        ESP_LOGI("DEBUG", "HDOP       : %.2f", (double)gps->hdop);
        ESP_LOGI("DEBUG", "Altitude   : %.1f m", (double)gps->altitude_m);

        /* ── Chuyển động ─────────────────────────────────────────────────── */
        ESP_LOGI("DEBUG", "Speed      : %.2f km/h", (double)gps->speed_kmh);
        ESP_LOGI("DEBUG", "Course     : %.1f deg", (double)gps->course_deg);
    }
    else
    {
        ESP_LOGI("DEBUG", "Fix valid  : NO");
    }
}
#endif

/* ========================================================================== */
/*                              LVGL CALLBACKS                                 */
/* ========================================================================== */

static uint8_t speed_compensation = 0;
static int current_screen = SCREEN_ID_SRC_MAIN;

static lv_timer_t *img_timer = NULL;
static void hide_image_cb(lv_timer_t *t)
{
    lv_obj_t *img = (lv_obj_t *)lv_timer_get_user_data(t);
    if (!lv_obj_has_flag(img, LV_OBJ_FLAG_HIDDEN))
        lv_obj_add_flag(img, LV_OBJ_FLAG_HIDDEN); // ẩn image

    lv_timer_pause(t);
}

void ui_show_image_2s(lv_obj_t *img)
{
    if (lv_obj_has_flag(img, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(img, LV_OBJ_FLAG_HIDDEN);

    if (img_timer)
    {
        lv_timer_set_user_data(img_timer, img);
        lv_timer_reset(img_timer);
        lv_timer_resume(img_timer);
    }
    else
    {
        img_timer = lv_timer_create(hide_image_cb, 2000, img);
    }
}

static void btn_inc_cb(lv_event_t *e)
{
    if (speed_compensation > 4)
    {
        speed_compensation = 0;
    }
    else
    {
        speed_compensation++;
    }
    ESP_LOGI(TAG, "Button +");
    lv_label_set_text_fmt(objects.speed_adjust_main, "+%d", speed_compensation);
    lv_label_set_text_fmt(objects.speed_adjust_main, "+%d", speed_compensation);
}

static void btn_next_screen_cb(lv_event_t *e)
{
    current_screen++;

    if (current_screen > _SCREEN_ID_LAST)
    {
        current_screen = _SCREEN_ID_FIRST;
    }

    loadScreen(current_screen);
}

static void btn_prev_screen_cb(lv_event_t *e)
{
    current_screen--;

    if (current_screen < _SCREEN_ID_FIRST)
    {
        current_screen = _SCREEN_ID_LAST;
    }

    loadScreen(current_screen);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation)
    {
    case LV_DISPLAY_ROTATION_0:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int x1 = area->x1, x2 = area->x2;
    int y1 = area->y1, y2 = area->y2;
    lv_draw_sw_rgb565_swap(px_map, (x2 + 1 - x1) * (y2 + 1 - y1));
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, px_map);
}

static void increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
    esp_lcd_touch_read_data(touch_pad);
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(
        touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        // ESP_LOGI("TOUCH", "x=%d y=%d state=%d", data->point.x, data->point.y, data->state);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* ========================================================================== */
/*                              TASKS                                          */
/* ========================================================================== */

// GPS Task
static void gps_task(void *arg)
{
    nmea_parser_t parser;
    nmea_parser_init(&parser);
    uint32_t last_seq = 0;
    static uint8_t rx_buf[UART_READ_BUF_SZ];

    ESP_LOGI(TAG, "GPS task started");

    while (1)
    {
        int rx_len = uart_read_bytes(GPS_UART_PORT, rx_buf, sizeof(rx_buf),
                                     pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));

        if (rx_len > 0)
        {
            nmea_parser_feed(&parser, rx_buf, (size_t)rx_len);

            if (parser.data.seq != last_seq)
            {
                uint8_t next = g_gps.index ^ 1;
                g_gps.buf[next] = parser.data;
                __atomic_store_n(&g_gps.index, next, __ATOMIC_RELEASE);

                xTaskNotify(rtc_task_handle, EVT_GPS_UPDATE, eSetBits);
                xTaskNotify(ui_lvgl_task_handle, EVT_GPS_UPDATE, eSetBits);
#if DEBUG_TASK
                ESP_LOGI("DEBUG", "Event Sent from [GPS_TASK]");
                xTaskNotify(dbg_task_handle, EVT_GPS_UPDATE, eSetBits);
#endif
                last_seq = parser.data.seq;
            }
        }
        else
        {
            taskYIELD();
        }
    }
}

// RTC Sync Task
static void rtc_sync_task(void *arg)
{
    ESP_LOGI(TAG, "RTC sync task started");
    gps_data_t d;
    uint32_t last_seq = 0;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gps_read_latest(&d);
        if (d.seq == last_seq)
            continue;
        last_seq = d.seq;

        if (gps_rtc_should_sync(&d))
        {
            gps_rtc_sync(&d);
            xTaskNotify(ui_lvgl_task_handle, EVT_RTC_SYNC, eSetBits);
#if DEBUG_TASK
            ESP_LOGI("DEBUG", "Event Sent from [RTC_SYNC_TASK]");
            xTaskNotify(dbg_task_handle, EVT_RTC_SYNC, eSetBits);
#endif
        }
    }
}

/* ========================================================================== */
/*                              UI + LVGL TASK (GỘP)                           */
/* ========================================================================== */

/**
 * @brief UI + LVGL combined task
 *
 * Task này đảm nhiệm cả hai vai trò:
 *   1. Gọi lv_timer_handler() để xử lý LVGL timer và render
 *   2. Nhận notification từ GPS task để cập nhật UI
 *
 * Vì chỉ có MỘT task duy nhất gọi LVGL API:
 *   - KHÔNG cần mutex (LVGL_LOCK/UNLOCK)
 *   - An toàn với lv_tick_inc() từ ISR
 *   - Đơn giản hóa kiến trúc
 */
static void ui_lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "UI+LVGL combined task started");

    uint32_t events;
    uint32_t time_till_next_ms;
    gps_data_t d;
    gps_data_t last_data = {};
    uint8_t spinGps = 0;
    signal_level_t last_signal = 0xFF; // invalid initial
    uint32_t last_gps_tick = 0;
    bool gps_timeout = false; // start assume no signal
    local_time_t current_time = {};
    local_time_t last_time = {};
    last_time.valid = true;
    char lat_buf[32];
    char lon_buf[32];
    static const char *frames[] = {
        "⠋",
        "⠙",
        "⠹",
        "⠸",
        "⠼",
        "⠴",
        "⠦",
        "⠧",
        "⠇",
        "⠏",
    };
    static const int frames_len = sizeof(frames) / sizeof(frames[0]);

    // add callback cho button
    lv_obj_add_event_cb(objects.btn_inc, btn_inc_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(objects.main_next_scr, btn_next_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.main_prev_scr, btn_prev_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.time_next_scr, btn_next_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.time_prev_scr, btn_prev_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.info_next_scr, btn_next_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.info_prev_scr, btn_prev_screen_cb, LV_EVENT_CLICKED, NULL);
    ESP_LOGI(TAG, "Add button callback");

    while (1)
    {
        // ===== PHẦN 1: XỬ LÝ LVGL (CÓ MUTEX) =====
        LVGL_LOCK();
        time_till_next_ms = lv_timer_handler();

        uint32_t now = lv_tick_get();

        // last_speed_compensation = speed_compensation;
        if (!gps_timeout && (now - last_gps_tick > GPS_COM_TIMEOUT_MS))
        {
            gps_timeout = true;
            ESP_LOGW(TAG, "GPS timeout >%ds", GPS_COM_TIMEOUT_MS / 1000);

            // ===== UI REACT =====
            lv_label_set_text(objects.sat_num, "NO GPS");

            // reset signal icon
            last_signal = SIG_NOSIGNAL;
            lv_image_set_src(objects.signal_streng, signal_img_table[SIG_NOSIGNAL]);

            // reset speed
            lv_label_set_text(objects.speed_after_adjust, "");

            // hide speed unit label
            if (!lv_obj_has_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN))
            {
                lv_obj_add_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN);
            }

            // info scr
            lv_label_set_text(objects.fix_info, "NO GPS");
            lv_label_set_text(objects.sat_info, "SAT : ");
            lv_label_set_text(objects.hdop_info, "HDOP: ");
            lv_label_set_text(objects.lat_info, "LAT : ");
            lv_label_set_text(objects.long_info, "LONG: ");
        }

        // Time screen
        gps_rtc_get_local_time(&current_time);
        if (current_time.valid != last_time.valid)
        {
            if (!current_time.valid)
            {
                lv_label_set_text(objects.hour_minute, "--:--");
                lv_label_set_text(objects.second, "--");
                lv_label_set_text(objects.date, "Waiting GPS");
            }
        }

        if ((current_time.valid) && (!time_equal(&current_time, &last_time)))
        {
            // ESP_LOGI(TAG, "Time advanced!");
            lv_label_set_text_fmt(objects.hour_minute, "%02d:%02d", current_time.hour, current_time.minute);
            lv_label_set_text_fmt(objects.second, "%02d", current_time.second);
            lv_label_set_text_fmt(objects.date, "%02d/%02d/%04d", current_time.day, current_time.month, current_time.year);
        }

        last_time = current_time;
        // ===== PHẦN 2: XỬ LÝ SỰ KIỆN UI TRONG KHI VẪN GIỮ MUTEX =====
        // Kiểm tra notification từ GPS task với timeout = 0 (không chờ)
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &events, 0) == pdTRUE)
        {
            if (events & EVT_GPS_UPDATE)
            {
                last_gps_tick = lv_tick_get();

                if (gps_timeout)
                {
                    gps_timeout = false;
                    ESP_LOGI(TAG, "GPS signal restored");
                    // show speed unit label
                    if (lv_obj_has_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN))
                    {
                        lv_obj_clear_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN);
                    }
                    // show sat num label
                    if (lv_obj_has_flag(objects.sat_num, LV_OBJ_FLAG_HIDDEN))
                    {
                        lv_obj_clear_flag(objects.sat_num, LV_OBJ_FLAG_HIDDEN);
                    }
                }
                gps_read_latest(&d);
                if (d.seq != last_data.seq)
                {
                    // Case 1: Alway render
                    bool valid_changed = d.valid != last_data.valid;
                    if (valid_changed)
                    {
                        if (!d.valid)
                        {
                            // Signal bar
                            lv_image_set_src(objects.signal_streng, signal_img_table[SIG_NOSIGNAL]);
                            lv_label_set_text(objects.fix_info, "FIX : NO");
                            lv_label_set_text(objects.hdop_info, "HDOP: ");
                            lv_label_set_text_fmt(objects.sat_info, "SAT : %d", d.satellites);
                            lv_label_set_text(objects.lat_info, "LAT :");
                            lv_label_set_text(objects.long_info, "LONG:");
                        }
                        else
                        {
                            lv_label_set_text(objects.fix_info, "FIX : YES");
                        }
                    }
                    signal_level_t new_signal = hdop_to_level(d.hdop);

                    if (new_signal != last_signal)
                    {
                        last_signal = new_signal;
                        lv_image_set_src(objects.signal_streng, signal_img_table[new_signal]);
                    }

                    int speed_kmh = (int)(d.speed_kmh + 0.5f);

                    if (speed_kmh < 1)
                    {
                        speed_kmh = 0;
                    }
                    else
                    {
                        speed_kmh += speed_compensation;
                    }

                    if (!d.valid)
                    {
                        lv_label_set_text(objects.speed_after_adjust, "");
                        lv_label_set_text(objects.sat_num, "NOT FIXED!");
                        lv_label_set_text(objects.fix_info, "FIX : NO");
                    }
                    else
                    {
                        lv_label_set_text_fmt(objects.speed_after_adjust, "%d", speed_kmh);
                        lv_label_set_text_fmt(objects.sat_num, "SAT: %d", d.satellites);
                        lv_label_set_text_fmt(objects.hdop_info, "HDOP: %.1f", d.hdop);
                        lv_label_set_text_fmt(objects.sat_info, "SAT : %d", d.satellites);
                        format_lat(d.latitude, lat_buf, sizeof(lat_buf));
                        format_lon(d.longitude, lon_buf, sizeof(lon_buf));
                        lv_label_set_text(objects.lat_info, lat_buf);
                        lv_label_set_text(objects.long_info, lon_buf);
                    }
                    // GPS+Renderspin
                    spinGps = (spinGps + 1) % frames_len;
                    lv_label_set_text(objects.gps_render_loading_indicator, frames[spinGps]);
                    // ESP_LOGI(TAG, "GPS spin update: %c", "/-\\|"[spinGps]);

                    // Last action
                    last_data = d;
                }
            }

            if (events & EVT_RTC_SYNC)
            {
                // TODO: Hiển thị icon RTC đã đồng bộ
                ui_show_image_2s(objects.rtc_sync_icon);
                ESP_LOGI(TAG, "RTC event!");
            }
        }
        LVGL_UNLOCK();

        // ===== PHẦN 3: DELAY THÔNG MINH =====
        // LVGL trả về thời gian (ms) cần đợi đến timer tiếp theo
        // Clamp giá trị để đảm bảo:
        //   - Tối thiểu 5ms (tránh busy-wait, cho task khác chạy)
        //   - Tối đa 500ms (đảm bảo UI responsive với GPS update)
        // in case of triggering a task watch dog time out
        time_till_next_ms = LV_MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = LV_MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}

#if DEBUG_TASK
static void debug_task(void *arg)
{
    ESP_LOGI(TAG, "Start debug task");
    uint32_t events;
    gps_data_t d;
    uint32_t last_seq = 0;

    while (1)
    {
        xTaskNotifyWait(0, 0xFFFFFFFF, &events, portMAX_DELAY);

        if (events & EVT_GPS_UPDATE)
        {
            gps_read_latest(&d);
            if (d.seq == last_seq)
                continue;
            last_seq = d.seq;
            print_gps_data(&d);
        }
        if (events & EVT_RTC_SYNC)
        {
            ESP_LOGI(TAG, "RTC synced");
        }
    }
}
#endif

/* ========================================================================== */
/*                              INITIALIZATION                                 */
/* ========================================================================== */

static void gps_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, UART_PIN_NO_CHANGE, GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, UART_RX_RING_BUF, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART%d init OK - RX=GPIO%d - %d baud", GPS_UART_PORT, GPS_RX_GPIO, GPS_BAUD_RATE);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== GPS HUD - ATGM336H / ESP32 ===");
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    // LVGL mutex
    s_lvgl_mutex = xSemaphoreCreateMutex();
    configASSERT(s_lvgl_mutex);

    // GPIO init
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT) | (1ULL << LED_RED) |
                        (1ULL << LED_BLUE) | (1ULL << LED_GREEN),
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_BLUE, 1);
    gpio_set_level(LED_GREEN, 1);
    gpio_set_level(PIN_NUM_BK_LIGHT, 0);

    // SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_HOR_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // LCD Panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // LCD Panel
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);

    // LVGL Init
    lv_init();
    lv_display_t *display = lv_display_create(LCD_HOR_RES, LCD_VER_RES);
    size_t draw_buffer_sz = LCD_HOR_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    configASSERT(buf1 && buf2);

    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(display, panel_handle);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    // LVGL tick timer
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    // Flush ready callback
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    // Touch
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = {
        .cs_gpio_num = PIN_NUM_TOUCH_CS,
        .dc_gpio_num = GPIO_NUM_NC,
        .spi_mode = 0,
        .pclk_hz = 2 * 1000 * 1000,
        .trans_queue_depth = 3,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_HOR_RES,
        .y_max = LCD_VER_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = PIN_NUM_TOUCH_INT,
        .flags = {.swap_xy = 0, .mirror_x = 1, .mirror_y = 1},
    };
    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, tp);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);
    LVGL_LOCK();
    lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);
    lvgl_port_update_callback(display);
    ui_init();
    LVGL_UNLOCK();

    // RTC & UART
    gps_rtc_init();
    gps_uart_init();

    // Create tasks
    // LVGL task
    xTaskCreatePinnedToCore(ui_lvgl_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL,
                            LVGL_TASK_PRIORITY, &ui_lvgl_task_handle, 0);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 3072, NULL, 6, &gps_task_handle, 0);
    xTaskCreatePinnedToCore(rtc_sync_task, "rtc_sync_task", 2048, NULL, 5, &rtc_task_handle, 0);
#if DEBUG_TASK
    xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL, 5, &dbg_task_handle, 0);
#endif
}