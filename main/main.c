/**
 * ============================================================================
 *  GPS HUD – ESP32 + ATGM336H
 *  ESP-IDF 5.x / 6.x
 *
 *  DATA FLOW:
 *      UART → NMEA FSM → double buffer → notify →
 *          ├── RTC task  (sync system time from GPS)
 *          ├── UI task   (update screen widgets)
 *          └── DEBUG     (optional serial logging)
 *
 *  DESIGN GOALS:
 *      - Lock-free GPS data sharing via atomic double-buffer swap
 *      - LVGL thread-safety via mutex (single writer, single renderer)
 *      - No blocking in ISR or time-critical paths
 *      - Event-driven UI with smooth, frame-rate-aware rendering
 * ============================================================================
 *  *****NMEA message output from ATGM336H
 *  $GNGGA,064939.000,2101.19944,N,10550.49781,E,1,05,7.9,66.8,M,-27.8,M,,*51
 *  $GNGLL,2101.19944,N,10550.49781,E,064939.000,A,A*47
 *  $GNGSA,A,3,14,17,19,30,,,,,,,,,9.6,7.9,5.5,1*38
 *  $GNGSA,A,3,23,,,,,,,,,,,,9.6,7.9,5.5,4*34
 *  $GPGSV,2,1,05,14,36,329,32,17,39,279,34,19,21,258,29,22,,,21,0*51
 *  $GPGSV,2,2,05,30,64,281,40,0*5E
 *  $BDGSV,1,1,01,23,52,332,33,0*41
 *  $GNRMC,064939.000,A,2101.19944,N,10550.49781,E,0.00,0.00,300126,,,A,V*0C
 *  $GNVTG,0.00,T,,M,0.00,N,0.00,K,A*23
 *  $GNZDA,064939.000,30,01,2026,00,00*4D
 *  $GPTXT,01,01,01,ANTENNA OK*35
 * ============================================================================
 *  TODO
 *  1) [IN PROGRESS] Chuyển các logic tính toán ra khỏi ui_task về gps_task
 *     (tính timeout, format lat/long, hdop_to_level …)
 *  2) Persist speed_compensation vào NVS để giữ giá trị qua reboot.
 *  3) Cân nhắc tách EVT_GPS_TIMEOUT thành event riêng thay vì kiểm tra
 *     trong ui_task mỗi frame.
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

/* --- GPIO & UART --- */
#define GPS_UART_PORT UART_NUM_2
#define GPS_RX_GPIO GPIO_NUM_21
#define LED_RED GPIO_NUM_4
#define LED_GREEN GPIO_NUM_17
#define LED_BLUE GPIO_NUM_16
#define PIN_NUM_BK_LIGHT GPIO_NUM_27
#define GPS_BAUD_RATE 115200
#define UART_RX_RING_BUF 2048                           // Ring buffer size for incoming NMEA data
#define UART_READ_BUF_SZ 256                            // Single read chunk per UART polling cycle
#define UART_READ_TIMEOUT_MS 10                         // How long to wait for each UART read attempt
#define GPS_COM_TIMEOUT_MS 2000                         // If no GPS packet arrives within this window, declare signal lost
#define GPS_RTC_STALE_THRESHOLD_MS (2 * 60 * 60 * 1000) // 2h (2 * 60 * 60 * 1000)

/* --- FreeRTOS event bit flags (used with xTaskNotify) --- */
#define EVT_GPS_UPDATE (1 << 0)       // gps_task → rtc_task, ui_task, dbg_task
#define EVT_RTC_SYNC_DONE (1 << 1)    // rtc_task → ui_task, dbg_task
#define EVT_RTC_SYNC_REQUEST (1 << 2) // gps_task → rtc_task (chỉ gps_task gửi)
#define EVT_GPS_TIMEOUT (1 << 3)      // gps_task → ui_task (mất tín hiệu module)
#define EVT_GPS_RESTORED (1 << 4)     // gps_task → ui_task (module phản hồi lại)
#define EVT_RTC_STALE (1 << 5)        // gps_task → ui_task: RTC vượt ngưỡng stale
#define EVT_RTC_NOT_STALE (1 << 6)    // gps_task → ui_task: RTC vừa được sync lại

/* --- Task handles (populated at startup) --- */
static TaskHandle_t gps_task_handle;
static TaskHandle_t rtc_task_handle;
static TaskHandle_t ui_task_handle;
static TaskHandle_t lvgl_task_handle;
#if DEBUG_TASK
static TaskHandle_t dbg_task_handle;
#endif

/* ========================================================================== */
/*                              LVGL CONFIG                                    */
/* ========================================================================== */

/*
 * Single mutex guards all LVGL API calls.
 * Only one task (ui_task) ever calls LVGL; the mutex exists as a safety
 * guard when the initialization path (app_main) also touches LVGL before the
 * task is running.
 */
static SemaphoreHandle_t s_lvgl_mutex;
#define LVGL_LOCK() xSemaphoreTake(s_lvgl_mutex, portMAX_DELAY)
#define LVGL_UNLOCK() xSemaphoreGive(s_lvgl_mutex)

/* --- SPI bus for LCD + Touch (shared SPI2) --- */
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

/* --- Physical display resolution --- */
#define LCD_HOR_RES 320
#define LCD_VER_RES 480
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

/*
 * LVGL draw buffer: 60 lines × 320 px × 2 bytes = ~37 KB per buffer.
 * Two buffers enable double-buffered DMA transfers so the CPU can prepare
 * the next tile while the previous one is being clocked out over SPI.
 *
 * RECOMMENDATION: Increasing LVGL_DRAW_BUF_LINES (e.g. to 120) reduces the
 * number of partial-flush calls per frame and can improve perceived smoothness
 * at the cost of extra PSRAM/DRAM usage.
 */
#define LVGL_DRAW_BUF_LINES 60
#define LVGL_TICK_PERIOD_MS 1      // lv_tick_inc() is called every 1 ms via esp_timer ISR
#define LVGL_TASK_MAX_DELAY_MS 500 // Upper bound on vTaskDelay between lv_timer_handler() calls
#define LVGL_TASK_MIN_DELAY_MS 5   // Lower bound – prevents busy-waiting the CPU
#define LVGL_TASK_STACK_SIZE (8 * 1024)
#define LVGL_TASK_PRIORITY 5

#define SYNC_SYMBOL "\xEF\x80\xA1"
#define SIG_NONE_SYMBOL "\xEF\x9A\x94"
#define SIG_BAD_SYMBOL "\xEF\x9A\x91"
#define SIG_MORDERATE_SYMBOL "\xEF\x9A\x92"
#define SIG_GOOD_SYMBOL "\xEF\x9A\x93"
#define SIG_EX_SYMBOL "\xEF\x9A\x90"

/* ========================================================================== */
/*                              GPS SHARED DOUBLE BUFFER                       */
/* ========================================================================== */

/*
 * Lock-free single-producer / multiple-consumer double buffer.
 *
 * How it works:
 *   - gps_task (writer) always writes to the *inactive* slot (index ^ 1),
 *     then atomically flips `index` to point readers at the new data.
 *   - Readers (ui_task, rtc_sync_task) call gps_read_latest(), which
 *     re-reads `index` before and after copying to detect a concurrent flip.
 *     If a flip happened mid-copy, the read is retried.
 *
 * This avoids any mutex in the hot data path.
 *
 * RECOMMENDATION: If gps_data_t grows large (> a cache line), consider
 * wrapping with a seqlock instead of the retry loop for more deterministic
 * worst-case latency.
 */
typedef struct
{
    gps_data_t buf[2];      // Two slots; only the one at [index] is "live"
    volatile uint8_t index; // Index of the most recently completed write
} gps_shared_t;

static gps_shared_t g_gps = {0};

/**
 * @brief Safely read the latest GPS data from the double buffer.
 *
 * Retries if a concurrent writer flipped `index` mid-copy.
 *
 * @param out  Destination struct to fill with the latest GPS data.
 */
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

/* ========================================================================== */
/*                              ODOMETER                                      */
/* ========================================================================== */
/* --- Odometer & EMA Configuration --- */
static double g_odometer_m = 0.0;    // Tổng quãng đường tính bằng mét
static float g_ema_speed_kmh = 0.0f; // Vận tốc sau khi lọc EMA
static float g_ema_alpha = 0.3f;     // Hệ số lọc (0.1 - 0.5), càng nhỏ càng mượt nhưng càng trễ
static uint32_t g_last_odo_tick = 0; // Lưu tick cuối cùng để tính delta time

/**
 * @brief Cập nhật Odometer và lọc vận tốc EMA.
 * @param current_speed_kmh Vận tốc thô từ GPS module.
 * @param is_valid Trạng thái fix của GPS.
 * @param reset_ema Nếu true, gán thẳng vận tốc hiện tại vào EMA (dùng khi mới có fix lại).
 */
static void update_odometer_ema(float current_speed_kmh, bool is_valid, bool reset)
{
    if (!is_valid)
    {
        g_ema_speed_kmh = 0;
        // Không cập nhật g_last_odo_tick ở đây để khi có fix lại,
        // logic reset ở gps_task sẽ xử lý.
        return;
    }

    uint32_t now = xTaskGetTickCount();

    if (reset)
    {
        g_ema_speed_kmh = current_speed_kmh;
        g_last_odo_tick = now; // Chốt mốc thời gian mới
        return;                // THOÁT: Không cộng dồn quãng đường cho frame reset này
    }

    // Tính Delta Time
    double delta_t_sec = (double)(now - g_last_odo_tick) / configTICK_RATE_HZ;
    g_last_odo_tick = now;

    // Lọc EMA
    g_ema_speed_kmh = (g_ema_alpha * current_speed_kmh) + ((1.0f - g_ema_alpha) * g_ema_speed_kmh);

    // Cộng dồn Odometer (chỉ cộng khi vận tốc đủ lớn để lọc nhiễu)
    if (g_ema_speed_kmh > 0.8f)
    {
        g_odometer_m += (g_ema_speed_kmh / 3.6) * delta_t_sec;
    }
}

/* ========================================================================== */
/*                              SIGNAL QUALITY HELPERS                         */
/* ========================================================================== */

/**
 * @brief Discrete signal quality levels derived from HDOP.
 *
 * HDOP (Horizontal Dilution of Precision) thresholds follow standard GPS
 * classification:
 *   < 1.5  → Excellent   (ideal conditions, survey-grade accuracy)
 *   < 2.5  → Good        (suitable for navigation)
 *   < 5.0  → Moderate    (acceptable, open sky with some obstruction)
 *   >= 5.0 → Bad         (unreliable fix; heavy urban canyon / indoors)
 *   special → No Signal  (GPS module not communicating at all)
 */
typedef enum
{
    SIG_NOSIGNAL = 0,
    SIG_BAD,
    SIG_MODERATE,
    SIG_GOOD,
    SIG_EXCELLENT
} signal_level_t;

/**
 * @brief Convert a raw HDOP float to a discrete signal_level_t.
 *
 * Called on every GPS frame; the result is compared against the previous
 * level so the signal-strength icon is only redrawn when it changes.
 */
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

static const void *signal_icon_table[] = {
    &SIG_NONE_SYMBOL,      // SIG_NOSIGNAL
    &SIG_BAD_SYMBOL,       // SIG_BAD
    &SIG_MORDERATE_SYMBOL, // SIG_MODERATE
    &SIG_GOOD_SYMBOL,      // SIG_GOOD
    &SIG_EX_SYMBOL         // SIG_EXCELLENT
};

/* ========================================================================== */
/*                              UTILITY FUNCTIONS                              */
/* ========================================================================== */

/**
 * @brief Compare two local_time_t structs for full equality (including validity).
 *
 * Used in the UI task to skip label updates when the displayed time has not
 * changed, reducing unnecessary LVGL redraws.
 */
bool compare_hh_mm(local_time_t *a, local_time_t *b)
{
    return (a->hour == b->hour) &&
           (a->minute == b->minute);
}

bool compare_ss(local_time_t *a, local_time_t *b)
{
    return (a->second == b->second);
}

bool compare_dd(local_time_t *a, local_time_t *b)
{
    return (a->day == b->day);
}

/**
 * @brief Format a decimal-degree latitude into a human-readable DMS string.
 *
 * Output example: "LAT : 10°46.312'N"
 *
 * @param lat  Decimal degrees (positive = North, negative = South).
 * @param buf  Destination character buffer.
 * @param len  Size of buf.
 */
static void format_lat(double lat, char *buf, size_t len)
{
    char hemi = (lat >= 0) ? 'N' : 'S';
    lat = fabs(lat);
    int deg = (int)lat;
    double min = (lat - deg) * 60.0;
    snprintf(buf, len, "LAT : %02d°%06.3f'%c", deg, min, hemi);
}

/**
 * @brief Format a decimal-degree longitude into a human-readable DMS string.
 *
 * Output example: "LONG: 106°41.534'E"
 *
 * @param lon  Decimal degrees (positive = East, negative = West).
 * @param buf  Destination character buffer.
 * @param len  Size of buf.
 */
static void format_lon(double lon, char *buf, size_t len)
{
    char hemi = (lon >= 0) ? 'E' : 'W';
    lon = fabs(lon);
    int deg = (int)lon;
    double min = (lon - deg) * 60.0;
    snprintf(buf, len, "LONG: %03d°%06.3f'%c", deg, min, hemi);
}

/* ========================================================================== */
/*                              DEBUG TASK (OPTIONAL)                          */
/* ========================================================================== */

#if DEBUG_TASK
/**
 * @brief Print GPS data to the serial console.
 *
 * Time / date are sourced from the local RTC (UTC+7), NOT from gps_data_t,
 * to match exactly what the UI displays. Raw GPS UTC values are shown only
 * as a fallback before the first RTC sync.
 *
 * Position, speed and signal quality are always read directly from gps_data_t.
 *
 * @param gps  Pointer to the latest GPS data snapshot.
 */
static void print_gps_data(const gps_data_t *gps)
{
    local_time_t lt;
    gps_rtc_get_local_time(&lt);

    ESP_LOGI(TAG, "--------------------------------------");

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
         * RTC has never been synced yet (waiting for first GPS fix).
         * Fall back to raw UTC from the GPS sentence as a temporary display.
         */
        ESP_LOGI("DEBUG", "Time (UTC) : %02d:%02d:%02d  [waiting for RTC sync]",
                 gps->hour, gps->minute, gps->second);
        ESP_LOGI("DEBUG", "Date (UTC) : %02d/%02d/%04d",
                 gps->day, gps->month, gps->year);
    }

    if (gps->valid)
    {
        ESP_LOGI("DEBUG", "Fix valid  : YES");
        ESP_LOGI("DEBUG", "Latitude   : %.7f %s",
                 gps->latitude >= 0 ? gps->latitude : -gps->latitude,
                 gps->latitude >= 0 ? "N" : "S");
        ESP_LOGI("DEBUG", "Longitude  : %.7f %s",
                 gps->longitude >= 0 ? gps->longitude : -gps->longitude,
                 gps->longitude >= 0 ? "E" : "W");
        ESP_LOGI("DEBUG", "Satellites : %d", gps->satellites);
        ESP_LOGI("DEBUG", "HDOP       : %.2f", (double)gps->hdop);
        ESP_LOGI("DEBUG", "Altitude   : %.1f m", (double)gps->altitude_m);
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

/*
 * speed_compensation: user-adjustable additive offset (+0 … +4 km/h).
 * Added to the GPS speed only when the vehicle is moving (speed > 0),
 * so a stationary reading stays at 0.
 *
 * RECOMMENDATION: Persist speed_compensation to NVS so it survives reboots.
 */
static uint8_t speed_compensation = 0;
static int current_screen = SCREEN_ID_SRC_MAIN;

/*
 * One shared LVGL timer for the "show image for 2 s then hide" pattern.
 * Re-using a single timer avoids unbounded timer creation if the event fires
 * frequently (e.g. rapid RTC sync during weak signal).
 */
static lv_timer_t *label_timer = NULL;

/**
 * @brief LVGL timer callback – ẩn widget chỉ định sau 2 giây.
 *
 * Sau khi ẩn, timer bị pause để không tự kích hoạt lại cho đến lần
 * gọi ui_show_label_2s() tiếp theo.
 */
static void hide_label_cb(lv_timer_t *t)
{
    lv_obj_t *label = (lv_obj_t *)lv_timer_get_user_data(t);
    if (!lv_obj_has_flag(label, LV_OBJ_FLAG_HIDDEN))
        lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
    lv_timer_pause(t);
}

/**
 * @brief Hiển thị một LVGL widget trong 2 giây rồi tự ẩn.
 *
 * An toàn khi gọi lặp lại: nếu widget đang hiển thị, countdown 2 giây
 * sẽ được reset lại từ đầu.
 *
 * @param label  Widget LVGL cần hiển thị tạm thời.
 */
void ui_show_label_2s(lv_obj_t *label)
{
    if (lv_obj_has_flag(label, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(label, LV_OBJ_FLAG_HIDDEN);

    if (label_timer)
    {
        lv_timer_set_user_data(label_timer, label);
        lv_timer_reset(label_timer);
        lv_timer_resume(label_timer);
    }
    else
    {
        label_timer = lv_timer_create(hide_label_cb, 2000, label);
    }
}

/**
 * @brief Increment the speed compensation offset (cycles 0 → 1 → … → 4 → 0).
 *
 * Triggered by the "+" button on the main screen.
 * Both label objects are updated so the value is visible regardless of which
 * screen is currently active.
 */
static void btn_inc_cb(lv_event_t *e)
{
    speed_compensation = (speed_compensation > 4) ? 0 : speed_compensation + 1;
    ESP_LOGI("UI", "Speed compensation: +%d", speed_compensation);
    lv_label_set_text_fmt(objects.speed_adjust_main, "+%d", speed_compensation);
}

/**
 * @brief Advance to the next screen, wrapping around after the last screen.
 */
static void btn_next_screen_cb(lv_event_t *e)
{
    current_screen++;
    if (current_screen > _SCREEN_ID_LAST)
        current_screen = _SCREEN_ID_FIRST;
    loadScreen(current_screen);
    ESP_LOGI("UI", "Go to next screen");
}

/**
 * @brief Go back to the previous screen, wrapping around before the first screen.
 */
static void btn_prev_screen_cb(lv_event_t *e)
{
    current_screen--;
    if (current_screen < _SCREEN_ID_FIRST)
        current_screen = _SCREEN_ID_LAST;
    loadScreen(current_screen);
    ESP_LOGI("UI", "Go to previous screen");
}

/**
 * @brief Called by the LCD panel IO driver when a DMA transfer completes.
 *
 * Signals LVGL that the framebuffer flush is done so it can release the
 * draw buffer and start the next rendering pass.
 *
 * Runs in ISR context – must not call blocking APIs.
 */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/**
 * @brief Reconfigure the ST7796 swap/mirror registers when the display
 *        rotation changes.
 *
 * Called by LVGL whenever lv_display_set_rotation() is invoked.
 * Each rotation case maps to the correct hardware mirror/swap combination
 * for the ST7796 controller.
 */
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

/**
 * @brief LVGL flush callback – sends a rendered tile to the LCD via SPI DMA.
 *
 * Byte-swaps the RGB565 pixel data in-place before sending, because the
 * ST7796 expects big-endian 16-bit colour words while LVGL produces
 * little-endian.
 *
 * RECOMMENDATION: If the DMA engine supports hardware byte-swap, enable it
 * in spi_device_interface_config_t to eliminate the software swap and save
 * CPU time proportional to (area * 2) bytes per flush.
 */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int x1 = area->x1, x2 = area->x2;
    int y1 = area->y1, y2 = area->y2;
    lv_draw_sw_rgb565_swap(px_map, (x2 + 1 - x1) * (y2 + 1 - y1));
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, px_map);
}

/**
 * @brief esp_timer periodic callback that drives the LVGL tick counter.
 *
 * Runs in ISR context every LVGL_TICK_PERIOD_MS (1 ms).
 * lv_tick_inc() is ISR-safe per the LVGL documentation.
 */
static void increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

/**
 * @brief LVGL input device read callback for the XPT2046 touch controller.
 *
 * Called by LVGL on every input scan cycle. Reads the latest touch
 * coordinates from the driver and maps them to LVGL's pointer state.
 *
 * RECOMMENDATION: Apply a simple IIR low-pass filter on touchpad_x/y to
 * reduce jitter – the XPT2046 ADC can be noisy without hardware averaging.
 */
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
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* ========================================================================== */
/*                              TASKS                                          */
/* ========================================================================== */

/**
 * @brief GPS UART reader & NMEA parser task.
 *
 * Polls UART2 in a tight loop (10 ms timeout per read), feeds raw bytes into
 * the incremental NMEA FSM parser, and on each completed sentence:
 *   1. Writes the new gps_data_t into the inactive double-buffer slot.
 *   2. Atomically flips the slot index (making the new data "live").
 *   3. Notifies rtc_sync_task and ui_task via FreeRTOS task notification.
 *
 * The `seq` counter in gps_data_t allows receivers to detect stale reads
 * without additional locking.
 *
 * RECOMMENDATION: Consider raising GPS_BAUD_RATE to 115200 (already set) and
 * configuring the ATGM336H to output only the NMEA sentences you actually use
 * (e.g. $GPRMC + $GPGGA) to reduce parse load.
 */
static void gps_task(void *arg)
{
    nmea_parser_t parser;
    nmea_parser_init(&parser);
    uint32_t last_seq = 0;
    static uint8_t rx_buf[UART_READ_BUF_SZ];
    TickType_t last_parsed_tick = xTaskGetTickCount(); // Thời điểm nhận byte cuối cùng
    TickType_t last_sync_tick = xTaskGetTickCount();   // Reset mỗi khi rtc_task sync xong
    bool gps_timeout = false;                          // Trạng thái timeout hiện tại
    bool last_stale = false;
    uint32_t events = 0;
    bool last_valid = false;
    TickType_t last_stale_check = xTaskGetTickCount();

    ESP_LOGI("GPS", "GPS task started");

    while (1)
    {
        /* ── Non-blocking: nhận EVT_RTC_SYNC_DONE từ rtc_task ───────────── */
        if (xTaskNotifyWait(0, EVT_RTC_SYNC_DONE, &events, 0) == pdTRUE)
        {
            if (events & EVT_RTC_SYNC_DONE)
            {
                last_sync_tick = xTaskGetTickCount();
                ESP_LOGI("GPS", "RTC sync done – stale timer reset");
            }
        }

        int rx_len = uart_read_bytes(GPS_UART_PORT, rx_buf, sizeof(rx_buf),
                                     pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
        if (rx_len > 0)
        {
            nmea_parser_feed(&parser, rx_buf, (size_t)rx_len);
            TickType_t now = xTaskGetTickCount();

            /* ── Chỉ xử lý khi parse được bản tin NMEA hoàn chỉnh mới ──────── */
            if (parser.data.seq != last_seq)
            {
                last_parsed_tick = now;
                // Nếu vừa hồi phục từ timeout hoặc vừa có fix, hãy reset EMA
                bool ema_need_reset = gps_timeout || (parser.data.valid && !last_valid);
                if (ema_need_reset)
                {
                    // QUAN TRỌNG: Đồng bộ lại tick để delta_t của lần tính tới sẽ bắt đầu từ điểm này
                    g_last_odo_tick = now;
                    // Gán thẳng vận tốc hiện tại cho EMA để tránh việc lọc từ giá trị 0 cũ
                    g_ema_speed_kmh = parser.data.valid ? parser.data.speed_kmh : 0;
                    ESP_LOGI("GPS", "EMA/Odo Reset - Syncing tick to prevent jump");
                }
                /* Phục hồi: chỉ báo UI sau khi có bản tin hợp lệ, không phải khi có byte */
                if (gps_timeout)
                {
                    gps_timeout = false;
                    xTaskNotify(ui_task_handle, EVT_GPS_RESTORED, eSetBits);
                    ESP_LOGI("GPS", "[EVT_GPS_RESTORED -> ui], GPS restored – notifying UI");

                    xTaskNotify(rtc_task_handle, EVT_RTC_SYNC_REQUEST, eSetBits);
                    ESP_LOGI("GPS", "[EVT_RTC_SYNC_REQUEST -> rtc]");
                }

                update_odometer_ema(parser.data.speed_kmh, parser.data.valid, ema_need_reset);
                parser.data.odometer_m = g_odometer_m;
                /* Write to the inactive slot, then flip the index atomically. */
                uint8_t next = g_gps.index ^ 1;
                g_gps.buf[next] = parser.data;
                __atomic_store_n(&g_gps.index, next, __ATOMIC_RELEASE);
                /* ── No fix → Fix: request sync ngay ───────────────────────────────── */
                bool valid_changed = (parser.data.valid != last_valid);
                if (valid_changed && parser.data.valid)
                {
                    xTaskNotify(rtc_task_handle, EVT_RTC_SYNC_REQUEST, eSetBits);
                    ESP_LOGI("GPS", "[EVT_RTC_SYNC_REQUEST -> rtc], GPS no fix → fix, request RTC sync (seq=%lu)", parser.data.seq);
                }
                last_valid = parser.data.valid;
                /* Quyết định RTC sync */
                if (gps_rtc_should_sync(&parser.data))
                {
                    xTaskNotify(rtc_task_handle, EVT_RTC_SYNC_REQUEST, eSetBits);
                    ESP_LOGI("GPS", "[EVT_RTC_SYNC_REQUEST -> rtc] RTC sync request (seq=%lu)", parser.data.seq);
                }

                xTaskNotify(ui_task_handle, EVT_GPS_UPDATE, eSetBits);
#if DEBUG_TASK
                ESP_LOGI("DEBUG", "[EVT_GPS_UPDATE -> debug]");
                xTaskNotify(dbg_task_handle, EVT_GPS_UPDATE, eSetBits);
#endif
                last_seq = parser.data.seq;
            }
        }
        else
        {
            /* No data in this window – yield to avoid starving lower-priority tasks. */
            /* ── Không có bản tin mới: kiểm tra timeout ─────────────────── */
            TickType_t now = xTaskGetTickCount();
            if (!gps_timeout &&
                (now - last_parsed_tick) > pdMS_TO_TICKS(GPS_COM_TIMEOUT_MS))
            {
                gps_timeout = true;
                xTaskNotify(ui_task_handle, EVT_GPS_TIMEOUT, eSetBits);
                ESP_LOGW("GPS", " [EVT_GPS_TIMEOUT -> ui] GPS timeout > %ds – notifying UI", GPS_COM_TIMEOUT_MS / 1000);
                g_last_odo_tick = 0; // Reset tick để không tính quãng đường nhảy vọt khi có tín hiệu lại
                g_ema_speed_kmh = 0;
            }
        }
        last_stale_check = xTaskGetTickCount();
        /* ── Stale check: so sánh tick nội bộ, không gọi API ────────── */
        bool is_stale = (last_stale_check - last_sync_tick) >
                        pdMS_TO_TICKS(GPS_RTC_STALE_THRESHOLD_MS);
        if (is_stale != last_stale)
        {
            last_stale = is_stale;
            if (is_stale)
            {
                xTaskNotify(ui_task_handle, EVT_RTC_STALE, eSetBits);
                ESP_LOGW("GPS", "[EVT_RTC_STALE -> ui] RTC stale – no sync for > %llums",
                         (uint64_t)GPS_RTC_STALE_THRESHOLD_MS);
                // Cần check gps timeout, do parser không update nếu timeout, vẫn giữ giá trị cũ
                if ((!gps_timeout) && (parser.data.valid))
                {
                    xTaskNotify(rtc_task_handle, EVT_RTC_SYNC_REQUEST, eSetBits);
                    ESP_LOGI("GPS", "[EVT_RTC_SYNC_REQUEST -> rtc] Force RTC sync due to stale (seq=%lu)", parser.data.seq);
                }
            }
            else
            {
                xTaskNotify(ui_task_handle, EVT_RTC_NOT_STALE, eSetBits);
                ESP_LOGI("GPS", "[EVT_RTC_NOT_STALE -> ui] RTC no longer stale");
            }
        }
    }
}

/**
 * @brief Thực thi lệnh sync RTC từ gps_task.
 *
 * Task này không tự quyết định khi nào cần sync – mọi logic (should_sync,
 * stale, force sync khi fix phục hồi) đã được gps_task xử lý. rtc_sync_task
 * chỉ đóng vai trò executor:
 *
 *   1. Chờ EVT_RTC_SYNC_REQUEST từ gps_task (block vô thời hạn).
 *   2. Đọc GPS data mới nhất từ double buffer.
 *   3. Gọi gps_rtc_sync() để cập nhật system time.
 *   4. Gửi EVT_RTC_SYNC_DONE về:
 *        - gps_task  : reset stale timer nội bộ.
 *        - ui_task   : flash icon sync 2 giây.
 *        - dbg_task  : log (chỉ khi DEBUG_TASK = 1).
 */
static void rtc_sync_task(void *arg)
{
    ESP_LOGI("RTC", "RTC sync task started");
    gps_data_t d;
    uint32_t events;

    while (1)
    {
        /*
         * Chỉ wake up khi nhận EVT_RTC_SYNC_REQUEST từ gps_task.
         * Mọi quyết định "có nên sync không" và "stale chưa" đã được
         * gps_task xử lý – rtc_task chỉ thực thi lệnh sync và báo lại.
         */
        if (xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY) == pdTRUE)
        {
            if (events & EVT_RTC_SYNC_REQUEST)
            {
                gps_read_latest(&d);
                gps_rtc_sync(&d);

                /* Báo gps_task reset stale timer nội bộ */
                xTaskNotify(gps_task_handle, EVT_RTC_SYNC_DONE, eSetBits);
                /* Báo ui_task flash icon sync */
                xTaskNotify(ui_task_handle, EVT_RTC_SYNC_DONE, eSetBits);
                ESP_LOGI("RTC", "[EVT_RTC_SYNC_DONE -> gps, ui]");

#if DEBUG_TASK
                xTaskNotify(dbg_task_handle, EVT_RTC_SYNC_DONE, eSetBits);
                ESP_LOGI("DEBUG", "[RTC_SYNC_TASK -> debug]");
#endif
            }
        }
    }
}

/* ========================================================================== */
/*                              UI + LVGL TASK                                */
/* ========================================================================== */

/**
 * @brief LVGL rendering task – gọi lv_timer_handler() theo chu kỳ thích nghi.
 *
 * Task này là vòng lặp render thuần tuý: không xử lý GPS, không đọc RTC.
 * Mọi logic nghiệp vụ nằm ở ui_task; lvgl_port_task chỉ bơm LVGL timer
 * để các widget dirty được vẽ ra màn hình.
 *
 * Delay thích nghi:
 *   lv_timer_handler() trả về số ms đến timer LVGL tiếp theo. Giá trị này
 *   được clamp vào [LVGL_TASK_MIN_DELAY_MS, LVGL_TASK_MAX_DELAY_MS] để
 *   tránh busy-wait mà vẫn đảm bảo wakeup tối thiểu 2 lần/giây.
 */
static void lvgl_port_task(void *arg)
{
    ESP_LOGI("LVGL", "Starting LVGL PORT task (rendering)");
    uint32_t time_till_next_ms = 0;
    while (1)
    {
        LVGL_LOCK();
        time_till_next_ms = lv_timer_handler();
        LVGL_UNLOCK();
        /* ── ADAPTIVE DELAY ─────────────────────────────────────────────── */
        /*
         * lv_timer_handler() trả về ms đến LVGL timer kế tiếp.
         * Clamp MIN=5 ms tránh CPU spin, MAX=500 ms đảm bảo wakeup tối thiểu.
         */
        time_till_next_ms = LV_MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
        time_till_next_ms = LV_MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}

/**
 * @brief Cập nhật giao diện LVGL theo event từ gps_task và rtc_task.
 *
 * Task này là consumer thuần tuý – không tự tính toán timeout, stale hay
 * quyết định sync RTC. Mọi trạng thái được gps_task và rtc_task xử lý
 * trước rồi thông báo qua xTaskNotify.
 *
 * Loop structure per iteration:
 *   1. LVGL_LOCK
 *   2. Poll RTC clock – cập nhật label giờ/phút/giây/ngày khi thay đổi.
 *      So sánh từng trường riêng lẻ để tránh redraw không cần thiết.
 *   3. xTaskNotifyWait(timeout = 0) – non-blocking, xử lý event:
 *        EVT_GPS_TIMEOUT    → blank speed/position, hiện NO GPS, ẩn speed unit.
 *        EVT_GPS_RESTORED   → khôi phục các widget bị ẩn.
 *        EVT_GPS_UPDATE     → cập nhật speed, sat, signal icon, lat, lon.
 *        EVT_RTC_STALE      → đổi màu đồng hồ sang xám.
 *        EVT_RTC_NOT_STALE  → đổi màu đồng hồ về vàng.
 *        EVT_RTC_SYNC_DONE  → flash icon sync 2 giây.
 *   4. LVGL_UNLOCK
 *   5. vTaskDelay(10 ms) – nhường CPU cho các task khác.
 *
 * Toàn bộ LVGL call nằm trong LVGL_LOCK/UNLOCK, kể cả xử lý event,
 * đảm bảo không có race condition với lvgl_port_task.
 *
 * RECOMMENDATION: Nếu thêm nhiều widget và stack overflow, tăng
 * LVGL_TASK_STACK_SIZE và kiểm tra watermark qua
 * uxTaskGetStackHighWaterMark() trong debug build.
 */
static void ui_task(void *arg)
{
    ESP_LOGI("UI", "UI task started");

    uint32_t events;
    gps_data_t d;
    gps_data_t last_data = {};
    uint8_t spinGps = 0;
    signal_level_t last_signal = 0xFF;
    local_time_t current_time = {};
    local_time_t last_time = {};
    char lat_buf[32];
    char lon_buf[32];

    last_time.valid = true;

    static const char *WEEKDAY_STR[] = {
        "Chủ nhật", "Thứ 2", "Thứ 3", "Thứ 4", "Thứ 5", "Thứ 6", "Thứ 7"};

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

    /* Đăng ký callback các nút touch */
    lv_obj_add_event_cb(objects.btn_inc, btn_inc_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(objects.main_next_scr, btn_next_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.main_prev_scr, btn_prev_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.time_next_scr, btn_next_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.time_prev_scr, btn_prev_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.info_next_scr, btn_next_screen_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.info_prev_scr, btn_prev_screen_cb, LV_EVENT_CLICKED, NULL);
    ESP_LOGI("UI", "Button callbacks registered");

    /* Khởi tạo icon mặc định */
    lv_label_set_text(objects.icon_sync_rtc, SYNC_SYMBOL);
    lv_label_set_text(objects.signal_bar_icon, SIG_NONE_SYMBOL);
    lv_obj_set_style_text_color(objects.signal_bar_icon,
                                lv_color_hex(0xff4c4c),
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    while (1)
    {
        LVGL_LOCK();

        /* ── SECTION 1: RTC CLOCK DISPLAY ───────────────────────────────── */
        /*
         * Poll RTC mỗi vòng để đồng hồ chạy liên tục kể cả khi không có
         * GPS packet mới. So sánh từng trường riêng để tránh redraw thừa.
         */
        gps_rtc_get_local_time(&current_time);

        if (current_time.valid != last_time.valid)
        {
            if (!current_time.valid)
            {
                lv_label_set_text(objects.hour_minute, "--:--");
                lv_label_set_text(objects.second, "--");
                lv_label_set_text(objects.date, "Waiting GPS");
            }
            else
            {
                /* Force redraw toàn bộ lần đầu có valid time */
                last_time.hour = -1;
                last_time.minute = -1;
                last_time.second = -1;
                last_time.day = -1;
            }
        }

        if (current_time.valid)
        {
            if (!compare_hh_mm(&current_time, &last_time))
                lv_label_set_text_fmt(objects.hour_minute, "%02d:%02d",
                                      current_time.hour, current_time.minute);

            if (!compare_ss(&current_time, &last_time))
                lv_label_set_text_fmt(objects.second, "%02d", current_time.second);

            if (!compare_dd(&current_time, &last_time))
                lv_label_set_text_fmt(objects.date, "%s, %02d/%02d/%04d",
                                      WEEKDAY_STR[current_time.week_day],
                                      current_time.day, current_time.month, current_time.year);
        }

        last_time = current_time;

        /* ── SECTION 2: EVENT DISPATCH ───────────────────────────────────── */
        if (xTaskNotifyWait(0, UINT32_MAX, &events, 0) == pdTRUE)
        {
            /* ── EVT_GPS_TIMEOUT ─────────────────────────────────────────── */
            if (events & EVT_GPS_TIMEOUT)
            {
                lv_label_set_text(objects.sat_num, "NO GPS");
                lv_label_set_text(objects.fix_info, "NO GPS");
                lv_label_set_text(objects.sat_info, "SAT : ");
                lv_label_set_text(objects.hdop_info, "HDOP: ");
                lv_label_set_text(objects.lat_info, "LAT : ");
                lv_label_set_text(objects.long_info, "LONG: ");
                lv_label_set_text(objects.speed_after_adjust, "");
                if (!lv_obj_has_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN))
                    lv_obj_add_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN);
                last_signal = SIG_NOSIGNAL;
                lv_label_set_text(objects.signal_bar_icon, signal_icon_table[SIG_NOSIGNAL]);
                lv_obj_set_style_text_color(objects.signal_bar_icon,
                                            lv_color_hex(0xff4c4c),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);
            }

            /* ── EVT_GPS_RESTORED ────────────────────────────────────────── */
            if (events & EVT_GPS_RESTORED)
            {
                if (lv_obj_has_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN))
                    lv_obj_clear_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN);
                if (lv_obj_has_flag(objects.sat_num, LV_OBJ_FLAG_HIDDEN))
                    lv_obj_clear_flag(objects.sat_num, LV_OBJ_FLAG_HIDDEN);
            }

            /* ── EVT_GPS_UPDATE ──────────────────────────────────────────── */
            if (events & EVT_GPS_UPDATE)
            {
                gps_read_latest(&d);
                // ESP_LOGI("UI", "ODO: %.2f m", d.odometer_m);
                spinGps = (spinGps + 1) % frames_len;
                if (d.odometer_m < 1000)
                {
                    lv_label_set_text_fmt(objects.odometer_m, "%.0f m", d.odometer_m);
                }
                else
                {
                    lv_label_set_text_fmt(objects.odometer_m, "%.1f km", d.odometer_m / 1000);
                }

                if (d.seq != last_data.seq)
                {
                    bool valid_changed = (d.valid != last_data.valid);
                    if (valid_changed && d.valid)
                        ESP_LOGI("UI", "GPS --> Fix");
                    else if (valid_changed && !d.valid)
                    {
                        ESP_LOGI("UI", "GPS --> No Fix");
                        lv_label_set_text(objects.signal_bar_icon,
                                          signal_icon_table[SIG_NOSIGNAL]);
                    }

                    signal_level_t new_signal = hdop_to_level(d.hdop);

                    int speed_kmh = (int)(d.speed_kmh + 0.5f);
                    if (speed_kmh < 1)
                        speed_kmh = 0;
                    else
                        speed_kmh += speed_compensation;

                    if (!d.valid)
                    {
                        lv_label_set_text(objects.speed_after_adjust, "");
                        lv_label_set_text(objects.fix_info, "FIX : NO");
                        lv_label_set_text(objects.sat_num, "NOT FIXED!");
                        lv_label_set_text_fmt(objects.hdop_info, "HDOP: %.1f (LAST)", d.hdop);
                        lv_label_set_text_fmt(objects.sat_info, "SAT : %d", d.satellites);
                        lv_label_set_text(objects.lat_info, "LAT :");
                        lv_label_set_text(objects.long_info, "LONG:");
                        if (last_signal != SIG_NOSIGNAL)
                        {
                            last_signal = SIG_NOSIGNAL;
                            lv_label_set_text(objects.signal_bar_icon,
                                              signal_icon_table[SIG_NOSIGNAL]);
                            lv_obj_set_style_text_color(objects.signal_bar_icon,
                                                        lv_color_hex(0xff4c4c),
                                                        LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                    }
                    else
                    {
                        if (new_signal != last_signal)
                        {
                            last_signal = new_signal;
                            lv_label_set_text(objects.signal_bar_icon,
                                              signal_icon_table[new_signal]);
                            switch (new_signal)
                            {
                            case SIG_MODERATE:
                                lv_obj_set_style_text_color(objects.signal_bar_icon,
                                                            lv_color_hex(0xFFB300),
                                                            LV_PART_MAIN | LV_STATE_DEFAULT);
                                break;
                            case SIG_GOOD:
                                lv_obj_set_style_text_color(objects.signal_bar_icon,
                                                            lv_color_hex(0x8BC34A),
                                                            LV_PART_MAIN | LV_STATE_DEFAULT);
                                break;
                            case SIG_EXCELLENT:
                                lv_obj_set_style_text_color(objects.signal_bar_icon,
                                                            lv_color_hex(0x4CAF50),
                                                            LV_PART_MAIN | LV_STATE_DEFAULT);
                                break;
                            default:
                                lv_obj_set_style_text_color(objects.signal_bar_icon,
                                                            lv_color_hex(0xff4c4c),
                                                            LV_PART_MAIN | LV_STATE_DEFAULT);
                                break;
                            }
                        }

                        lv_label_set_text_fmt(objects.speed_after_adjust, "%d", speed_kmh);
                        lv_label_set_text_fmt(objects.sat_num, "SAT: %d", d.satellites);
                        lv_label_set_text(objects.fix_info, "FIX : YES");
                        lv_label_set_text_fmt(objects.hdop_info, "HDOP: %.1f", d.hdop);
                        lv_label_set_text_fmt(objects.sat_info, "SAT : %d", d.satellites);
                        format_lat(d.latitude, lat_buf, sizeof(lat_buf));
                        format_lon(d.longitude, lon_buf, sizeof(lon_buf));
                        lv_label_set_text(objects.lat_info, lat_buf);
                        lv_label_set_text(objects.long_info, lon_buf);
                    }

                    lv_label_set_text(objects.gps_render_loading_indicator, frames[spinGps]);
                    lv_label_set_text(objects.gps_render_loading_indicator_1, frames[spinGps]);

                    last_data = d;
                }
            }

            /* ── EVT_RTC_STALE ───────────────────────────────────────────── */
            if (events & EVT_RTC_STALE)
            {
                lv_obj_set_style_text_color(objects.hour_minute,
                                            lv_color_hex(0xa0a0a0),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);
            }

            /* ── EVT_RTC_NOT_STALE ───────────────────────────────────────── */
            if (events & EVT_RTC_NOT_STALE)
            {
                lv_obj_set_style_text_color(objects.hour_minute,
                                            lv_color_hex(0xffbe00),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);
            }

            /* ── EVT_RTC_SYNC_DONE ───────────────────────────────────────── */
            if (events & EVT_RTC_SYNC_DONE)
            {
                ui_show_label_2s(objects.icon_sync_rtc);
                ESP_LOGI("UI", "RTC synced – icon shown");
            }
        }

        LVGL_UNLOCK();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ========================================================================== */
/*                              DEBUG TASK (OPTIONAL)                          */
/* ========================================================================== */

#if DEBUG_TASK
/**
 * @brief Serial debug task – mirrors GPS events to the UART console.
 *
 * Enabled only when DEBUG_TASK = 1. Receives the same notifications as
 * ui_task but only logs to ESP_LOGI; does not touch LVGL.
 *
 * RECOMMENDATION: Disable in production builds (DEBUG_TASK 0) to recover
 * ~2 KB of task stack and eliminate serial log overhead at runtime.
 */
static void debug_task(void *arg)
{
    ESP_LOGI("DEBUG", "Start debug task");
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
            UBaseType_t gps_stack = uxTaskGetStackHighWaterMark(gps_task_handle);
            UBaseType_t ui_stack = uxTaskGetStackHighWaterMark(ui_task_handle);
            UBaseType_t rtc_stack = uxTaskGetStackHighWaterMark(rtc_task_handle);
            UBaseType_t lvgl_stack = uxTaskGetStackHighWaterMark(lvgl_task_handle);
            ESP_LOGI("DEBUG", "Free GPS stack in bytes: %u", gps_stack);
            ESP_LOGI("DEBUG", "Free UI stack in bytes: %u", ui_stack);
            ESP_LOGI("DEBUG", "Free RTC stack in bytes: %u", rtc_stack);
            ESP_LOGI("DEBUG", "Free LVGL stack in bytes: %u", lvgl_stack);
        }
        if (events & EVT_RTC_SYNC_DONE)
        {
            ESP_LOGI("DEBUG", "[EVT_RTC_SYNC_DONE] Event received: RTC synced");
        }
    }
}
#endif

/* ========================================================================== */
/*                              HARDWARE INITIALISATION                        */
/* ========================================================================== */

/**
 * @brief Initialise UART2 for receiving NMEA sentences from the ATGM336H.
 *
 * Only RX is wired; TX is not needed (the module outputs NMEA autonomously).
 * No event queue is used – gps_task polls the driver ring buffer directly,
 * which keeps the ISR path minimal.
 *
 * RECOMMENDATION: If you need to send UBX/PMTK configuration commands to the
 * module (e.g. to change output rate), allocate a TX pin and set up a TX
 * buffer in uart_driver_install().
 */
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
    ESP_LOGI("GPS", "UART%d init OK – RX=GPIO%d – %d baud",
             GPS_UART_PORT, GPS_RX_GPIO, GPS_BAUD_RATE);
}

/* ========================================================================== */
/*                              APPLICATION ENTRY POINT                        */
/* ========================================================================== */

/**
 * @brief Main application entry point.
 *
 * Initialisation order:
 *   1. LVGL mutex
 *   2. GPIO (backlight, RGB LED)
 *   3. SPI bus (shared by LCD + touch)
 *   4. LCD panel IO → ST7796 panel driver → panel on
 *   5. LVGL core, display, DMA draw buffers, tick timer, flush callback
 *   6. XPT2046 touch controller → LVGL input device
 *   7. Rotation + UI layout (under mutex)
 *   8. RTC module, GPS UART
 *   9. FreeRTOS tasks
 *
 * Task pinning:
 *   All tasks are pinned to core 0. This is intentional – LVGL is not
 *   thread-safe across cores without a proper inter-core lock. Keeping
 *   everything on core 0 avoids cache-coherency surprises on the ESP32
 *   dual-core architecture.
 *
 * RECOMMENDATION: Consider moving gps_task to core 1 (tskNO_AFFINITY or
 * xTaskCreatePinnedToCore(..., 1)) to offload UART polling and NMEA parsing
 * away from the rendering core. The double-buffer + atomic swap design
 * already makes the GPS data handoff safe across cores.
 */
void app_main(void)
{
    ESP_LOGI("MAIN", "=== GPS HUD – ATGM336H / ESP32 ===");
    ESP_LOGI("MAIN", "IDF version: %s", esp_get_idf_version());

    /* --- LVGL mutex -------------------------------------------------------- */
    s_lvgl_mutex = xSemaphoreCreateMutex();
    configASSERT(s_lvgl_mutex);

    /* --- GPIO: backlight + RGB LED (active-low) ---------------------------- */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT) | (1ULL << LED_RED) |
                        (1ULL << LED_BLUE) | (1ULL << LED_GREEN),
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(LED_RED, 1); /* LEDs off */
    gpio_set_level(LED_BLUE, 1);
    gpio_set_level(LED_GREEN, 1);
    gpio_set_level(PIN_NUM_BK_LIGHT, 0); /* Backlight off until panel is ready */

    /* --- SPI bus (LCD + touch share SPI2) ---------------------------------- */
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_HOR_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* --- LCD panel IO (SPI → ST7796 command/data protocol) ----------------- */
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
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    /* --- ST7796 panel driver ----------------------------------------------- */
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
    gpio_set_level(PIN_NUM_BK_LIGHT, 1); /* Panel ready – enable backlight */

    /* --- LVGL initialisation ----------------------------------------------- */
    lv_init();
    lv_display_t *display = lv_display_create(LCD_HOR_RES, LCD_VER_RES);

    /*
     * Allocate draw buffers from DMA-capable memory so the SPI peripheral can
     * read them directly without an extra memcpy through the CPU.
     */
    size_t draw_buffer_sz = LCD_HOR_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    configASSERT(buf1 && buf2);

    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(display, panel_handle);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    /* --- LVGL 1 ms tick timer (esp_timer ISR) ------------------------------ */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    /* --- Register DMA-done callback so LVGL knows when a flush completes --- */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    /* --- XPT2046 touch controller ------------------------------------------ */
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
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &tp_io_config, &tp_io_handle));

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

    /* Set rotation and load initial UI while holding the mutex (app_main context). */
    LVGL_LOCK();
    lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);
    lvgl_port_update_callback(display);
    ui_init();
    LVGL_UNLOCK();

    /* --- RTC module and GPS UART ------------------------------------------- */
    gps_rtc_init();
    gps_uart_init();

    /* --- FreeRTOS tasks ---------------------------------------------------- */
    //  * Task pinning:
    //  *   lvgl_port_task và ui_task chạy trên core 1 để tách biệt rendering
    //  *   khỏi I/O. gps_task và rtc_sync_task chạy trên core 0 để xử lý UART
    //  *   và RTC độc lập với render loop. Double-buffer + atomic swap đảm bảo
    //  *   handoff an toàn giữa hai core.
    //  *
    //  *   Core 0: gps_task (priority 6), rtc_sync_task (priority 5)
    //  *   Core 1: lvgl_port_task (priority 5), ui_task (priority 5), debug_task (priority 5)
    xTaskCreatePinnedToCore(lvgl_port_task, "lvgl_port", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, &lvgl_task_handle, 1);
    xTaskCreatePinnedToCore(ui_task, "ui_task", 6144, NULL, 5, &ui_task_handle, 1);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 3072, NULL, 6, &gps_task_handle, 0);
    xTaskCreatePinnedToCore(rtc_sync_task, "rtc_sync_task", 2048, NULL, 5, &rtc_task_handle, 0);
#if DEBUG_TASK
    xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL, 5, &dbg_task_handle, 1);
#endif
}