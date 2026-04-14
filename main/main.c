/**
 * @file main.c
 * @brief GPS HUD – ESP32 + ATGM336H, ESP-IDF 5.x / 6.x
 *        Phiên bản với RTC local GMT+7.
 *
 * Luồng xử lý thời gian:
 *   GPS NMEA (UTC) → nmea_parser → gps_data_t (UTC)
 *                                      ↓
 *                          gps_rtc_should_sync()?
 *                               ↓ YES
 *                          gps_rtc_sync()        → settimeofday() [UTC]
 *                                                       ↓
 *   Hiển thị ← gps_rtc_get_local_time() ← gettimeofday() + localtime_r() [GMT+7]
 *
 * Thời gian GPS UTC vẫn lưu nguyên trong gps_data_t (không bị sửa đổi).
 * Hiển thị hoàn toàn dùng RTC local thông qua gps_rtc_get_local_time().
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "gps_nmea.h"
#include "gps_rtc.h"

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Cấu hình chương trình                                                      */
/* ─────────────────────────────────────────────────────────────────────────── */
#define DEBUG_TASK

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Cấu hình phần cứng                                                         */
/* ─────────────────────────────────────────────────────────────────────────── */

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
// Notify bit
#define EVT_GPS_UPDATE (1 << 0)
#define EVT_RTC_SYNC (1 << 1)

static const char *TAG = "CYD_3.5inch_GPS_HUD";
static TaskHandle_t gps_task_handle;
static TaskHandle_t rtc_task_handle;
static TaskHandle_t ui_task_handle;

#ifdef DEBUG_TASK
static TaskHandle_t dbg_task_handle;
#endif

// Double buffer, pointer swap (Producder ghi, consumer đọc)
typedef struct
{
    gps_data_t buf[2];
    volatile uint8_t index; // buffer đang active (0 hoặc 1)
} gps_shared_t;

static gps_shared_t g_gps = {0};

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Khởi tạo UART                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

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
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT,
                                 UART_PIN_NO_CHANGE, GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT,
                                        UART_RX_RING_BUF, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART%d init OK - RX=GPIO%d - %d baud",
             GPS_UART_PORT, GPS_RX_GPIO, GPS_BAUD_RATE);
}

#ifdef DEBUG_TASK
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
        ESP_LOGI("DEBUG", "Time (GMT+7): %02d:%02d:%02d.%03d",
                 lt.hour, lt.minute, lt.second, lt.millisecond);
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
/*  GPS Task                                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

static void gps_task(void *arg)
{
    nmea_parser_t parser;
    nmea_parser_init(&parser);
    uint32_t last_seq = 0;

    static uint8_t rx_buf[UART_READ_BUF_SZ];

    ESP_LOGI(TAG, "GPS task started – waiting for NMEA data...");

    while (1)
    {
        int rx_len = uart_read_bytes(
            GPS_UART_PORT,
            rx_buf,
            sizeof(rx_buf),
            pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));

        if (rx_len > 0)
        {
            nmea_parser_feed(&parser, rx_buf, (size_t)rx_len);

            if (parser.data.seq != last_seq)
            {
                // publish latest GPS data (overwrite queue)
                uint8_t next = g_gps.index ^ 1; // flip 0<->1
                g_gps.buf[next] = parser.data;  // copy struct
                // đảm bảo data write xong trước khi publish index
                __atomic_store_n(&g_gps.index, next, __ATOMIC_RELEASE);

                // notify consumers
                xTaskNotify(rtc_task_handle, EVT_GPS_UPDATE, eSetBits);
                xTaskNotify(ui_task_handle, EVT_GPS_UPDATE, eSetBits);
#ifdef DEBUG_TASK
                xTaskNotify(dbg_task_handle, EVT_GPS_UPDATE, eSetBits);

#endif

                last_seq = parser.data.seq;
            }
        }
    }
}

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
        {
            continue;
        }
        last_seq = d.seq;
        /*
         * Kiểm tra có cần sync RTC không:
         *   - Lần đầu: ngay khi GPS báo fix hợp lệ
         *   - Sau đó: mỗi GPS_RTC_SYNC_INTERVAL_S (10 phút)
         *
         * Không sync liên tục ở 10Hz – CPU overhead không cần thiết
         * và RTC của ESP32 đủ chính xác trong khoảng 10 phút.
         */
        if (gps_rtc_should_sync(&d))
        {
            gps_rtc_sync(&d);
            // notify UI & debug có sync
            xTaskNotify(ui_task_handle, EVT_RTC_SYNC, eSetBits);
#ifdef DEBUG_TASK
            xTaskNotify(dbg_task_handle, EVT_RTC_SYNC, eSetBits);
#endif
        }
    }
}

static void ui_task(void *arg)
{
    ESP_LOGI(TAG, "UI task started");
    uint32_t events;
    gps_data_t d;
    uint32_t last_seq = 0;

    while (1)
    {
        // wait notify hoặc timeout 1s (force render new frame)
        xTaskNotifyWait(
            0,          // don't clear on entry
            0xFFFFFFFF, // clear all bits on exit
            &events,
            pdMS_TO_TICKS(1000));

        if (events & EVT_GPS_UPDATE)
        {
            gps_read_latest(&d);

            // nếu có data mới → update full
            if (d.seq != last_seq)
            {
                last_seq = d.seq;

                // ui_update(d.speed_kmh, d.sats, d.fix, d.utc_time);
            }
            else
            {
                // không có data mới → vẫn refresh (ví dụ clock)
                // ui_update_time_only(d.utc_time);
            }
        }

        if (events & EVT_RTC_SYNC)
        {
            // show RTC Sync icon
        }
    }
}

#ifdef DEBUG_TASK
static void debug_task(void *arg)
{
    ESP_LOGI("DEBUG", "Start debug task");
    uint32_t events;
    gps_data_t d;
    uint32_t last_seq = 0;

    while (1)
    {
        xTaskNotifyWait(
            0,          // don't clear on entry
            0xFFFFFFFF, // clear all bits on exit
            &events,
            portMAX_DELAY);
        if (events & EVT_GPS_UPDATE)
        {
            gps_read_latest(&d);
            if (d.seq == last_seq)
            {
                continue;
            }
            last_seq = d.seq;
            print_gps_data(&d);
        }
        if (events & EVT_RTC_SYNC)
        {
            ESP_LOGI("DEBUG", "Sync RTC Time");
        }
    }
}
#endif

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Entry point                                                                  */
/* ─────────────────────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "=== GPS HUD - ATGM336H / ESP32 ===");
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());
    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  Turn off onboard LED                                                       */
    /* ─────────────────────────────────────────────────────────────────────────── */

    ESP_LOGI(TAG, "Turn off onboard LED");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT) | (1ULL << LED_RED) | (1ULL << LED_BLUE) | (1ULL << LED_RED)};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    // onboard LED is inverted level ~ 1 = off / 0 = on
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_BLUE, 1);
    gpio_set_level(LED_GREEN, 1);
    ESP_LOGI(TAG, "Turn off LCD");
    gpio_set_level(PIN_NUM_BK_LIGHT, 0);

    /*
     * Bước 1: Init timezone GMT+7.
     * PHẢI gọi trước mọi hàm time() / localtime() / gettimeofday()
     * để tzset() có hiệu lực sớm nhất.
     */
    gps_rtc_init();

    /* Bước 2: Khởi tạo UART */
    gps_uart_init();

    /* Bước 3: Tạo task*/
    xTaskCreatePinnedToCore(
        gps_task,
        "gps_task",
        3072,
        NULL,
        5,
        &gps_task_handle,
        1 /* Core 1 – tránh xung đột WiFi/BT nếu thêm sau */
    );

    xTaskCreatePinnedToCore(
        ui_task,
        "ui_task",
        8192,
        NULL,
        5,
        &ui_task_handle,
        1 /* Core 1 – tránh xung đột WiFi/BT nếu thêm sau */
    );

    xTaskCreatePinnedToCore(
        rtc_sync_task,
        "rtc_sync_task",
        2048,
        NULL,
        5,
        &rtc_task_handle,
        1 /* Core 1 – tránh xung đột WiFi/BT nếu thêm sau */
    );

#ifdef DEBUG_TASK
    xTaskCreatePinnedToCore(
        debug_task,
        "debug_task",
        2048,
        NULL,
        5,
        &dbg_task_handle,
        1 /* Core 1 – tránh xung đột WiFi/BT nếu thêm sau */
    );
#endif
}