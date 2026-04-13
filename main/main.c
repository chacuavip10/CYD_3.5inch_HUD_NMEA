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
/*  Cấu hình phần cứng                                                         */
/* ─────────────────────────────────────────────────────────────────────────── */

#define GPS_UART_PORT UART_NUM_2
#define GPS_RX_GPIO GPIO_NUM_21
#define GPS_BAUD_RATE 115200
#define UART_RX_RING_BUF 2048
#define GPS_TASK_STACK_SZ 3072
#define GPS_TASK_PRIORITY 5
#define UART_READ_BUF_SZ 256
#define UART_READ_TIMEOUT_MS 10

static const char *TAG = "GPS";

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
        ESP_LOGI(TAG, "Time (GMT+7): %02d:%02d:%02d.%03d",
                 lt.hour, lt.minute, lt.second, lt.millisecond);
        ESP_LOGI(TAG, "Date (GMT+7): %02d/%02d/%04d",
                 lt.day, lt.month, lt.year);
    }
    else
    {
        /*
         * RTC chưa được sync lần nào (chờ GPS fix lần đầu).
         * Hiển thị UTC thô từ GPS làm fallback tạm thời.
         */
        ESP_LOGI(TAG, "Time (UTC) : %02d:%02d:%02d  [waiting for RTC sync]",
                 gps->hour, gps->minute, gps->second);
        ESP_LOGI(TAG, "Date (UTC) : %02d/%02d/%04d",
                 gps->day, gps->month, gps->year);
    }
    if (gps->valid)
    {
        /* ── Trạng thái fix ──────────────────────────────────────────────── */
        ESP_LOGI(TAG, "Fix valid  : YES");

        /* ── Vị trí ──────────────────────────────────────────────────────── */
        ESP_LOGI(TAG, "Latitude   : %.7f %s",
                 gps->latitude >= 0 ? gps->latitude : -gps->latitude,
                 gps->latitude >= 0 ? "N" : "S");
        ESP_LOGI(TAG, "Longitude  : %.7f %s",
                 gps->longitude >= 0 ? gps->longitude : -gps->longitude,
                 gps->longitude >= 0 ? "E" : "W");

        /* ── Chất lượng tín hiệu ─────────────────────────────────────────── */
        ESP_LOGI(TAG, "Satellites : %d", gps->satellites);
        ESP_LOGI(TAG, "HDOP       : %.2f", (double)gps->hdop);
        ESP_LOGI(TAG, "Altitude   : %.1f m", (double)gps->altitude_m);

        /* ── Chuyển động ─────────────────────────────────────────────────── */
        ESP_LOGI(TAG, "Speed      : %.2f km/h", (double)gps->speed_kmh);
        ESP_LOGI(TAG, "Course     : %.1f deg", (double)gps->course_deg);
    }
    else
    {
        ESP_LOGI(TAG, "Fix valid  : NO");
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  GPS Task                                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

static void gps_task(void *arg)
{
    nmea_parser_t parser;
    nmea_parser_init(&parser);

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

            if (parser.data.updated)
            {
                /*
                 * Kiểm tra có cần sync RTC không:
                 *   - Lần đầu: ngay khi GPS báo fix hợp lệ
                 *   - Sau đó: mỗi GPS_RTC_SYNC_INTERVAL_S (10 phút)
                 *
                 * Không sync liên tục ở 10Hz – CPU overhead không cần thiết
                 * và RTC của ESP32 đủ chính xác trong khoảng 10 phút.
                 */
                if (gps_rtc_should_sync(&parser.data))
                {
                    gps_rtc_sync(&parser.data);
                }

                print_gps_data(&parser.data);
                parser.data.updated = false;
            }
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Entry point                                                                  */
/* ─────────────────────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "=== GPS HUD - ATGM336H / ESP32 ===");
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    /*
     * Bước 1: Init timezone GMT+7.
     * PHẢI gọi trước mọi hàm time() / localtime() / gettimeofday()
     * để tzset() có hiệu lực sớm nhất.
     */
    gps_rtc_init();

    /* Bước 2: Khởi tạo UART */
    gps_uart_init();

    /* Bước 3: Tạo GPS task trên Core 1 */
    BaseType_t ret = xTaskCreatePinnedToCore(
        gps_task,
        "gps_task",
        GPS_TASK_STACK_SZ,
        NULL,
        GPS_TASK_PRIORITY,
        NULL,
        1 /* Core 1 – tránh xung đột WiFi/BT nếu thêm sau */
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create GPS task!");
    }
}