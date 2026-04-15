/**
 * @file gps_rtc.h
 * @brief Đồng bộ RTC của ESP32 từ dữ liệu GPS UTC, hiển thị giờ địa phương GMT+7.
 *
 * Chiến lược:
 *   - GPS cung cấp thời gian UTC → lưu nguyên trong gps_data_t (không chỉnh sửa).
 *   - Dùng settimeofday() để nạp UTC vào system clock của ESP32.
 *   - Đặt timezone POSIX "ICT-7" → localtime() tự cộng +7h, xử lý rollover ngày/tháng.
 *   - Sync lần đầu khi GPS có fix hợp lệ, sau đó mỗi GPS_RTC_SYNC_INTERVAL_S giây.
 *   - Hiển thị thời gian bằng gettimeofday() + localtime_r(), KHÔNG đọc từ gps_data_t.
 *
 * Tại sao dùng timegm() thay vì mktime():
 *   mktime() hiểu struct tm là LOCAL time và trừ đi UTC offset khi quy đổi sang time_t.
 *   Nếu TZ = "ICT-7" và bạn truyền vào struct tm UTC, mktime() sẽ trừ thêm 7h → sai.
 *   timegm() (có trong ESP-IDF newlib) luôn hiểu struct tm là UTC → an toàn.
 */

#pragma once

#include <stdbool.h>
#include <time.h>
#include "gps_nmea.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Cấu hình                                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * Chuỗi POSIX timezone cho UTC+7 (Việt Nam / Indochina Time).
 * Cú pháp POSIX: <tên><offset>, offset = số giờ phải CỘNG để ra UTC.
 * UTC+7 → cần trừ 7h để ra UTC → offset = -7 trong POSIX → "ICT-7".
 */
#define GPS_RTC_TZ_STRING "ICT-7"

/**
 * Chu kỳ đồng bộ RTC từ GPS sau lần đầu tiên (giây).
 * Mặc định 10 phút = 600 giây.
 */
#define GPS_RTC_SYNC_INTERVAL_S (15)

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  Struct thời gian địa phương – dùng để hiển thị                             */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Thời gian địa phương (GMT+7) đã chuyển đổi, sẵn sàng hiển thị.
     * Được điền bởi gps_rtc_get_local_time().
     */
    typedef struct
    {
        uint16_t year;  /**< Năm đầy đủ, vd: 2025               */
        uint8_t month;  /**< Tháng [1–12]                        */
        uint8_t day;    /**< Ngày  [1–31]                        */
        uint8_t hour;   /**< Giờ   [0–23], đã cộng +7h          */
        uint8_t minute; /**< Phút  [0–59]                        */
        uint8_t second; /**< Giây  [0–59]                        */
        // uint16_t millisecond; /**< Millisecond [0–999]                 */
        bool valid; /**< false = RTC chưa được sync lần nào */
    } local_time_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  API                                                                         */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Khởi tạo timezone GMT+7 cho system.
     *
     * Phải gọi một lần trong app_main() trước khi dùng các hàm khác.
     * Gọi setenv("TZ", "ICT-7", 1) + tzset() để mọi hàm time C standard
     * (localtime, strftime, ...) đều tự động dùng GMT+7.
     */
    void gps_rtc_init(void);

    /**
     * @brief Kiểm tra có nên đồng bộ RTC từ GPS không.
     *
     * Trả về true nếu:
     *   - GPS có fix hợp lệ (d->valid == true)
     *   - VÀ (chưa sync lần nào, HOẶC đã qua GPS_RTC_SYNC_INTERVAL_S kể từ lần sync cuối)
     *
     * @param d  Dữ liệu GPS hiện tại.
     * @return   true = nên gọi gps_rtc_sync() ngay bây giờ.
     */
    bool gps_rtc_should_sync(const gps_data_t *d);

    /**
     * @brief Thực hiện đồng bộ RTC từ GPS UTC time.
     *
     * Dùng timegm() để convert UTC struct tm → time_t (không bị ảnh hưởng TZ).
     * Sau đó settimeofday() để cập nhật system clock.
     * Ghi nhớ timestamp để tính chu kỳ sync tiếp theo.
     *
     * @param d  Dữ liệu GPS hợp lệ (d->valid phải == true).
     */
    void gps_rtc_sync(const gps_data_t *d);

    /**
     * @brief Lấy thời gian địa phương GMT+7 từ system RTC.
     *
     * Dùng gettimeofday() + localtime_r(). Timezone đã được set về ICT-7
     * trong gps_rtc_init() nên localtime_r() tự động cộng +7h và xử lý
     * rollover ngày/tháng/năm.
     *
     * @param out  Con trỏ tới local_time_t để điền kết quả.
     *             out->valid = false nếu RTC chưa được sync lần nào.
     */
    void gps_rtc_get_local_time(local_time_t *out);

    /**
     * @brief Kiểm tra RTC đã được sync từ GPS ít nhất một lần chưa.
     * @return true = đã sync, thời gian hợp lệ.
     */
    bool gps_rtc_is_synced(void);

#ifdef __cplusplus
}
#endif