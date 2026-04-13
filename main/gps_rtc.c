/**
 * @file gps_rtc.c
 * @brief Triển khai đồng bộ RTC từ GPS và đọc giờ địa phương GMT+7.
 */

#include "gps_rtc.h"

#include <string.h>
#include <sys/time.h> /* settimeofday, gettimeofday */
#include <stdlib.h>   /* setenv */

#include "esp_log.h"

static const char *TAG = "GPS_RTC";

/* ─────────────────────────────────────────────────────────────────────────── */
/*  State nội bộ                                                                */
/* ─────────────────────────────────────────────────────────────────────────── */

/** Cờ đánh dấu đã sync RTC ít nhất một lần từ GPS. */
static bool s_rtc_synced = false;

/**
 * Timestamp (UTC epoch) của lần sync gần nhất.
 * Dùng để tính khoảng cách đến lần sync tiếp theo.
 * Giá trị 0 = chưa sync lần nào.
 */
static time_t s_last_sync_utc = 0;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  API công khai                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

void gps_rtc_init(void)
{
    /**
     * Đặt biến môi trường TZ cho toàn bộ chương trình.
     * "ICT-7" = Indochina Time, UTC+7.
     *
     * Cú pháp POSIX TZ: <tên_timezone><offset_giờ>
     *   offset = số giờ cần CỘNG vào local để ra UTC
     *   UTC+7 (Việt Nam) → cần CỘNG thêm -7 → offset = -7 → "ICT-7"
     *
     * Sau khi tzset(), tất cả hàm C standard (localtime, localtime_r,
     * mktime, strftime với %Z ...) đều dùng timezone này tự động.
     */
    setenv("TZ", GPS_RTC_TZ_STRING, 1 /* overwrite */);
    tzset();

    ESP_LOGI(TAG, "Timezone set: %s (UTC+7 / Giờ Việt Nam)", GPS_RTC_TZ_STRING);
}

bool gps_rtc_should_sync(const gps_data_t *d)
{
    /* Điều kiện tiên quyết: GPS phải có fix hợp lệ */
    if (!d->valid)
    {
        return false;
    }

    /* Lần đầu tiên: sync ngay */
    if (!s_rtc_synced)
    {
        return true;
    }

    /**
     * Các lần tiếp theo: so sánh time_t hiện tại với lần sync cuối.
     * time() trả về UTC epoch, không phụ thuộc timezone → an toàn.
     */
    time_t now_utc;
    time(&now_utc);

    return (now_utc - s_last_sync_utc) >= (time_t)GPS_RTC_SYNC_INTERVAL_S;
}

void gps_rtc_sync(const gps_data_t *d)
{
    /**
     * Bước 1: Điền struct tm từ thời gian UTC của GPS.
     *
     * Lưu ý quy ước của struct tm:
     *   tm_year = năm - 1900   (vd: 2025 → 125)
     *   tm_mon  = tháng - 1    (vd: tháng 6 → 5)
     *   tm_mday = ngày [1–31]  (giữ nguyên)
     *   tm_isdst = 0           (GPS UTC không có DST)
     */
    struct tm utc_tm = {
        .tm_year = (int)d->year - 1900,
        .tm_mon = (int)d->month - 1,
        .tm_mday = (int)d->day,
        .tm_hour = (int)d->hour,
        .tm_min = (int)d->minute,
        .tm_sec = (int)d->second,
        .tm_isdst = 0,
    };

    /**
     * Bước 2: Chuyển UTC struct tm → UTC time_t bằng timegm().
     *
     * QUAN TRỌNG: Phải dùng timegm(), KHÔNG dùng mktime()!
     *
     * mktime() hiểu đầu vào là LOCAL time và trừ UTC offset khi quy đổi.
     * Nếu TZ = "ICT-7" (UTC+7) thì mktime() sẽ trừ thêm 7h:
     *   → GPS UTC 10:00 → mktime() hiểu là local 10:00 → ra UTC 03:00 → SAI!
     *
     * timegm() luôn hiểu đầu vào là UTC, bỏ qua hoàn toàn biến TZ:
     *   → GPS UTC 10:00 → timegm() → UTC epoch 10:00 → ĐÚNG.
     *
     * timegm() có sẵn trong ESP-IDF (newlib) – không cần thư viện ngoài.
     */
    time_t utc_epoch = timegm(&utc_tm);

    if (utc_epoch == (time_t)(-1))
    {
        ESP_LOGW(TAG, "timegm() thất bại – dữ liệu GPS không hợp lệ?");
        return;
    }

    /**
     * Bước 3: Nạp vào system clock của ESP32.
     *
     * settimeofday() nhận UTC time (không phải local time).
     * System clock của ESP32 luôn lưu UTC nội bộ.
     * localtime() / localtime_r() mới áp dụng TZ để ra giờ local.
     */
    struct timeval tv = {
        .tv_sec = utc_epoch,
        .tv_usec = (suseconds_t)(d->millisecond) * 1000L,
    };

    if (settimeofday(&tv, NULL) != 0)
    {
        ESP_LOGE(TAG, "settimeofday() thất bại");
        return;
    }

    /* Cập nhật state */
    s_rtc_synced = true;
    s_last_sync_utc = utc_epoch;

    /* Log để debug */
    ESP_LOGI(TAG, "RTC synced – GPS UTC: %04d-%02d-%02d %02d:%02d:%02d",
             d->year, d->month, d->day,
             d->hour, d->minute, d->second);
}

void gps_rtc_get_local_time(local_time_t *out)
{
    memset(out, 0, sizeof(*out));

    /* Nếu chưa sync lần nào → báo invalid */
    if (!s_rtc_synced)
    {
        out->valid = false;
        return;
    }

    /**
     * Đọc system clock (UTC epoch + microseconds).
     * gettimeofday() luôn trả về UTC, bất kể TZ.
     */
    struct timeval tv;
    gettimeofday(&tv, NULL);

    /**
     * localtime_r() chuyển UTC time_t → struct tm theo TZ hiện tại.
     * Vì TZ = "ICT-7", nó tự động:
     *   - Cộng 7h vào giờ UTC
     *   - Xử lý rollover: 23:00 UTC+7h = 06:00 ngày hôm sau
     *   - Cập nhật tm_mday, tm_mon, tm_year nếu qua ngày/tháng/năm
     *
     * Dùng localtime_r() thay vì localtime() vì localtime_r() thread-safe
     * (không dùng static buffer nội bộ).
     */
    struct tm local_tm;
    localtime_r(&tv.tv_sec, &local_tm);

    /* Điền kết quả ra struct của chúng ta */
    out->year = (uint16_t)(local_tm.tm_year + 1900);
    out->month = (uint8_t)(local_tm.tm_mon + 1);
    out->day = (uint8_t)(local_tm.tm_mday);
    out->hour = (uint8_t)(local_tm.tm_hour);
    out->minute = (uint8_t)(local_tm.tm_min);
    out->second = (uint8_t)(local_tm.tm_sec);
    out->millisecond = (uint16_t)(tv.tv_usec / 1000L);
    out->valid = true;
}

bool gps_rtc_is_synced(void)
{
    return s_rtc_synced;
}