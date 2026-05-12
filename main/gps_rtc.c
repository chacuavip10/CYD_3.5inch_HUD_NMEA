/**
 * @file gps_rtc.c
 * @brief RTC synchronization from GPS and local time reading for GMT+7.
 */

#include "gps_rtc.h"

#include <stdlib.h> /* setenv */
#include <string.h>
#include <sys/time.h> /* settimeofday, gettimeofday */

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS_RTC";
static local_time_t cur_time;

/* ───────────────────────────────────────────────────────────────────────── */
/*  Internal helpers                                                         */
/* ───────────────────────────────────────────────────────────────────────── */

/** Returns current boot time in milliseconds. Shared by should_sync and
 *  is_stale to avoid repeating the /1000 pattern. */
static inline int64_t boot_ms(void) {
  return esp_timer_get_time() / 1000; /* µs → ms */
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  Internal state                                                           */
/* ───────────────────────────────────────────────────────────────────────── */

/** Flag indicating that the RTC has been synchronized at least once from GPS.
 */
static bool s_rtc_synced = false;

/**
 * Boot-clock timestamp (ms) of the most recent RTC sync.
 * Used by gps_rtc_should_sync() and gps_rtc_is_stale().
 * 0 means no sync has occurred yet.
 *
 * NOTE: esp_timer_get_time() resets after deep sleep. If deep sleep is used,
 * fall back to comparing UTC epochs via time() in gps_rtc_should_sync().
 */
static int64_t s_last_sync_boot_ms = 0;

/* ───────────────────────────────────────────────────────────────────────── */
/*  Public API                                                               */
/* ───────────────────────────────────────────────────────────────────────── */

void gps_rtc_init(void) {
  /**
   * Set the TZ environment variable for the entire program.
   * "ICT-7" = Indochina Time, UTC+7.
   *
   * POSIX TZ syntax: <timezone_name><offset_hours>
   *   offset = number of hours to ADD to local time to obtain UTC.
   *   UTC+7 (Vietnam) → must subtract 7h to get UTC → offset = -7 → "ICT-7"
   *
   * After tzset(), all standard C time functions (localtime, localtime_r,
   * mktime, strftime with %Z, etc.) automatically use this timezone.
   */
  setenv("TZ", GPS_RTC_TZ_STRING, 1 /* overwrite */);
  tzset();

  ESP_LOGI(TAG, "Timezone set: %s (UTC+7 / Vietnam Standard Time)",
           GPS_RTC_TZ_STRING);
}

bool gps_rtc_should_sync(const gps_data_t *d) {
  if (!d->valid)
    return false;

  if (!s_rtc_synced)
    return true;

  /**
   * Use boot timer instead of time() syscall.
   *
   * time() calls gettimeofday() internally → syscall + epoch conversion.
   * boot_ms() reads esp_timer_get_time() → single register read, ~10 ns.
   *
   * Both measure elapsed time; boot_ms is cheaper and sufficient here
   * since we only need "has GPS_RTC_SYNC_INTERVAL_S seconds passed".
   */
  return (boot_ms() - s_last_sync_boot_ms) >=
         (int64_t)GPS_RTC_SYNC_INTERVAL_S * 1000;
}

void gps_rtc_sync(const gps_data_t *d) {
  /**
   * Step 1: Populate a struct tm from the GPS UTC time.
   *
   * struct tm field conventions:
   *   tm_year = year - 1900   (e.g. 2025 → 125)
   *   tm_mon  = month - 1     (e.g. June → 5)
   *   tm_mday = day [1–31]    (unchanged)
   *   tm_isdst = 0            (GPS UTC has no DST)
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
   * Step 2: Convert UTC struct tm → UTC time_t using timegm().
   *
   * IMPORTANT: Must use timegm(), NOT mktime()!
   *
   * mktime() interprets its input as LOCAL time and subtracts the UTC offset
   * during conversion. With TZ = "ICT-7" (UTC+7), mktime() would subtract 7h:
   *   → GPS UTC 10:00 → mktime() treats as local 10:00 → produces UTC 03:00 →
   * WRONG!
   *
   * timegm() always treats its input as UTC and ignores the TZ variable
   * entirely: → GPS UTC 10:00 → timegm() → UTC epoch 10:00 → CORRECT.
   *
   * timegm() is available in ESP-IDF (newlib) – no external library required.
   */
  time_t utc_epoch = timegm(&utc_tm);

  if (utc_epoch == (time_t)(-1)) {
    ESP_LOGW(TAG, "timegm() failed – invalid GPS data?");
    return;
  }

  /**
   * Step 3: Load the result into the ESP32 system clock.
   *
   * settimeofday() expects UTC time (not local time).
   * The ESP32 system clock always stores UTC internally.
   * localtime() / localtime_r() apply the TZ offset to produce local time.
   */
  struct timeval tv = {
      .tv_sec = utc_epoch,
      .tv_usec = (suseconds_t)(d->millisecond) * 1000L,
  };

  if (settimeofday(&tv, NULL) != 0) {
    ESP_LOGE(TAG, "settimeofday() failed");
    return;
  }

  /* Update internal state */
  s_rtc_synced = true;
  s_last_sync_boot_ms = boot_ms();
  gps_rtc_get_local_time(&cur_time);
  /* Log for debugging */
  ESP_LOGI(TAG, "RTC synced – Local time: %04d-%02d-%02d %02d:%02d:%02d",
           cur_time.year, cur_time.month, cur_time.day, cur_time.hour,
           cur_time.minute, cur_time.second);
}

void gps_rtc_get_local_time(local_time_t *out) {
  /* Not yet synced → report invalid */
  if (!s_rtc_synced) {
    out->valid = false;
    return;
  }

  /**
   * Read the system clock (UTC epoch + microseconds).
   * gettimeofday() always returns UTC, regardless of TZ.
   */
  struct timeval tv;
  gettimeofday(&tv, NULL);

  /**
   * localtime_r() converts a UTC time_t → struct tm according to the current
   * TZ. With TZ = "ICT-7", it automatically:
   *   - Adds 7h to the UTC time
   *   - Handles rollover: 23:00 UTC + 7h = 06:00 the next day
   *   - Updates tm_mday, tm_mon, tm_year when crossing day/month/year
   * boundaries
   *
   * localtime_r() is used instead of localtime() because localtime_r() is
   * thread-safe (it does not use an internal static buffer).
   */
  struct tm local_tm;
  localtime_r(&tv.tv_sec, &local_tm);

  /* Fill our output struct */
  out->year = (uint16_t)(local_tm.tm_year + 1900);
  out->month = (uint8_t)(local_tm.tm_mon + 1);
  out->day = (uint8_t)(local_tm.tm_mday);
  out->week_day = (uint8_t)local_tm.tm_wday;
  out->hour = (uint8_t)(local_tm.tm_hour);
  out->minute = (uint8_t)(local_tm.tm_min);
  out->second = (uint8_t)(local_tm.tm_sec);
  // out->millisecond = (uint16_t)(tv.tv_usec / 1000L);
  out->valid = true;
  out->last_sync_ms = (uint32_t)s_last_sync_boot_ms;
}

bool gps_rtc_is_synced(void) { return s_rtc_synced; }

bool gps_rtc_is_stale(uint32_t threshold_ms) {
  if (!s_rtc_synced)
    return true;
  return (boot_ms() - s_last_sync_boot_ms) >= (int64_t)threshold_ms;
}