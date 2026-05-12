/**
 * @file gps_rtc.h
 * @brief Synchronize the ESP32 RTC from GPS UTC data and expose local time at GMT+7.
 *
 * Strategy:
 *   - GPS provides UTC time → stored as-is in gps_data_t (never adjusted).
 *   - settimeofday() loads UTC into the ESP32 system clock.
 *   - POSIX timezone "ICT-7" is set → localtime() automatically adds +7h and
 *     handles day/month rollover.
 *   - First sync occurs when GPS has a valid fix; subsequent syncs every
 *     GPS_RTC_SYNC_INTERVAL_S seconds.
 *   - Time is read via gettimeofday() + localtime_r(), NOT from gps_data_t.
 *
 * Why timegm() instead of mktime():
 *   mktime() interprets struct tm as LOCAL time and subtracts the UTC offset
 *   when converting to time_t. With TZ = "ICT-7", passing a UTC struct tm to
 *   mktime() would subtract an extra 7h → incorrect result.
 *   timegm() (available in ESP-IDF newlib) always interprets struct tm as UTC → safe.
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
/*  Configuration                                                              */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * POSIX timezone string for UTC+7 (Vietnam / Indochina Time).
 * POSIX syntax: <name><offset>, where offset = hours to ADD to local time to get UTC.
 * UTC+7 → subtract 7h to get UTC → POSIX offset = -7 → "ICT-7".
 */
#define GPS_RTC_TZ_STRING "ICT-7"

/**
 * RTC re-sync interval after the initial sync (seconds).
 * Default: 10 minutes = 600 seconds.
 */
#define GPS_RTC_SYNC_INTERVAL_S (10 * 60)

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  Local time struct – used for display                                       */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Converted local time (GMT+7), ready for display.
     * Populated by gps_rtc_get_local_time().
     */
    typedef struct
    {
        uint16_t year;    /**< Full year, e.g. 2025                        */
        uint8_t month;    /**< Month [1–12]                                */
        uint8_t day;      /**< Day   [1–31]                                */
        uint8_t week_day; /**< Day of week: 0=Sunday ... 6=Saturday        */
        uint8_t hour;     /**< Hour  [0–23], already offset by +7h         */
        uint8_t minute;   /**< Minute [0–59]                               */
        uint8_t second;   /**< Second [0–59]                               */
        // uint16_t millisecond; /**< Millisecond [0–999]                  */
        bool valid;            /**< false = RTC has never been synced      */
        uint32_t last_sync_ms; /**< Millis since boot at last RTC sync     */
    } local_time_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  API                                                                        */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Initialize the GMT+7 timezone for the system.
     *
     * Must be called once in app_main() before any other function in this module.
     * Calls setenv("TZ", "ICT-7", 1) + tzset() so that all standard C time
     * functions (localtime, strftime, ...) automatically use GMT+7.
     */
    void gps_rtc_init(void);

    /**
     * @brief Check whether the RTC should be synchronized from GPS.
     *
     * Returns true if:
     *   - GPS has a valid fix (d->valid == true)
     *   - AND (RTC has never been synced, OR GPS_RTC_SYNC_INTERVAL_S seconds
     *     have elapsed since the last sync)
     *
     * @param d  Current GPS data.
     * @return   true = gps_rtc_sync() should be called now.
     */
    bool gps_rtc_should_sync(const gps_data_t *d);

    /**
     * @brief Synchronize the RTC from GPS UTC time.
     *
     * Uses timegm() to convert a UTC struct tm → time_t (unaffected by TZ),
     * then settimeofday() to update the system clock.
     * Records the sync timestamp for computing the next sync interval.
     *
     * @param d  Valid GPS data (d->valid must be true).
     */
    void gps_rtc_sync(const gps_data_t *d);

    /**
     * @brief Get the current local time (GMT+7) from the system RTC.
     *
     * Uses gettimeofday() + localtime_r(). Because the timezone was set to
     * ICT-7 in gps_rtc_init(), localtime_r() automatically adds +7h and
     * handles day/month/year rollover.
     *
     * @param out  Pointer to a local_time_t struct to populate.
     *             out->valid is false if the RTC has never been synced.
     */
    void gps_rtc_get_local_time(local_time_t *out);

    /**
     * @brief Check whether the RTC has been synced from GPS at least once.
     * @return true = synced; time is valid.
     */
    bool gps_rtc_is_synced(void);

    /**
     * @brief Check if the RTC sync is stale.
     *
     * @param threshold_ms Staleness threshold in milliseconds (e.g. 2h = 7200000 ms).
     * @return true  RTC is stale or has never been synchronized.
     * @return false RTC sync is fresh.
     */
    bool gps_rtc_is_stale(uint32_t threshold_ms);

#ifdef __cplusplus
}
#endif