/**
 * @file gps_nmea.h
 * @brief NMEA 0183 parser for the ATGM336H module – ESP-IDF 5.x / 6.x
 *
 * Handles two sentence types only:
 *   - GGA : lat/lon, satellite count, HDOP, altitude
 *   - RMC : lat/lon, speed, course, date/time, fix status
 *
 * Supports both the GP (single-constellation GPS) and GN (multi-constellation) prefixes.
 *
 * A byte-by-byte FSM algorithm → no large buffers, no malloc, no strtok.
 * XOR checksum is verified before parsing.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Configuration constants                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

/** Maximum length of one NMEA sentence (excluding '$' and '*XX\r\n'). */
#define NMEA_MAX_SENTENCE_LEN 96u

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  Message rate statistics struct                                             */
    /* ─────────────────────────────────────────────────────────────────────────── */

    typedef struct
    {
        uint32_t total_sentences; // Total valid NMEA sentences (all types)
        uint32_t gga_sentences;   // Number of GGA sentences
        uint32_t rmc_sentences;   // Number of RMC sentences
        uint32_t other_sentences; // Number of other sentences (GSV, GSA, GLL, VTG, ZDA, TXT, ...)

        float total_rate_per_sec; // Overall rate (sentences/second)
        float gga_rate_per_sec;   // GGA rate (sentences/second)
        float rmc_rate_per_sec;   // RMC rate (sentences/second)
        float other_rate_per_sec; // Other sentence rate (sentences/second)
    } nmea_stats_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  FSM state enum                                                             */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief FSM parser states.
     *
     * State diagram:
     *   IDLE ──'$'──> SENTENCE ──'*'──> CKSUM_1 ──hex──> CKSUM_2 ──> IDLE
     *                    │ (overflow/\r\n)                              ↑
     *                    └──────────────────────────────────────────────┘
     */
    typedef enum
    {
        NMEA_STATE_IDLE = 0, /**< Waiting for the '$' start character       */
        NMEA_STATE_SENTENCE, /**< Reading sentence body, accumulating XOR   */
        NMEA_STATE_CKSUM_1,  /**< Reading high nibble of checksum (hex)     */
        NMEA_STATE_CKSUM_2,  /**< Reading low nibble of checksum (hex)      */
    } nmea_state_t;

    typedef enum
    {
        SIG_NOSIGNAL = 0,
        SIG_BAD,
        SIG_MODERATE,
        SIG_GOOD,
        SIG_EXCELLENT
    } signal_level_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  GPS data struct – all parsed information is stored here                    */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief All GPS information extracted from GGA and RMC sentences.
     *
     * `valid` is true when RMC reports status 'A' (Active / fix acquired).
     * `seq` is incremented each time a complete GGA+RMC pair is successfully parsed.
     */
    typedef struct
    {
        /* ── UTC time (from RMC) ──────────────────────────────────────────────── */
        uint8_t hour;         /**< UTC hour   [0–23]                    */
        uint8_t minute;       /**< Minute     [0–59]                    */
        uint8_t second;       /**< Second     [0–59]                    */
        uint16_t millisecond; /**< Millisecond [0–999]                  */

        /* ── UTC date (from RMC) ─────────────────────────────────────────────── */
        uint8_t day;   /**< Day   [1–31]                                */
        uint8_t month; /**< Month [1–12]                                */
        uint16_t year; /**< Full year, e.g. 2025                        */

        /* ── Position ────────────────────────────────────────────────────────── */
        double latitude;  /**< Decimal latitude,  + = North, – = South  */
        double longitude; /**< Decimal longitude, + = East,  – = West   */

        /* ── Signal quality (from GGA) ───────────────────────────────────────── */
        uint8_t satellites; /**< Number of satellites in use            */
        float hdop;         /**< Horizontal Dilution of Precision       */
        signal_level_t signal_level;
        float altitude_m; /**< Altitude above mean sea level (meters)   */

        /* ── Motion (from RMC) ───────────────────────────────────────────────── */
        float speed_kmh;  /**< Speed in km/h (converted from knots)     */
        float course_deg; /**< Course over ground (degrees, 0–360, 0=N) */

        /* ── Odometry ────────────────────────────────────────────────────────── */
        double odometer_m; /**< Cumulative distance travelled (meters)  */
        bool is_moving;    /**< true when speed exceeds 0.8 km/h        */

        /* ── Status ──────────────────────────────────────────────────────────── */
        bool valid;         /**< true = valid fix (RMC status == 'A')   */
        bool valid_changed; // Flag indicating the fix status changed in this frame
        uint32_t seq;       // Version counter; incremented on each new GGA+RMC pair
    } gps_data_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  FSM parser context                                                         */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief NMEA parser context – each UART port should have its own instance.
     *
     * Initialize with nmea_parser_init() before use.
     */
    typedef struct
    {
        nmea_state_t state;              /**< Current FSM state                   */
        char buf[NMEA_MAX_SENTENCE_LEN]; /**< Sentence accumulation buffer        */
        uint8_t buf_idx;                 /**< Write index into buf                */
        uint8_t cksum_calc;              /**< Running XOR checksum                */
        uint8_t cksum_recv;              /**< Checksum received from the sentence */
        gps_data_t data;                 /**< Most recently parsed GPS data       */
        bool gga_updated;
        bool rmc_updated;
        /* ── Message rate statistics ─────────────────────────────────────- */
        nmea_stats_t stats;         /**< NMEA statistics                     */
        int64_t last_stats_time_us; /**< Timestamp of last statistics update */
    } nmea_parser_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  Public API                                                                 */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Initialize the parser to its default state, clearing all data.
     * @param p  Pointer to an nmea_parser_t instance.
     */
    void nmea_parser_init(nmea_parser_t *p);

    /**
     * @brief Feed a raw byte block from UART into the FSM.
     *
     * May be called repeatedly with arbitrary chunk sizes (no sentence
     * boundary alignment required). When a complete sentence with a valid
     * checksum is received, p->data is updated and p->data.seq is incremented.
     *
     * @param p    Pointer to the parser context.
     * @param raw  Pointer to the raw data block.
     * @param len  Number of bytes in the block.
     */
    void nmea_parser_feed(nmea_parser_t *p, const uint8_t *raw, size_t len);

    /**
     * @brief Retrieve NMEA message rate statistics.
     *
     * @param p    Pointer to the parser context.
     * @param out  Pointer to an nmea_stats_t struct to receive the data.
     */
    void nmea_parser_get_stats(nmea_parser_t *p, nmea_stats_t *out);

    /**
     * @brief Reset NMEA message rate statistics.
     *
     * @param p  Pointer to the parser context.
     */
    void nmea_parser_reset_stats(nmea_parser_t *p);

#ifdef __cplusplus
}
#endif