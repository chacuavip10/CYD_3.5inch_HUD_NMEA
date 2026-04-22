/**
 * @file gps_nmea.h
 * @brief NMEA 0183 parser cho module ATGM336H – ESP-IDF 5.x / 6.x
 *
 * Chỉ xử lý 2 loại bản tin:
 *   - GGA : lat/lon, số vệ tinh, HDOP, độ cao
 *   - RMC : lat/lon, tốc độ, hướng đi, ngày/giờ, trạng thái fix
 *
 * Hỗ trợ cả prefix GP (GPS đơn chòm) và GN (multi-constellation).
 *
 * Thuật toán FSM xử lý từng byte → không cần buffer lớn, không malloc,
 * không strtok. Checksum XOR được kiểm tra trước khi parse.
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
/*  Hằng số cấu hình                                                           */
/* ─────────────────────────────────────────────────────────────────────────── */

/** Độ dài tối đa một câu NMEA (không tính '$' và '*XX\r\n'). */
#define NMEA_MAX_SENTENCE_LEN 96u

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  Struct dữ liệu GPS – tất cả thông tin được lưu tại đây                    */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Toàn bộ thông tin GPS trích xuất từ GGA + RMC.
     *
     * Trường `valid` = true khi RMC báo trạng thái 'A' (Active / có fix).
     * Trường `updated` được set = true mỗi khi parse thành công một bản tin;
     * ứng dụng cần tự clear về false sau khi đọc.
     */
    typedef struct
    {
        /* ── Thời gian UTC (từ GGA hoặc RMC) ─────────────────────────────────── */
        uint8_t hour;         /**< Giờ UTC  [0–23]                            */
        uint8_t minute;       /**< Phút     [0–59]                            */
        uint8_t second;       /**< Giây     [0–59]                            */
        uint16_t millisecond; /**< Phần nghìn giây [0–999]                   */

        /* ── Ngày tháng UTC (từ RMC) ─────────────────────────────────────────── */
        uint8_t day;   /**< Ngày  [1–31]                               */
        uint8_t month; /**< Tháng [1–12]                               */
        uint16_t year; /**< Năm đầy đủ, vd: 2025                      */

        /* ── Vị trí ──────────────────────────────────────────────────────────── */
        double latitude;  /**< Vĩ độ thập phân, + = Bắc, – = Nam        */
        double longitude; /**< Kinh độ thập phân, + = Đông, – = Tây     */

        /* ── Chất lượng tín hiệu (từ GGA) ───────────────────────────────────── */
        uint8_t satellites; /**< Số vệ tinh đang dùng                      */
        float hdop;         /**< Horizontal Dilution of Precision           */
        float altitude_m;   /**< Độ cao so với mực nước biển (mét)         */

        /* ── Chuyển động (từ RMC) ────────────────────────────────────────────── */
        float speed_kmh;  /**< Tốc độ km/h (đã chuyển từ knots)          */
        float course_deg; /**< Hướng đi (độ, 0–360, 0 = Bắc)            */

        /* ── Chuyển động (từ RMC) ────────────────────────────────────────────── */
        double odometer_m; /**< Odometer in met          */

        /* ── Trạng thái ──────────────────────────────────────────────────────── */
        bool valid;   /**< true = fix hợp lệ (RMC status == 'A')    */
        uint32_t seq; // version, tracking dữ liệu mới
    } gps_data_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  FSM parser context                                                          */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Các trạng thái của FSM parser.
     *
     * Sơ đồ:
     *   IDLE ──'$'──> SENTENCE ──'*'──> CKSUM_1 ──hex──> CKSUM_2 ──> IDLE
     *                    │ (overflow/\r\n)                              ↑
     *                    └──────────────────────────────────────────────┘
     */
    typedef enum
    {
        NMEA_STATE_IDLE = 0, /**< Chờ ký tự '$' bắt đầu câu              */
        NMEA_STATE_SENTENCE, /**< Đang đọc phần thân câu, tính checksum  */
        NMEA_STATE_CKSUM_1,  /**< Đọc nibble cao của checksum (hex)      */
        NMEA_STATE_CKSUM_2,  /**< Đọc nibble thấp của checksum (hex)     */
    } nmea_state_t;

    /**
     * @brief Context của NMEA parser – mỗi port UART nên có một instance riêng.
     *
     * Khởi tạo bằng nmea_parser_init() trước khi dùng.
     */
    typedef struct
    {
        nmea_state_t state;              /**< Trạng thái FSM hiện tại    */
        char buf[NMEA_MAX_SENTENCE_LEN]; /**< Buffer tích lũy câu NMEA   */
        uint8_t buf_idx;                 /**< Con trỏ ghi vào buf        */
        uint8_t cksum_calc;              /**< Checksum XOR đang tính     */
        uint8_t cksum_recv;              /**< Checksum nhận được từ câu  */
        gps_data_t data;                 /**< Dữ liệu GPS đã parse       */
        bool gga_updated;
        bool rmc_updated;
    } nmea_parser_t;

    /* ─────────────────────────────────────────────────────────────────────────── */
    /*  API công khai                                                               */
    /* ─────────────────────────────────────────────────────────────────────────── */

    /**
     * @brief Khởi tạo parser về trạng thái ban đầu, xóa toàn bộ dữ liệu.
     * @param p  Con trỏ tới nmea_parser_t.
     */
    void nmea_parser_init(nmea_parser_t *p);

    /**
     * @brief Nạp một block byte thô từ UART vào FSM.
     *
     * Có thể gọi nhiều lần với chunk bất kỳ (không cần cắt theo câu).
     * Khi một câu hoàn chỉnh và checksum hợp lệ được nhận, p->data sẽ
     * được cập nhật và p->data.updated = true.
     *
     * @param p    Con trỏ tới parser context.
     * @param raw  Con trỏ tới block dữ liệu thô.
     * @param len  Số byte trong block.
     */
    void nmea_parser_feed(nmea_parser_t *p, const uint8_t *raw, size_t len);

#ifdef __cplusplus
}
#endif