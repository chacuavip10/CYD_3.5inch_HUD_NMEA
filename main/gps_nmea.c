/**
 * @file gps_nmea.c
 * @brief Triển khai NMEA FSM parser – xử lý GGA và RMC.
 *
 * Chiến lược parse field:
 *   - Dùng con trỏ đi thẳng qua buffer, tránh strtok (không thread-safe
 *     và gây side-effect).
 *   - Hàm tiện ích nội bộ parse số nguyên / số thực không dùng sscanf/atof
 *     để giảm overhead và stack usage trên ESP32.
 *
 * Cấu trúc câu NMEA:
 *   $<talker><type>,<f0>,<f1>,...,<fN>*<CKHI><CKLO>\r\n
 *   Checksum = XOR tất cả byte giữa '$' và '*' (không tính 2 ký tự này).
 */

#include "gps_nmea.h"

#include <string.h> /* memset */
#include <math.h>   /* floor  */

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Macro tiện ích nội bộ                                                      */
/* ─────────────────────────────────────────────────────────────────────────── */

/** Kiểm tra ký tự có phải chữ số thập phân không. */
#define IS_DIGIT(c) ((c) >= '0' && (c) <= '9')

/** Chuyển ký tự hex (0-9, A-F, a-f) sang giá trị nibble. Trả 0 nếu sai. */
static inline uint8_t hex_to_nibble(char c)
{
    if (c >= '0' && c <= '9')
        return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F')
        return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f')
        return (uint8_t)(c - 'a' + 10);
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Parser số nội bộ – không dùng stdlib floating point                        */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Parse số nguyên không dấu từ chuỗi, dừng ở ký tự không phải digit.
 *
 * @param s    Con trỏ chuỗi đầu vào.
 * @param out  Kết quả đầu ra.
 * @return     Số ký tự đã tiêu thụ.
 */
static int parse_uint(const char *s, uint32_t *out)
{
    uint32_t val = 0;
    int n = 0;
    while (IS_DIGIT(s[n]))
    {
        val = val * 10u + (uint32_t)(s[n] - '0');
        n++;
    }
    *out = val;
    return n;
}

/**
 * @brief Parse số thực không dấu (vd: "123.456") từ chuỗi.
 *
 * @param s    Con trỏ chuỗi đầu vào.
 * @param out  Kết quả đầu ra dưới dạng double.
 * @return     Số ký tự đã tiêu thụ.
 */
static int parse_double(const char *s, double *out)
{
    uint32_t int_part = 0;
    int n = 0;
    n += parse_uint(s, &int_part);
    double val = (double)int_part;

    if (s[n] == '.')
    {
        n++; /* bỏ dấu '.' */
        double scale = 0.1;
        while (IS_DIGIT(s[n]))
        {
            val += (double)(s[n] - '0') * scale;
            scale *= 0.1;
            n++;
        }
    }
    *out = val;
    return n;
}

/**
 * @brief Parse số thực, trả về float.
 */
static int parse_float(const char *s, float *out)
{
    double d = 0.0;
    int n = parse_double(s, &d);
    *out = (float)d;
    return n;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Tiện ích truy cập field trong câu NMEA                                     */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Trỏ đến field thứ N (0-based) trong câu NMEA đã null-terminated.
 *
 * Field 0 là tên câu (vd: "GNGGA"), field 1 là tham số đầu tiên, v.v.
 * Hàm không sao chép – trả về con trỏ thẳng vào buffer.
 *
 * @param sentence  Chuỗi câu NMEA, không có '$', đã null-terminated.
 * @param field_n   Chỉ số field cần lấy (0-based).
 * @return          Con trỏ tới field, hoặc NULL nếu không đủ field.
 */
static const char *field_ptr(const char *sentence, int field_n)
{
    const char *p = sentence;
    for (int i = 0; i < field_n; i++)
    {
        /* Tìm dấu ',' tiếp theo */
        while (*p && *p != ',')
            p++;
        if (*p == '\0')
            return NULL; /* không đủ field */
        p++;             /* bỏ qua ',' */
    }
    return p;
}

/**
 * @brief Độ dài của field tại con trỏ p (đến ',' hoặc '\0').
 */
static int field_len(const char *p)
{
    int n = 0;
    while (p[n] && p[n] != ',')
        n++;
    return n;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Chuyển đổi tọa độ NMEA → độ thập phân                                     */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Chuyển tọa độ định dạng NMEA (DDDMM.MMMM) sang độ thập phân.
 *
 * Ví dụ: "10523.1234" → degrees=105, minutes=23.1234
 *        → decimal = 105 + 23.1234/60 = 105.38539
 *
 * @param nmea_val  Chuỗi số NMEA.
 * @param dir       Ký tự hướng: 'N','S','E','W'.
 * @return          Tọa độ thập phân, âm nếu S hoặc W.
 */
static double nmea_to_decimal_deg(const char *nmea_val, char dir)
{
    if (!nmea_val || *nmea_val == '\0' || *nmea_val == ',')
        return 0.0;

    double raw = 0.0;
    parse_double(nmea_val, &raw);

    /* NMEA: DDDMM.MMMMM → tách phần degrees (chia nguyên 100) */
    double degrees = floor(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + minutes / 60.0;

    /* Áp dụng dấu theo hướng */
    if (dir == 'S' || dir == 'W')
        decimal = -decimal;

    return decimal;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Parse bản tin GGA                                                           */
/* ─────────────────────────────────────────────────────────────────────────── */
/**
 * GGA field layout (field_n tính từ 0, field 0 = "GPGGA"/"GNGGA"):
 *   1  : UTC time  HHMMSS.sss
 *   2  : Latitude  DDMM.MMMM
 *   3  : N/S
 *   4  : Longitude DDDMM.MMMM
 *   5  : E/W
 *   6  : Fix quality (0=invalid,1=GPS,2=DGPS,...)
 *   7  : Số vệ tinh
 *   8  : HDOP
 *   9  : Altitude (mét)
 *   10 : 'M'
 *   ... (bỏ qua phần còn lại)
 */
static void parse_gga(nmea_parser_t *p, const char *sentence)
{
    const char *f;
    gps_data_t *d = &p->data;

    /* ── Field 1: Thời gian UTC – HHMMSS.sss ──────────────────────────── */
    f = field_ptr(sentence, 1);
    if (f && field_len(f) >= 6)
    {
        uint32_t v;
        /* HH */
        d->hour = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
        /* MM */
        d->minute = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
        /* SS */
        d->second = (uint8_t)((f[4] - '0') * 10 + (f[5] - '0'));
        /* .sss (nếu có) */
        if (f[6] == '.')
        {
            v = 0;
            parse_uint(f + 7, &v);
            /* ATGM336H xuất 2 chữ số thập phân → *10 để ra milliseconds */
            d->millisecond = (uint16_t)(v * 10u);
        }
        else
        {
            d->millisecond = 0;
        }
    }

    /* ── Field 2–3: Vĩ độ + hướng ────────────────────────────────────── */
    f = field_ptr(sentence, 2);
    const char *f3 = field_ptr(sentence, 3);
    if (f && f3 && field_len(f) > 0 && field_len(f3) > 0)
    {
        d->latitude = nmea_to_decimal_deg(f, *f3);
    }

    /* ── Field 4–5: Kinh độ + hướng ──────────────────────────────────── */
    f = field_ptr(sentence, 4);
    const char *f5 = field_ptr(sentence, 5);
    if (f && f5 && field_len(f) > 0 && field_len(f5) > 0)
    {
        d->longitude = nmea_to_decimal_deg(f, *f5);
    }

    /* ── Field 7: Số vệ tinh ──────────────────────────────────────────── */
    f = field_ptr(sentence, 7);
    if (f && field_len(f) > 0)
    {
        uint32_t sats = 0;
        parse_uint(f, &sats);
        d->satellites = (uint8_t)sats;
    }

    /* ── Field 8: HDOP ────────────────────────────────────────────────── */
    f = field_ptr(sentence, 8);
    if (f && field_len(f) > 0)
    {
        parse_float(f, &d->hdop);
    }

    /* ── Field 9: Altitude (mét) ──────────────────────────────────────── */
    f = field_ptr(sentence, 9);
    if (f && field_len(f) > 0)
    {
        parse_float(f, &d->altitude_m);
    }

    /* Đánh dấu đã có dữ liệu mới */
    p->gga_updated = true;
    // chỉ set updated nếu đã có cả 2
    if (p->rmc_updated)
    {
        d->seq++; // tăng version

        // reset cycle
        p->gga_updated = false;
        p->rmc_updated = false;
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Parse bản tin RMC                                                           */
/* ─────────────────────────────────────────────────────────────────────────── */
/**
 * RMC field layout:
 *   1  : UTC time  HHMMSS.sss
 *   2  : Status    A=valid / V=void
 *   3  : Latitude  DDMM.MMMM
 *   4  : N/S
 *   5  : Longitude DDDMM.MMMM
 *   6  : E/W
 *   7  : Speed over ground (knots)
 *   8  : Course over ground (degrees)
 *   9  : Date       DDMMYY
 *   ... (bỏ qua phần còn lại)
 */
static void parse_rmc(nmea_parser_t *p, const char *sentence)
{
    const char *f;
    gps_data_t *d = &p->data;

    /* ── Field 1: Thời gian UTC ──────────────────────────────────────── */
    f = field_ptr(sentence, 1);
    if (f && field_len(f) >= 6)
    {
        d->hour = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
        d->minute = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
        d->second = (uint8_t)((f[4] - '0') * 10 + (f[5] - '0'));
        if (f[6] == '.')
        {
            uint32_t v = 0;
            parse_uint(f + 7, &v);
            d->millisecond = (uint16_t)(v * 10u);
        }
    }

    /* ── Field 2: Trạng thái fix ─────────────────────────────────────── */
    f = field_ptr(sentence, 2);
    d->valid = (f && *f == 'A');

    /* ── Field 3–4: Vĩ độ + hướng ────────────────────────────────────── */
    f = field_ptr(sentence, 3);
    const char *f4 = field_ptr(sentence, 4);
    if (f && f4 && field_len(f) > 0 && field_len(f4) > 0)
    {
        d->latitude = nmea_to_decimal_deg(f, *f4);
    }

    /* ── Field 5–6: Kinh độ + hướng ──────────────────────────────────── */
    f = field_ptr(sentence, 5);
    const char *f6 = field_ptr(sentence, 6);
    if (f && f6 && field_len(f) > 0 && field_len(f6) > 0)
    {
        d->longitude = nmea_to_decimal_deg(f, *f6);
    }

    /* ── Field 7: Tốc độ (knots) → km/h ─────────────────────────────── */
    f = field_ptr(sentence, 7);
    if (f && field_len(f) > 0)
    {
        float knots = 0.0f;
        parse_float(f, &knots);
        d->speed_kmh = knots * 1.852f; /* 1 knot = 1.852 km/h */
    }

    /* ── Field 8: Hướng đi (Course over ground) ──────────────────────── */
    f = field_ptr(sentence, 8);
    if (f && field_len(f) > 0)
    {
        parse_float(f, &d->course_deg);
    }

    /* ── Field 9: Ngày tháng DDMMYY ──────────────────────────────────── */
    f = field_ptr(sentence, 9);
    if (f && field_len(f) == 6)
    {
        d->day = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
        d->month = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
        /* YY → YYYY: giả sử thế kỷ 21 (2000+YY) */
        d->year = (uint16_t)(2000u + (f[4] - '0') * 10u + (f[5] - '0'));
    }

    p->rmc_updated = true;

    if (p->gga_updated)
    {
        d->seq++; // tăng version

        // reset cycle
        p->gga_updated = false;
        p->rmc_updated = false;
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Dispatcher: nhận dạng loại câu và gọi parser tương ứng                    */
/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief So sánh prefix của câu NMEA với chuỗi mẫu (case-sensitive, 5 ký tự).
 *
 * Câu trong buf bắt đầu bằng talker (GP/GN/GL...) + type (GGA/RMC/...).
 * Hàm kiểm tra 2 talker phổ biến để tương thích với cả ATGM336H cũ/mới.
 *
 * @param buf   Buffer câu NMEA (không có '$').
 * @param type  Chuỗi 3 ký tự: "GGA" hoặc "RMC".
 * @return      true nếu khớp.
 */
static bool sentence_is(const char *buf, const char *type)
{
    /* Talker có thể là GP (GPS only) hoặc GN (GNSS multi-const) */
    return ((buf[0] == 'G') &&
            (buf[1] == 'P' || buf[1] == 'N' || buf[1] == 'L') &&
            buf[2] == type[0] &&
            buf[3] == type[1] &&
            buf[4] == type[2]);
}

/**
 * @brief Xử lý một câu NMEA hoàn chỉnh đã qua kiểm tra checksum.
 *
 * Được gọi bởi FSM khi checksum hợp lệ.
 *
 * @param p  Parser context.
 */
static void dispatch_sentence(nmea_parser_t *p)
{
    const char *buf = p->buf;

    if (sentence_is(buf, "GGA"))
    {
        parse_gga(p, buf);
    }
    else if (sentence_is(buf, "RMC"))
    {
        parse_rmc(p, buf);
    }
    /* Các bản tin khác bị bỏ qua (module đã cấu hình chỉ xuất GGA+RMC) */
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  API công khai                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

void nmea_parser_init(nmea_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = NMEA_STATE_IDLE;
}

/**
 * @brief Nạp block byte vào FSM – trái tim của parser.
 *
 * FSM xử lý từng byte, tích lũy câu vào buffer, kiểm tra checksum,
 * sau đó gọi dispatch_sentence() khi câu hoàn chỉnh và hợp lệ.
 *
 * Lưu ý: Hàm này không allocate bộ nhớ, an toàn gọi từ task UART.
 */
void nmea_parser_feed(nmea_parser_t *p, const uint8_t *raw, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        uint8_t c = raw[i];

        switch (p->state)
        {

        /* ── Chờ '$' bắt đầu câu ──────────────────────────────────────── */
        case NMEA_STATE_IDLE:
            if (c == '$')
            {
                p->buf_idx = 0;
                p->cksum_calc = 0;
                p->state = NMEA_STATE_SENTENCE;
            }
            break;

        /* ── Đọc phần thân, tính checksum XOR ────────────────────────── */
        case NMEA_STATE_SENTENCE:
            if (c == '*')
            {
                /* Kết thúc phần data, bắt đầu đọc checksum */
                p->buf[p->buf_idx] = '\0'; /* null-terminate buffer */
                p->cksum_recv = 0;
                p->state = NMEA_STATE_CKSUM_1;
            }
            else if (c == '\r' || c == '\n')
            {
                /* Câu bị cắt giữa chừng (không có '*') → bỏ qua */
                p->state = NMEA_STATE_IDLE;
            }
            else if (p->buf_idx >= NMEA_MAX_SENTENCE_LEN - 1u)
            {
                /* Buffer overflow → câu quá dài, bỏ qua */
                p->state = NMEA_STATE_IDLE;
            }
            else
            {
                /* Byte bình thường: ghi vào buf VÀ XOR vào checksum */
                p->cksum_calc ^= c;
                p->buf[p->buf_idx++] = (char)c;
            }
            break;

        /* ── Nibble cao của checksum (hex) ────────────────────────────── */
        case NMEA_STATE_CKSUM_1:
            p->cksum_recv = (uint8_t)(hex_to_nibble((char)c) << 4);
            p->state = NMEA_STATE_CKSUM_2;
            break;

        /* ── Nibble thấp → kiểm tra và dispatch ──────────────────────── */
        case NMEA_STATE_CKSUM_2:
            p->cksum_recv |= hex_to_nibble((char)c);

            if (p->cksum_recv == p->cksum_calc)
            {
                /* Checksum khớp → câu hợp lệ, tiến hành parse */
                dispatch_sentence(p);
            }
            /* Dù checksum đúng hay sai, reset về IDLE chờ câu tiếp theo */
            p->state = NMEA_STATE_IDLE;
            break;

        default:
            p->state = NMEA_STATE_IDLE;
            break;
        }
    }
}