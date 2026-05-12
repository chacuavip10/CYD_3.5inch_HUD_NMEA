/**
 * @file gps_nmea.c
 * @brief NMEA FSM parser implementation – handles GGA and RMC sentences.
 *
 * Field parsing strategy:
 *   - Uses a pointer walking directly through the buffer; avoids strtok
 *     (not thread-safe and causes side-effects).
 *   - Internal utility functions parse integers / floats without sscanf/atof
 *     to reduce overhead and stack usage on ESP32.
 *
 * NMEA sentence structure:
 *   $<talker><type>,<f0>,<f1>,...,<fN>*<CKHI><CKLO>\r\n
 *   Checksum = XOR of all bytes between '$' and '*' (exclusive).
 */

#include "gps_nmea.h"

#include "esp_timer.h" /* esp_timer_get_time for stats */
#include <math.h>      /* floor  */
#include <string.h>    /* memset */

typedef enum {
  NMEA_TYPE_GGA = 0,
  NMEA_TYPE_RMC = 1,
  NMEA_TYPE_OTHER = 2
} nmea_sentence_type_t;

/* ───────────────────────────────────────────────────────────────────────── */
/*  Internal utility macros                                                  */
/* ───────────────────────────────────────────────────────────────────────── */

/** Check whether a character is a decimal digit. */
#define IS_DIGIT(c) ((c) >= '0' && (c) <= '9')

/** Convert a hex character (0-9, A-F, a-f) to its nibble value. Returns 0 for
 * invalid input. */
static inline uint8_t hex_to_nibble(char c) {
  if (c >= '0' && c <= '9')
    return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F')
    return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f')
    return (uint8_t)(c - 'a' + 10);
  return 0;
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  Internal number parsers – no stdlib floating point                       */
/* ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Parse an unsigned integer from a string, stopping at the first
 * non-digit character.
 *
 * @param s    Input string pointer.
 * @param out  Output result.
 * @return     Number of characters consumed.
 */
static int parse_uint(const char *s, uint32_t *out) {
  uint32_t val = 0;
  int n = 0;
  while (IS_DIGIT(s[n])) {
    val = val * 10u + (uint32_t)(s[n] - '0');
    n++;
  }
  *out = val;
  return n;
}

/**
 * @brief Parse an unsigned floating-point number (e.g. "123.456") from a
 * string.
 *
 * @param s    Input string pointer.
 * @param out  Output result as a double.
 * @return     Number of characters consumed.
 */
static int parse_double(const char *s, double *out) {
  uint32_t int_part = 0;
  int n = 0;
  n += parse_uint(s, &int_part);
  double val = (double)int_part;

  if (s[n] == '.') {
    n++; /* skip '.' */
    double scale = 0.1;
    while (IS_DIGIT(s[n])) {
      val += (double)(s[n] - '0') * scale;
      scale *= 0.1;
      n++;
    }
  }
  *out = val;
  return n;
}

/**
 * @brief Parse a floating-point number, returning a float.
 */
static int parse_float(const char *s, float *out) {
  double d = 0.0;
  int n = parse_double(s, &d);
  *out = (float)d;
  return n;
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  NMEA field access utilities                                              */
/* ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Return a pointer to the Nth field (0-based) in a null-terminated NMEA
 * sentence.
 *
 * Field 0 is the sentence name (e.g. "GNGGA"), field 1 is the first parameter,
 * etc. Does not copy data – returns a direct pointer into the buffer.
 *
 * @param sentence  NMEA sentence string without the leading '$',
 * null-terminated.
 * @param field_n   Zero-based index of the field to retrieve.
 * @return          Pointer to the field, or NULL if there are not enough
 * fields.
 */
static const char *field_ptr(const char *sentence, int field_n) {
  const char *p = sentence;
  for (int i = 0; i < field_n; i++) {
    /* Find the next ',' */
    while (*p && *p != ',')
      p++;
    if (*p == '\0')
      return NULL; /* not enough fields */
    p++;           /* skip ',' */
  }
  return p;
}

/**
 * @brief Return the length of the field at pointer p (up to ',' or '\0').
 */
static int field_len(const char *p) {
  int n = 0;
  while (p[n] && p[n] != ',')
    n++;
  return n;
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  NMEA coordinate → decimal degrees conversion                             */
/* ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Convert an NMEA coordinate (DDDMM.MMMM format) to decimal degrees.
 *
 * Example: "10523.1234" → degrees=105, minutes=23.1234
 *          → decimal = 105 + 23.1234/60 = 105.38539
 *
 * @param nmea_val  NMEA numeric string.
 * @param dir       Direction character: 'N', 'S', 'E', or 'W'.
 * @return          Decimal degrees; negative for South or West.
 */
static double nmea_to_decimal_deg(const char *nmea_val, char dir) {
  if (!nmea_val || *nmea_val == '\0' || *nmea_val == ',')
    return 0.0;

  double raw = 0.0;
  parse_double(nmea_val, &raw);

  /* NMEA: DDDMM.MMMMM → extract degrees part (integer divide by 100) */
  double degrees = floor(raw / 100.0);
  double minutes = raw - (degrees * 100.0);
  double decimal = degrees + minutes / 60.0;

  /* Apply sign based on direction */
  if (dir == 'S' || dir == 'W')
    decimal = -decimal;

  return decimal;
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  GGA sentence parser                                                      */
/* ───────────────────────────────────────────────────────────────────────── */
/**
 * GGA field layout (field_n 0-based, field 0 = "GPGGA"/"GNGGA"):
 *   1  : UTC time  HHMMSS.sss
 *   2  : Latitude  DDMM.MMMM
 *   3  : N/S
 *   4  : Longitude DDDMM.MMMM
 *   5  : E/W
 *   6  : Fix quality (0=invalid, 1=GPS, 2=DGPS, ...)
 *   7  : Number of satellites
 *   8  : HDOP
 *   9  : Altitude (meters)
 *   10 : 'M'
 *   ... (rest ignored)
 */
static void parse_gga(nmea_parser_t *p, const char *sentence) {
  const char *f;
  gps_data_t *d = &p->data;

  /* ── Field 1: UTC time – HHMMSS.sss ──────────────────────────────── */
  f = field_ptr(sentence, 1);
  // skip parse time, only parse on RMC
  // if (f && field_len(f) >= 6)
  // {
  //     uint32_t v;
  //     /* HH */
  //     d->hour = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
  //     /* MM */
  //     d->minute = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
  //     /* SS */
  //     d->second = (uint8_t)((f[4] - '0') * 10 + (f[5] - '0'));
  //     /* .sss (if present) */
  //     if (f[6] == '.')
  //     {
  //         v = 0;
  //         int digits = parse_uint(f + 7, &v);
  //         /* ATGM336H outputs 3 decimal digits → *100 to get milliseconds */
  //         if (digits == 1)
  //             d->millisecond = (uint16_t)(v * 100);
  //         else if (digits == 2)
  //             d->millisecond = (uint16_t)(v * 10);
  //         else if (digits == 3)
  //             d->millisecond = (uint16_t)v;
  //         else // digits > 3
  //             d->millisecond = 0;
  //     }
  //     else
  //     {
  //         d->millisecond = 0;
  //     }
  // }

  /* ── Fields 2–3: Latitude + direction ────────────────────────────── */
  f = field_ptr(sentence, 2);
  // const char *f3 = field_ptr(sentence, 3);
  // if (f && f3 && field_len(f) > 0 && field_len(f3) > 0) {
  //   d->latitude = nmea_to_decimal_deg(f, *f3);
  // }

  /* ── Fields 4–5: Longitude + direction ───────────────────────────── */
  f = field_ptr(sentence, 4);
  // const char *f5 = field_ptr(sentence, 5);
  // if (f && f5 && field_len(f) > 0 && field_len(f5) > 0) {
  //   d->longitude = nmea_to_decimal_deg(f, *f5);
  // }

  /* ── Field 7: Number of satellites ───────────────────────────────── */
  f = field_ptr(sentence, 7);
  if (f && field_len(f) > 0) {
    uint32_t sats = 0;
    parse_uint(f, &sats);
    d->satellites = (uint8_t)sats;
  }

  /* ── Field 8: HDOP ────────────────────────────────────────────────── */
  f = field_ptr(sentence, 8);
  if (f && field_len(f) > 0) {
    parse_float(f, &d->hdop);
  }

  /* ── Field 9: Altitude (meters) ───────────────────────────────────── */
  f = field_ptr(sentence, 9);
  if (f && field_len(f) > 0) {
    parse_float(f, &d->altitude_m);
  }

  /* Mark new data as available */
  p->gga_updated = true;
  // only set updated if both GGA and RMC have been received
  if (p->rmc_updated) {
    d->seq++; // increment version

    // reset cycle
    p->gga_updated = false;
    p->rmc_updated = false;
  }
}

/* ───────────────────────────────────────────────────────────────────────────
 */
/*  RMC sentence parser */
/* ───────────────────────────────────────────────────────────────────────────
 */
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
 *   ... (rest ignored)
 */
static void parse_rmc(nmea_parser_t *p, const char *sentence) {
  const char *f;
  gps_data_t *d = &p->data;

  /* ── Field 1: UTC time ───────────────────────────────────────────── */
  f = field_ptr(sentence, 1);
  if (f && field_len(f) >= 6) {
    d->hour = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
    d->minute = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
    d->second = (uint8_t)((f[4] - '0') * 10 + (f[5] - '0'));
    if (f[6] == '.') {
      uint32_t v = 0;
      int digits = parse_uint(f + 7, &v);
      /* ATGM336H outputs 3 decimal digits → multiply to get milliseconds */
      if (digits == 1)
        d->millisecond = (uint16_t)(v * 100);
      else if (digits == 2)
        d->millisecond = (uint16_t)(v * 10);
      else if (digits == 3)
        d->millisecond = (uint16_t)v;
      else // digits > 3
        d->millisecond = 0;
    } else {
      d->millisecond = 0;
    }
  }

  /* ── Field 2: Fix status ─────────────────────────────────────────── */
  f = field_ptr(sentence, 2);
  d->valid = (f && *f == 'A');

  /* ── Fields 3–4: Latitude + direction ────────────────────────────── */
  f = field_ptr(sentence, 3);
  const char *f4 = field_ptr(sentence, 4);
  if (f && f4 && field_len(f) > 0 && field_len(f4) > 0) {
    d->latitude = nmea_to_decimal_deg(f, *f4);
  }

  /* ── Fields 5–6: Longitude + direction ───────────────────────────── */
  f = field_ptr(sentence, 5);
  const char *f6 = field_ptr(sentence, 6);
  if (f && f6 && field_len(f) > 0 && field_len(f6) > 0) {
    d->longitude = nmea_to_decimal_deg(f, *f6);
  }

  /* ── Field 7: Speed (knots) → km/h ──────────────────────────────── */
  f = field_ptr(sentence, 7);
  if (f && field_len(f) > 0) {
    float knots = 0.0f;
    parse_float(f, &knots);
    d->speed_kmh = knots * 1.852f; /* 1 knot = 1.852 km/h */
  }

  /* ── Field 8: Course over ground ─────────────────────────────────── */
  f = field_ptr(sentence, 8);
  if (f && field_len(f) > 0) {
    parse_float(f, &d->course_deg);
  }

  /* ── Field 9: Date DDMMYY ────────────────────────────────────────── */
  f = field_ptr(sentence, 9);
  if (f && field_len(f) == 6) {
    d->day = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
    d->month = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
    /* YY → YYYY: assume 21st century (2000+YY) */
    d->year = (uint16_t)(2000u + (f[4] - '0') * 10u + (f[5] - '0'));
  }

  p->rmc_updated = true;

  if (p->gga_updated) {
    d->seq++; // increment version

    // reset cycle
    p->gga_updated = false;
    p->rmc_updated = false;
  }
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Message rate statistics                                                   */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Update message rate statistics (called for each valid sentence).
 *
 * @param p      Parser context.
 * @param type   0 = total, 1 = GGA, 2 = RMC
 */
static void nmea_update_stats(nmea_parser_t *p, nmea_sentence_type_t type) {
  int64_t now_us = esp_timer_get_time();
  float delta_sec = (float)(now_us - p->last_stats_time_us) / 1000000.0f;

  /* Increment counter for the sentence type */
  p->stats.total_sentences++;
  switch (type) {
  case NMEA_TYPE_GGA:
    p->stats.gga_sentences++;
    break;
  case NMEA_TYPE_RMC:
    p->stats.rmc_sentences++;
    break;
  case NMEA_TYPE_OTHER:
    p->stats.other_sentences++;
    break;
  }

  /* Compute rate if enough time has elapsed (minimum 1 second) */
  if (delta_sec >= 1.0f) {
    p->stats.total_rate_per_sec = p->stats.total_sentences / delta_sec;
    p->stats.gga_rate_per_sec = p->stats.gga_sentences / delta_sec;
    p->stats.rmc_rate_per_sec = p->stats.rmc_sentences / delta_sec;
    p->stats.other_rate_per_sec = p->stats.other_sentences / delta_sec;

    /* Reset counters and timestamp */
    p->stats.total_sentences = 0;
    p->stats.gga_sentences = 0;
    p->stats.rmc_sentences = 0;
    p->stats.other_sentences = 0;
    p->last_stats_time_us = now_us;
  }
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  Dispatcher: identify sentence type and invoke the corresponding parser   */
/* ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Compare the prefix of an NMEA sentence against a pattern
 * (case-sensitive, 5 chars).
 *
 * The buffer starts with a talker ID (GP/GN/GL...) followed by the type
 * (GGA/RMC/...). Two common talker IDs are checked for compatibility with both
 * old and new ATGM336H modules.
 *
 * @param buf   NMEA sentence buffer (without leading '$').
 * @param type  3-character string: "GGA" or "RMC".
 * @return      true if the sentence matches.
 */
static bool sentence_is(const char *buf, const char *type) {
  /* Talker may be GP (GPS only), GN (GNSS multi-constellation), or GL (GLONASS)
   */
  return ((buf[0] == 'G') &&
          (buf[1] == 'P' || buf[1] == 'N' || buf[1] == 'L') &&
          buf[2] == type[0] && buf[3] == type[1] && buf[4] == type[2]);
}

/**
 * @brief Process a complete NMEA sentence that has passed checksum validation.
 *
 * Called by the FSM when the checksum is valid.
 *
 * @param p  Parser context.
 */
static void dispatch_sentence(nmea_parser_t *p) {
  const char *buf = p->buf;

  if (sentence_is(buf, "GGA")) {
    parse_gga(p, buf);
    nmea_update_stats(p, NMEA_TYPE_GGA); // count and compute rate for GGA
  } else if (sentence_is(buf, "RMC")) {
    parse_rmc(p, buf);
    nmea_update_stats(p, NMEA_TYPE_RMC); // count and compute rate for RMC
  } else {
    nmea_update_stats(
        p, NMEA_TYPE_OTHER); // count other sentences (GSV, GSA, VTG, ...)
    /* Other sentence types are ignored and not parsed */
  }
}

/* ───────────────────────────────────────────────────────────────────────── */
/*  Public API                                                               */
/* ───────────────────────────────────────────────────────────────────────── */

void nmea_parser_init(nmea_parser_t *p) {
  memset(p, 0, sizeof(*p));
  p->state = NMEA_STATE_IDLE;
  p->last_stats_time_us = esp_timer_get_time(); /* Initialize stats timestamp */
  memset(&p->stats, 0, sizeof(p->stats));
}

/**
 * @brief Feed a block of bytes into the FSM – the core of the parser.
 *
 * The FSM processes each byte, accumulates the sentence into a buffer,
 * verifies the checksum, then calls dispatch_sentence() when a complete
 * and valid sentence is received.
 *
 * Note: This function does not allocate memory and is safe to call from a UART
 * task.
 */
void nmea_parser_feed(nmea_parser_t *p, const uint8_t *raw, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t c = raw[i];

    switch (p->state) {

    /* ── Wait for '$' to start a sentence ────────────────────────── */
    case NMEA_STATE_IDLE:
      if (c == '$') {
        p->buf_idx = 0;
        p->cksum_calc = 0;
        p->state = NMEA_STATE_SENTENCE;
      }
      break;

    /* ── Read sentence body and accumulate XOR checksum ──────────── */
    case NMEA_STATE_SENTENCE:
      if (c == '*') {
        /* End of data field; begin reading the checksum */
        p->buf[p->buf_idx] = '\0'; /* null-terminate buffer */
        p->cksum_recv = 0;
        p->state = NMEA_STATE_CKSUM_1;
      } else if (c == '\r' || c == '\n') {
        /* Sentence terminated prematurely (no '*') → discard */
        p->state = NMEA_STATE_IDLE;
      } else if (p->buf_idx >= NMEA_MAX_SENTENCE_LEN - 1u) {
        /* Buffer overflow → sentence too long, discard */
        p->state = NMEA_STATE_IDLE;
      } else {
        /* Normal byte: write to buffer AND XOR into checksum */
        p->cksum_calc ^= c;
        p->buf[p->buf_idx++] = (char)c;
      }
      break;

    /* ── High nibble of checksum (hex digit) ─────────────────────── */
    case NMEA_STATE_CKSUM_1:
      p->cksum_recv = (uint8_t)(hex_to_nibble((char)c) << 4);
      p->state = NMEA_STATE_CKSUM_2;
      break;

    /* ── Low nibble → verify checksum and dispatch ────────────────── */
    case NMEA_STATE_CKSUM_2:
      p->cksum_recv |= hex_to_nibble((char)c);

      if (p->cksum_recv == p->cksum_calc) {
        /* Checksum match → valid sentence, proceed to parse */
        dispatch_sentence(p);
      }
      /* Regardless of checksum result, return to IDLE for the next sentence */
      p->state = NMEA_STATE_IDLE;
      break;

    default:
      p->state = NMEA_STATE_IDLE;
      break;
    }
  }
}

void nmea_parser_get_stats(nmea_parser_t *p, nmea_stats_t *out) {
  if (p && out) {
    *out = p->stats;
  }
}

void nmea_parser_reset_stats(nmea_parser_t *p) {
  if (p) {
    memset(&p->stats, 0, sizeof(p->stats));
    p->last_stats_time_us = esp_timer_get_time();
  }
}