/**
 * ============================================================================
 *  GPS HUD – ESP32 + ATGM336H
 *  ESP-IDF 5.x / 6.x
 *
 *  DATA FLOW:
 *      UART → NMEA FSM → double buffer → notify →
 *          ├── RTC task  (sync system time from GPS)
 *          ├── UI task   (update screen widgets)
 *          └── DEBUG     (optional serial logging)
 *
 *  DESIGN GOALS:
 *      - Lock-free GPS data sharing via atomic double-buffer swap
 *      - LVGL thread-safety via mutex (single writer, single renderer)
 *      - No blocking in ISR or time-critical paths
 *      - Event-driven UI with smooth, frame-rate-aware rendering
 * ============================================================================
 *  *****NMEA message output from ATGM336H
 *  $GNGGA,064939.000,2101.19944,N,10550.49781,E,1,05,7.9,66.8,M,-27.8,M,,*51
 *  $GNGLL,2101.19944,N,10550.49781,E,064939.000,A,A*47
 *  $GNGSA,A,3,14,17,19,30,,,,,,,,,9.6,7.9,5.5,1*38
 *  $GNGSA,A,3,23,,,,,,,,,,,,9.6,7.9,5.5,4*34
 *  $GPGSV,2,1,05,14,36,329,32,17,39,279,34,19,21,258,29,22,,,21,0*51
 *  $GPGSV,2,2,05,30,64,281,40,0*5E
 *  $BDGSV,1,1,01,23,52,332,33,0*41
 *  $GNRMC,064939.000,A,2101.19944,N,10550.49781,E,0.00,0.00,300126,,,A,V*0C
 *  $GNVTG,0.00,T,,M,0.00,N,0.00,K,A*23
 *  $GNZDA,064939.000,30,01,2026,00,00*4D
 *  $GPTXT,01,01,01,ANTENNA OK*35
 * ============================================================================
 *  TODO
 *  1) check welcome sreen timeout because of WDT (add more vtaskdelay)
 */

/* ========================================================================== */
/*                              INCLUDES */
/* ========================================================================== */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_touch_xpt2046.h"
#include "gps_nmea.h"
#include "gps_rtc.h"
#include "ui/ui.h"

/* ========================================================================== */
/*                              CONFIGURATION                                 */
/* ========================================================================== */

#define DEBUG_TASK 0

/* --- GPIO & UART --- */
#define GPS_UART_PORT UART_NUM_2
#define GPS_RX_GPIO GPIO_NUM_21
#define GPS_TX_GPIO GPIO_NUM_22
#define LED_RED GPIO_NUM_4
#define LED_GREEN GPIO_NUM_17
#define LED_BLUE GPIO_NUM_16
#define PIN_NUM_BK_LIGHT GPIO_NUM_27
#define TARGET_GPS_BAUD_RATE 115200
// Ring buffer size for incoming NMEA data
#define UART_RX_RING_BUF 2048
// Single read chunk per UART polling cycle
#define UART_READ_BUF_SZ 1024
// How long to wait for each UART read attempt, should be < 1/2 * rate, gps
// 10hz -> < 50ms. With 40ms read timeout, max GGA+RMC is 15hz, if > 15hz,
// reduce timeout to 10-20 ms
#define UART_READ_TIMEOUT_MS 40
#define GPS_COM_TIMEOUT_MS 2000 // declare signal lost after ? ms
#define GPS_RTC_STALE_THRESHOLD_MS (2 * 60 * 60 * 1000) // 2h

/* --- FreeRTOS event bit flags (used with xTaskNotify) --- */
#define EVT_GPS_UPDATE (1 << 0)       // gps_task → rtc_task, ui_task, dbg_task
#define EVT_RTC_SYNC_DONE (1 << 1)    // rtc_task → ui_task, dbg_task
#define EVT_RTC_SYNC_REQUEST (1 << 2) // gps_task → rtc_task
#define EVT_GPS_TIMEOUT (1 << 3)      // gps_task → ui_task (Module signal lost)
#define EVT_GPS_RESTORED (1 << 4)     // gps_task → ui_task (GPS Packet arrive)
#define EVT_RTC_STALE (1 << 5)        // gps_task → ui_task (stale RTC)
#define EVT_RTC_NOT_STALE (1 << 6)    // gps_task → ui_task (RTC re-sync)
#define EVT_IS_MOVING_TO_MOVE (1 << 7)
#define EVT_IS_MOVING_TO_STOP (1 << 8)
#define EVT_VALID_CHANGE_TO_VALID (1 << 9)
#define EVT_VALID_CHANGE_TO_INVALID (1 << 10)
#define EVT_SIGNAL_CHANGE (1 << 11)
#define EVT_GPS_STATS_UPDATE (1 << 12) // NMEA throughput statistics update

/* --- Task handles (populated at startup) --- */
static TaskHandle_t gps_task_handle;
static TaskHandle_t rtc_task_handle;
static TaskHandle_t ui_task_handle;
static TaskHandle_t lvgl_task_handle;
#if DEBUG_TASK
static TaskHandle_t dbg_task_handle;
#endif

/* ========================================================================== */
/*                              NMEA RATE COUNTER                             */
/* ========================================================================== */

/* Message throughput statistics – displayed only when DEBUG_TASK = 1 */
static nmea_stats_t g_nmea_stats = {0};
static int64_t g_last_stats_display_us = 0;
#define STATS_DISPLAY_INTERVAL_MS 1000 /* Statistics update interval */

/* ========================================================================== */
/*                         EVT_GPS_UPDATE RATE COUNTER                        */
/* ========================================================================== */

/* Statistics for EVT_GPS_UPDATE - displayed only when DEBUG_TASK = 1 */
typedef struct {
  float update_rate_per_sec;  // EVT_GPS_UPDATE frequency per second
  uint32_t total_updates;     // Accumulative update count
  uint32_t last_update_count; // Updates within the last interval
  int64_t last_rate_calc_us;  // Timestamp of the last calculation
} event_rate_stats_t;

/* Double buffer for event rate counters */
typedef struct {
  event_rate_stats_t buf[2];
  volatile uint8_t index;
} event_rate_shared_t;

static event_rate_shared_t g_event_rate_shared = {0};

/* Thread-safe helper to read the latest event rate counters */
static inline void event_rate_read_latest(event_rate_stats_t *out) {
  uint8_t idx1, idx2;
  do {
    idx1 = __atomic_load_n(&g_event_rate_shared.index, __ATOMIC_ACQUIRE);
    *out = g_event_rate_shared.buf[idx1];
    idx2 = __atomic_load_n(&g_event_rate_shared.index, __ATOMIC_ACQUIRE);
  } while (idx1 != idx2);
}

/* ========================================================================== */
/*                              LVGL CONFIG                                   */
/* ========================================================================== */

/*
 * Single mutex guards all LVGL API calls.
 * Only one task (ui_task) ever calls LVGL; the mutex exists as a safety
 * guard when the initialization path (app_main) also touches LVGL before the
 * task is running.
 */
static SemaphoreHandle_t s_lvgl_mutex;
#define LVGL_LOCK() xSemaphoreTake(s_lvgl_mutex, portMAX_DELAY)
#define LVGL_UNLOCK() xSemaphoreGive(s_lvgl_mutex)

/* --- SPI bus for LCD + Touch (shared SPI2) --- */
#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)
#define PIN_NUM_SCLK GPIO_NUM_14
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_LCD_DC GPIO_NUM_2
#define PIN_NUM_LCD_RST GPIO_NUM_NC
#define PIN_NUM_LCD_CS GPIO_NUM_15
#define PIN_NUM_TOUCH_CS GPIO_NUM_33
#define PIN_NUM_TOUCH_INT GPIO_NUM_36

/* --- Physical display resolution --- */
#define LCD_HOR_RES 320
#define LCD_VER_RES 480
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

/*
 * LVGL draw buffer: 100 lines × 320 px × 2 bytes = ~64 KB per buffer.
 * Two buffers enable double-buffered DMA transfers so the CPU can prepare
 * the next tile while the previous one is being clocked out over SPI.
 *
 * RECOMMENDATION: Increasing LVGL_DRAW_BUF_LINES (e.g. to 120) reduces the
 * number of partial-flush calls per frame and can improve perceived smoothness
 * at the cost of extra PSRAM/DRAM usage. For ESP32, max is 100, only increase
 * if has psram
 */
#define LVGL_DRAW_BUF_LINES 100
#define LVGL_TICK_PERIOD_MS 1      // lv_tick_inc() interval via esp_timer ISR
#define LVGL_TASK_MAX_DELAY_MS 500 // Upper bound between lv_timer_handler()
#define LVGL_TASK_MIN_DELAY_MS 10  // Lower bound between lv_timer_handler()
#define LVGL_TASK_STACK_SIZE (8 * 1024)

#define SYNC_SYMBOL "\xEF\x80\xA1"
#define SIG_NONE_SYMBOL "\xEF\x9A\x94"
#define SIG_BAD_SYMBOL "\xEF\x9A\x91"
#define SIG_MORDERATE_SYMBOL "\xEF\x9A\x92"
#define SIG_GOOD_SYMBOL "\xEF\x9A\x93"
#define SIG_EX_SYMBOL "\xEF\x9A\x90"

/* ========================================================================== */
/*                            NMEA CONFIG COMMAND HELPER                      */
/* ========================================================================== */
/* Commands for configuring the ATGM336H module */
static char CFG_BAUDRATE_115200[] = "$PCAS01,5*19\r\n";
static char CFG_UPDATE_RATE_1HZ[] = "$PCAS02,1000*2E\r\n";
static char CFG_UPDATE_RATE_5HZ[] = "$PCAS02,200*1D\r\n";
static char CFG_UPDATE_RATE_10HZ[] = "$PCAS02,100*1E\r\n";
static char CFG_ENABLE_MESSAGE_GGA_RMC[] =
    "$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n";
static char CFG_ENABLE_CONSTELLATION_DUAL_GPS_BEIDOU[] =
    "$PCAS04,13,13,11*29\r\n";
static char CFG_ENABLE_CONSTELLATION_GPS_ONLY[] = "$PCAS04,1,1,1*18\r\n";
static char CFG_ENABLE_CONSTELLATION_BEIDOU_ONLY[] = "$PCAS04,12,12,10*28\r\n";

/**
 * @brief Send NMEA command to GPS module via UART2 TX
 * @param cmd NMEA command string (must include CR/LF termination)
 * @param timeout_ms Timeout in milliseconds for write operation
 * @return true if send successful, false otherwise
 */
static bool gps_send_command(const char *cmd, int timeout_ms) {
  if (!cmd)
    return false;

  int len = strlen(cmd);
  int written = uart_write_bytes(GPS_UART_PORT, cmd, len);

  if (written == len) {
    ESP_LOGI("GPS", "Command sent: %s", cmd);
    /* Flush TX buffer to ensure command is transmitted */
    uart_wait_tx_done(GPS_UART_PORT, pdMS_TO_TICKS(timeout_ms));
    return true;
  } else {
    ESP_LOGE("GPS", "Failed to send command, written %d/%d bytes", written,
             len);
    return false;
  }
}

/* ========================================================================== */
/*                        AUTO BAUDRATE DETECTION                             */
/* ========================================================================== */

/* Common baudrates to probe */
#define BAUD_RATES_TO_TEST {115200, 9600, 38400, 57600, 230400, 460800}
#define NUM_BAUD_RATES 6

/* Probing duration per baudrate (ms) */
#define BAUD_TEST_DURATION_MS 1500

/* Minimum valid NMEA sentences required to confirm correct baudrate */
#define MIN_VALID_SENTENCES 2

/* Cache for current GPS module baudrate */
static int g_current_gps_baudrate = TARGET_GPS_BAUD_RATE;

/**
 * @brief Check if the buffer contains valid NMEA sentences
 * @return true if at least 1 valid NMEA sentence pattern is found
 */
static bool is_valid_nmea_sentence(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len - 6; i++) {
    /* Find '$' character that starts an NMEA sentence */
    if (data[i] == '$') {
      /* Scan for checksum delimiter pattern: $...*XX\r\n */
      size_t j = i + 1;
      while (j < len && data[j] != '*' && j - i < 80) {
        j++;
      }
      if (j < len && data[j] == '*') {
        /* Checksum delimiter found; verify two hex digits follow */
        if (j + 2 < len) {
          char c1 = data[j + 1];
          char c2 = data[j + 2];
          if ((c1 >= '0' && c1 <= '9') || (c1 >= 'A' && c1 <= 'F')) {
            if ((c2 >= '0' && c2 <= '9') || (c2 >= 'A' && c2 <= 'F')) {
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

/**
 * @brief Automatically detect GPS module baudrate
 * @return detected baudrate, or 0 if detection failed
 */
static int detect_gps_baudrate(void) {
  const int baud_rates[] = BAUD_RATES_TO_TEST;
  uint8_t test_buf[512];

  ESP_LOGI("GPS_DETECT", "Starting automatic baudrate detection...");

  for (int i = 0; i < NUM_BAUD_RATES; i++) {
    int current_baud = baud_rates[i];
    LVGL_LOCK();
    lv_label_set_text_fmt(objects.boot_text, "Testing baud: %d", current_baud);
    LVGL_UNLOCK();
    vTaskDelay(100);
    ESP_LOGI("GPS_DETECT", "Testing baudrate: %d", current_baud);

    /* Apply baudrate to UART peripheral */
    ESP_LOGI("GPS_DETECT", "Setting UART driver to baudrate: %d", current_baud);
    ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_PORT, current_baud));
    /* Brief settling delay after UART baudrate change */
    vTaskDelay(pdMS_TO_TICKS(100));
    /* Flush RX buffer before testing */
    uart_flush_input(GPS_UART_PORT);

    TickType_t start_time = xTaskGetTickCount();
    uint32_t valid_sentences = 0;

    while ((xTaskGetTickCount() - start_time) <
           pdMS_TO_TICKS(BAUD_TEST_DURATION_MS)) {
      vTaskDelay(10);
      int len = uart_read_bytes(GPS_UART_PORT, test_buf, sizeof(test_buf),
                                pdMS_TO_TICKS(100));
      if (len > 0) {
        if (is_valid_nmea_sentence(test_buf, len)) {
          valid_sentences++;
          if (valid_sentences >= MIN_VALID_SENTENCES) {
            ESP_LOGI("GPS_DETECT",
                     "✓ Baudrate %d confirmed! (%d valid sentences)",
                     current_baud, valid_sentences);
            LVGL_LOCK();
            lv_label_set_text_fmt(objects.boot_text, "Detect: %d",
                                  current_baud);
            LVGL_UNLOCK();
            return current_baud;
          }
        }
      }
    }

    ESP_LOGW("GPS_DETECT", "✗ Baudrate %d failed (only %d valid sentences)",
             current_baud, valid_sentences);
  }

  ESP_LOGW("GPS_DETECT", "Could not detect baudrate");
  LVGL_LOCK();
  lv_label_set_text(objects.boot_text, "Detect fail");
  LVGL_UNLOCK();
  return 0;
}

/**
 * @brief Configure GPS module to 115200 baud and set required message rates
 * @param current_baud Current detected baudrate
 * @return true if configuration succeeded
 */
static bool setting_gps_module(int current_baud) {
  if (current_baud == 115200) {
    ESP_LOGI("GPS_CFG", "GPS already at 115200 baud");
    return true;
  }

  ESP_LOGI("GPS_DETECT", "Setting UART driver to baudrate: %d", current_baud);
  /* IMPORTANT: Restore UART to the current GPS baudrate before sending commands
   */
  ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_PORT, current_baud));

  /* Wait for UART to settle */
  vTaskDelay(pdMS_TO_TICKS(500));
  /* Send baudrate change command (PCAS01) to switch GPS module to 115200 */
  if (gps_send_command(CFG_BAUDRATE_115200, 100)) {
    ESP_LOGI("GPS", "Set module baudrate to 115200");
    /* Wait for GPS to apply new settings */
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_PORT, TARGET_GPS_BAUD_RATE));

    ESP_LOGI("GPS", "Look like gps module is in factory setting. Setting now!");
    /* Set GPS update rate */
    if (gps_send_command(CFG_UPDATE_RATE_10HZ, 100)) {
      ESP_LOGI("GPS", "Set update rate to 10hz");
      /* Wait for GPS to apply new settings */
      LVGL_LOCK();
      lv_label_set_text(objects.boot_text, "baud 115200");
      lv_bar_set_value(objects.boot_percent, 65, LV_ANIM_ON);
      LVGL_UNLOCK();
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      ESP_LOGE("GPS",
               "Failed to send configuration command: Set update rate to 10hz");
    }

    /* Set GPS sentences */
    if (gps_send_command(CFG_ENABLE_MESSAGE_GGA_RMC, 100)) {
      ESP_LOGI("GPS", "Set enable NMEA message: GGA & RMC");
      /* Wait for GPS to apply new settings */
      LVGL_LOCK();
      lv_label_set_text(objects.boot_text, "GGA & RMC");
      lv_bar_set_value(objects.boot_percent, 70, LV_ANIM_ON);
      LVGL_UNLOCK();
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      ESP_LOGE("GPS", "Failed to send configuration command: Set enable NMEA "
                      "message: GGA & RMC");
    }

    /* Set GPS constellations */
    if (gps_send_command(CFG_ENABLE_CONSTELLATION_DUAL_GPS_BEIDOU, 100)) {
      ESP_LOGI("GPS", "Set enable constellations GPS+BEIDOU");
      /* Wait for GPS to apply new settings */
      LVGL_LOCK();
      lv_label_set_text(objects.boot_text, "GPS & BEIDOU");
      lv_bar_set_value(objects.boot_percent, 75, LV_ANIM_ON);
      LVGL_UNLOCK();
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      ESP_LOGE("GPS", "Failed to send configuration command: Set enable "
                      "constellations GPS+BEIDOU");
    }
    /* Wait for GPS module to apply new settings */
    vTaskDelay(pdMS_TO_TICKS(100));
  } else {
    ESP_LOGE(
        "GPS",
        "Failed to send configuration command: Set module baudrate to 115200");
  }

  return true;
}

/**
 * @brief Initialize and auto-detect/configure GPS baudrate
 * @return true if successful
 */
static bool gps_init_with_auto_baudrate(void) {
  ESP_LOGI("GPS", "Starting GPS with auto baudrate detection");

  /* Step 1: Detect current baudrate */
  int detected_baud = detect_gps_baudrate();

  if (detected_baud == 0) {
    ESP_LOGE("GPS", "Failed to detect GPS baudrate!");
    return false; // Return false if baudrate detection failed
  }

  ESP_LOGI("GPS", "Detected baudrate: %d", detected_baud);
  g_current_gps_baudrate = detected_baud;

  /* Step 2: If not 115200, configure GPS to switch to 115200 */
  if (detected_baud != TARGET_GPS_BAUD_RATE) {
    LVGL_LOCK();
    lv_label_set_text(objects.boot_text, "Setting GPS Module");
    lv_bar_set_value(objects.boot_percent, 60, LV_ANIM_ON);
    LVGL_UNLOCK();
    if (!setting_gps_module(detected_baud)) {
      ESP_LOGW(
          "GPS",
          "Failed to configure GPS to 115200, continuing with detected baud");
      return true;
    }
    /* Step 3: Switch UART to 115200 and verify */
    ESP_LOGI("GPS_DETECT", "Setting UART driver to baudrate: %d",
             TARGET_GPS_BAUD_RATE);
    ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_PORT, TARGET_GPS_BAUD_RATE));
    g_current_gps_baudrate = TARGET_GPS_BAUD_RATE;
    /* Verify GPS is now responding at 115200 */
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t verify_buf[256];
    int len = uart_read_bytes(GPS_UART_PORT, verify_buf, sizeof(verify_buf),
                              pdMS_TO_TICKS(200));
    if (len > 0 && is_valid_nmea_sentence(verify_buf, len)) {
      ESP_LOGI("GPS", "✓ GPS successfully reconfigured to 115200 baud");
      LVGL_LOCK();
      lv_bar_set_value(objects.boot_percent, 80, LV_ANIM_ON);
      LVGL_UNLOCK();
    } else {
      ESP_LOGW("GPS", "⚠ GPS may not be at 115200");
      return false;
    }
  }

  return true;
}

/* ========================================================================== */
/*                              GPS SHARED DOUBLE BUFFER                      */
/* ========================================================================== */

/*
 * Lock-free single-producer / multiple-consumer double buffer.
 *
 * How it works:
 *   - gps_task (writer) always writes to the *inactive* slot (index ^ 1),
 *     then atomically flips `index` to point readers at the new data.
 *   - Readers (ui_task, rtc_sync_task) call gps_read_latest(), which
 *     re-reads `index` before and after copying to detect a concurrent flip.
 *     If a flip happened mid-copy, the read is retried.
 *
 * This avoids any mutex in the hot data path.
 *
 * RECOMMENDATION: If gps_data_t grows large (> a cache line), consider
 * wrapping with a seqlock instead of the retry loop for more deterministic
 * worst-case latency.
 */
typedef struct {
  gps_data_t buf[2];      // Two slots; only the one at [index] is "live"
  volatile uint8_t index; // Index of the most recently completed write
} gps_shared_t;

static gps_shared_t g_gps = {0};

/**
 * @brief Safely read the latest GPS data from the double buffer.
 *
 * Retries if a concurrent writer flipped `index` mid-copy.
 *
 * @param out  Destination struct to fill with the latest GPS data.
 */
static inline void gps_read_latest(gps_data_t *out) {
  uint8_t idx1, idx2;
  do {
    idx1 = __atomic_load_n(&g_gps.index, __ATOMIC_ACQUIRE);
    *out = g_gps.buf[idx1];
    idx2 = __atomic_load_n(&g_gps.index, __ATOMIC_ACQUIRE);
  } while (idx1 != idx2);
}

/* ========================================================================== */
/*                              GPS STATS SHARED (for UI)                     */
/* ========================================================================== */

/* Double buffer for rate_counters (similar to gps_data mechanism) */
typedef struct {
  nmea_stats_t buf[2];
  volatile uint8_t index;
} nmea_stats_shared_t;

static nmea_stats_shared_t g_nmea_stats_shared = {0};

/* Thread-safe helper to read the latest rate counters */
static inline void nmea_stats_read_latest(nmea_stats_t *out) {
  uint8_t idx1, idx2;
  do {
    idx1 = __atomic_load_n(&g_nmea_stats_shared.index, __ATOMIC_ACQUIRE);
    *out = g_nmea_stats_shared.buf[idx1];
    idx2 = __atomic_load_n(&g_nmea_stats_shared.index, __ATOMIC_ACQUIRE);
  } while (idx1 != idx2);
}

/* ========================================================================== */
/*                              ODOMETER                                      */
/* ========================================================================== */
/* --- Odometer & EMA Configuration --- */
#define MIN_ODO_SPEED 2.0f
#define MIN_ODO_HDOP 3.0f
static double g_odometer_m = 0.0;    // Total distance in meters
static float g_ema_speed_kmh = 0.0f; // EMA filtered velocity
static float g_ema_alpha = 0.3f;     // Smoothing factor (0.1 - 0.5)
static uint32_t g_last_odo_tick =
    0; // Last tick used for delta-time calculation

/**
 * @brief Update Odometer and EMA speed filtering
 * @param current_speed_kmh Raw speed from GPS module
 * @param is_valid GPS fix status
 * @param reset If true, assign current speed directly to EMA (used on first
 * fix)
 */
static void update_odometer_ema(float current_speed_kmh, bool is_valid,
                                bool reset) {
  if (!is_valid) {
    g_ema_speed_kmh = 0;
    // Do not update g_last_odo_tick here; when the fix is restored
    // the reset logic in gps_task will handle it.
    return;
  }

  uint32_t now = xTaskGetTickCount();

  if (reset) {
    g_ema_speed_kmh = current_speed_kmh;
    g_last_odo_tick = now; // Anchor a new time reference
    return; // EXIT: Do not accumulate distance on this reset frame
  }

  // Compute delta time since last odometer update
  double delta_t_sec = (double)(now - g_last_odo_tick) / configTICK_RATE_HZ;
  g_last_odo_tick = now;

  // Apply EMA filter to smooth raw GPS speed
  g_ema_speed_kmh = (g_ema_alpha * current_speed_kmh) +
                    ((1.0f - g_ema_alpha) * g_ema_speed_kmh);

  // Accumulate odometer distance only when speed exceeds noise threshold
  if (g_ema_speed_kmh > MIN_ODO_SPEED) {
    g_odometer_m += (g_ema_speed_kmh / 3.6) * delta_t_sec;
  }
}

/* ========================================================================== */
/*                              SIGNAL QUALITY HELPERS                        */
/* ========================================================================== */

/**
 * @brief Discrete signal quality levels derived from HDOP.
 *
 * HDOP (Horizontal Dilution of Precision) thresholds follow standard GPS
 * classification:
 *   < 1.5  → Excellent   (ideal conditions, survey-grade accuracy)
 *   < 2.5  → Good        (suitable for navigation)
 *   < 5.0  → Moderate    (acceptable, open sky with some obstruction)
 *   >= 5.0 → Bad         (unreliable fix; heavy urban canyon / indoors)
 *   special → No Signal  (GPS module not communicating at all)
 */

/**
 * @brief Convert a raw HDOP float to a discrete signal_level_t.
 *
 * Called on every GPS frame; the result is compared against the previous
 * level so the signal-strength icon is only redrawn when it changes.
 */
static inline signal_level_t hdop_to_level(float hdop) {
  if (hdop > 5.0f)
    return SIG_BAD;
  if (hdop > 2.5f)
    return SIG_MODERATE;
  if (hdop > 1.5f)
    return SIG_GOOD;
  return SIG_EXCELLENT;
}

static const void *signal_icon_table[] = {
    &SIG_NONE_SYMBOL,      // SIG_NOSIGNAL
    &SIG_BAD_SYMBOL,       // SIG_BAD
    &SIG_MORDERATE_SYMBOL, // SIG_MODERATE
    &SIG_GOOD_SYMBOL,      // SIG_GOOD
    &SIG_EX_SYMBOL         // SIG_EXCELLENT
};

/* ========================================================================== */
/*                              UTILITY FUNCTIONS                             */
/* ========================================================================== */

/**
 * @brief Compare two local_time_t structs for full equality (including
 * validity).
 *
 * Used in the UI task to skip label updates when the displayed time has not
 * changed, reducing unnecessary LVGL redraws.
 */
bool compare_hh_mm(local_time_t *a, local_time_t *b) {
  return (a->hour == b->hour) && (a->minute == b->minute);
}

bool compare_ss(local_time_t *a, local_time_t *b) {
  return (a->second == b->second);
}

bool compare_dd(local_time_t *a, local_time_t *b) { return (a->day == b->day); }

/**
 * @brief Format a decimal-degree latitude into a human-readable DMS string.
 *
 * Output example: "LAT : 10°46.312'N"
 *
 * @param lat  Decimal degrees (positive = North, negative = South).
 * @param buf  Destination character buffer.
 * @param len  Size of buf.
 */
static void format_lat(double lat, char *buf, size_t len) {
  char hemi = (lat >= 0) ? 'N' : 'S';
  lat = fabs(lat);
  int deg = (int)lat;
  double min = (lat - deg) * 60.0;
  snprintf(buf, len, "LAT : %02d°%06.3f'%c", deg, min, hemi);
}

/**
 * @brief Format a decimal-degree longitude into a human-readable DMS string.
 *
 * Output example: "LONG: 106°41.534'E"
 *
 * @param lon  Decimal degrees (positive = East, negative = West).
 * @param buf  Destination character buffer.
 * @param len  Size of buf.
 */
static void format_lon(double lon, char *buf, size_t len) {
  char hemi = (lon >= 0) ? 'E' : 'W';
  lon = fabs(lon);
  int deg = (int)lon;
  double min = (lon - deg) * 60.0;
  snprintf(buf, len, "LONG: %03d°%06.3f'%c", deg, min, hemi);
}

/* ========================================================================== */
/*                              DEBUG TASK (OPTIONAL)                         */
/* ========================================================================== */

#if DEBUG_TASK
/**
 * @brief Print GPS data to the serial console.
 *
 * Time / date are sourced from the local RTC (UTC+7), NOT from gps_data_t,
 * to match exactly what the UI displays. Raw GPS UTC values are shown only
 * as a fallback before the first RTC sync.
 *
 * Position, speed and signal quality are always read directly from gps_data_t.
 *
 * @param gps  Pointer to the latest GPS data snapshot.
 */
static void print_gps_data(const gps_data_t *gps) {
  local_time_t lt;
  gps_rtc_get_local_time(&lt);

  ESP_LOGI(TAG, "--------------------------------------");

  if (lt.valid) {
    ESP_LOGI("DEBUG", "Time (GMT+7): %02d:%02d:%02d", lt.hour, lt.minute,
             lt.second);
    ESP_LOGI("DEBUG", "Date (GMT+7): %02d/%02d/%04d", lt.day, lt.month,
             lt.year);
  } else {
    /*
     * RTC has never been synced yet (waiting for first GPS fix).
     * Fall back to raw UTC from the GPS sentence as a temporary display.
     */
    ESP_LOGI("DEBUG", "Time (UTC) : %02d:%02d:%02d  [waiting for RTC sync]",
             gps->hour, gps->minute, gps->second);
    ESP_LOGI("DEBUG", "Date (UTC) : %02d/%02d/%04d", gps->day, gps->month,
             gps->year);
  }

  if (gps->valid) {
    ESP_LOGI("DEBUG", "Fix valid  : YES");
    ESP_LOGI("DEBUG", "Latitude   : %.7f %s",
             gps->latitude >= 0 ? gps->latitude : -gps->latitude,
             gps->latitude >= 0 ? "N" : "S");
    ESP_LOGI("DEBUG", "Longitude  : %.7f %s",
             gps->longitude >= 0 ? gps->longitude : -gps->longitude,
             gps->longitude >= 0 ? "E" : "W");
    ESP_LOGI("DEBUG", "Satellites : %d", gps->satellites);
    ESP_LOGI("DEBUG", "HDOP       : %.2f", (double)gps->hdop);
    ESP_LOGI("DEBUG", "Altitude   : %.1f m", (double)gps->altitude_m);
    ESP_LOGI("DEBUG", "Speed      : %.2f km/h", (double)gps->speed_kmh);
    ESP_LOGI("DEBUG", "Course     : %.1f deg", (double)gps->course_deg);
  } else {
    ESP_LOGI("DEBUG", "Fix valid  : NO");
  }
}
#endif

/* ========================================================================== */
/*                              LVGL CALLBACKS                                */
/* ========================================================================== */

/*
 * speed_compensation: user-adjustable additive offset (+0 … +4 km/h).
 * Added to the GPS speed only when the vehicle is moving (speed > 0),
 * so a stationary reading stays at 0.
 *
 * RECOMMENDATION: Persist speed_compensation to NVS so it survives reboots.
 */
static uint8_t speed_compensation = 0;
static int current_screen = SCREEN_ID_SRC_MAIN;

/*
 * One shared LVGL timer for the "show image for 2 s then hide" pattern.
 * Re-using a single timer avoids unbounded timer creation if the event fires
 * frequently (e.g. rapid RTC sync during weak signal).
 */
static lv_timer_t *label_timer = NULL;

/**
 * @brief LVGL timer callback – hides the target widget after 2 seconds
 * The timer is paused after hiding to wait for the next trigger.
 */
static void hide_label_cb(lv_timer_t *t) {
  lv_obj_t *label = (lv_obj_t *)lv_timer_get_user_data(t);
  if (!lv_obj_has_flag(label, LV_OBJ_FLAG_HIDDEN))
    lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
  lv_timer_pause(t);
}

/**
 * @brief Show an LVGL widget for 2 seconds then auto-hide
 * Safe for repeated calls: resets the countdown if widget is already visible.
 */
void ui_show_label_2s(lv_obj_t *label) {
  if (lv_obj_has_flag(label, LV_OBJ_FLAG_HIDDEN))
    lv_obj_clear_flag(label, LV_OBJ_FLAG_HIDDEN);

  if (label_timer) {
    lv_timer_set_user_data(label_timer, label);
    lv_timer_reset(label_timer);
    lv_timer_resume(label_timer);
  } else {
    label_timer = lv_timer_create(hide_label_cb, 2000, label);
  }
}

/**
 * @brief Increment the speed compensation offset (cycles 0 → 1 → … → 4 → 0).
 *
 * Triggered by the "+" button on the main screen.
 * Both label objects are updated so the value is visible regardless of which
 * screen is currently active.
 */
static void btn_inc_cb(lv_event_t *e) {
  speed_compensation = (speed_compensation > 4) ? 0 : speed_compensation + 1;
  ESP_LOGI("UI", "Speed compensation: +%d", speed_compensation);
  lv_label_set_text_fmt(objects.speed_adjust_main, "+%d", speed_compensation);
}

/**
 * @brief Advance to the next screen, wrapping around after the last screen,
 * skip welcome screen
 */
static void btn_next_screen_cb(lv_event_t *e) {
  current_screen++;
  if (current_screen > _SCREEN_ID_LAST)
    current_screen = SCREEN_ID_SRC_MAIN;
  loadScreen(current_screen);
  ESP_LOGI("UI", "Go to next screen");
}

/**
 * @brief Go back to the previous screen, wrapping around before the first
 * screen, skip welcome screen
 */
static void btn_prev_screen_cb(lv_event_t *e) {
  current_screen--;
  if (current_screen < SCREEN_ID_SRC_MAIN)
    current_screen = _SCREEN_ID_LAST;
  loadScreen(current_screen);
  ESP_LOGI("UI", "Go to previous screen");
}

/**
 * @brief Called by the LCD panel IO driver when a DMA transfer completes.
 *
 * Signals LVGL that the framebuffer flush is done so it can release the
 * draw buffer and start the next rendering pass.
 *
 * Runs in ISR context – must not call blocking APIs.
 */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx) {
  lv_display_t *disp = (lv_display_t *)user_ctx;
  lv_display_flush_ready(disp);
  return false;
}

/**
 * @brief Reconfigure the ST7796 swap/mirror registers when the display
 *        rotation changes.
 *
 * Called by LVGL whenever lv_display_set_rotation() is invoked.
 * Each rotation case maps to the correct hardware mirror/swap combination
 * for the ST7796 controller.
 */
static void lvgl_port_update_callback(lv_display_t *disp) {
  esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
  lv_display_rotation_t rotation = lv_display_get_rotation(disp);

  switch (rotation) {
  case LV_DISPLAY_ROTATION_0:
    esp_lcd_panel_swap_xy(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, true, false);
    break;
  case LV_DISPLAY_ROTATION_90:
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, true, true);
    break;
  case LV_DISPLAY_ROTATION_180:
    esp_lcd_panel_swap_xy(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, false, true);
    break;
  case LV_DISPLAY_ROTATION_270:
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, false);
    break;
  }
}

/**
 * @brief LVGL flush callback – sends a rendered tile to the LCD via SPI DMA.
 *
 * Byte-swaps the RGB565 pixel data in-place before sending, because the
 * ST7796 expects big-endian 16-bit colour words while LVGL produces
 * little-endian.
 *
 * RECOMMENDATION: If the DMA engine supports hardware byte-swap, enable it
 * in spi_device_interface_config_t to eliminate the software swap and save
 * CPU time proportional to (area * 2) bytes per flush.
 */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                          uint8_t *px_map) {
  esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
  int x1 = area->x1, x2 = area->x2;
  int y1 = area->y1, y2 = area->y2;
  lv_draw_sw_rgb565_swap(px_map, (x2 + 1 - x1) * (y2 + 1 - y1));
  esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, px_map);
}

/**
 * @brief esp_timer periodic callback that drives the LVGL tick counter.
 *
 * Runs in ISR context every LVGL_TICK_PERIOD_MS (1 ms).
 * lv_tick_inc() is ISR-safe per the LVGL documentation.
 */
static void increase_lvgl_tick(void *arg) { lv_tick_inc(LVGL_TICK_PERIOD_MS); }

/**
 * @brief LVGL input device read callback for the XPT2046 touch controller.
 *
 * Called by LVGL on every input scan cycle. Reads the latest touch
 * coordinates from the driver and maps them to LVGL's pointer state.
 *
 * RECOMMENDATION: Apply a simple IIR low-pass filter on touchpad_x/y to
 * reduce jitter – the XPT2046 ADC can be noisy without hardware averaging.
 */
static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t touchpad_x[1] = {0};
  uint16_t touchpad_y[1] = {0};
  uint8_t touchpad_cnt = 0;

  esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
  esp_lcd_touch_read_data(touch_pad);
  bool touchpad_pressed = esp_lcd_touch_get_coordinates(
      touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

  if (touchpad_pressed && touchpad_cnt > 0) {
    data->point.x = touchpad_x[0];
    data->point.y = touchpad_y[0];
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

/* ========================================================================== */
/*                              TASKS                                         */
/* ========================================================================== */
#define LVGL_TASK_PRIORITY 6
#define GPS_TASK_PRIORITY 6
#define UI_TASK_PRIORITY 5
#define RTC_TASK_PRIORITY 4
#define DEBUG_TASK_PRIORITY 2

/**
 * @brief GPS UART reader & NMEA parser task.
 *
 * Polls UART2 in a tight loop (10 ms timeout per read), feeds raw bytes into
 * the incremental NMEA FSM parser, and on each completed sentence:
 *   1. Writes the new gps_data_t into the inactive double-buffer slot.
 *   2. Atomically flips the slot index (making the new data "live").
 *   3. Notifies rtc_sync_task and ui_task via FreeRTOS task notification.
 *
 * The `seq` counter in gps_data_t allows receivers to detect stale reads
 * without additional locking.
 *
 * RECOMMENDATION: Consider raising TARGET_GPS_BAUD_RATE to 115200 (already set)
 * and configuring the ATGM336H to output only the NMEA sentences you actually
 * use (e.g. $GPRMC + $GPGGA) to reduce parse load.
 */
static void gps_task(void *arg) {
  nmea_parser_t parser;
  nmea_parser_init(&parser);
  gps_data_t last_data = {0};
  static uint8_t rx_buf[UART_READ_BUF_SZ];
  TickType_t last_parsed_tick =
      xTaskGetTickCount(); // Timestamp of the last successfully received NMEA
                           // byte
  TickType_t last_sync_tick =
      xTaskGetTickCount();  // Reset each time rtc_sync_task completes a sync
  bool gps_timeout = false; // Current GPS communication timeout state
  bool last_stale = false;
  uint32_t events = 0;
  TickType_t last_stale_check = xTaskGetTickCount();
  /* ── Event rate counter ────────────────────────────────────────────── */
  static uint32_t update_count_in_interval = 0;
  static int64_t last_rate_calc_us = 0;
  event_rate_stats_t current_rate_stats = {0};

  ESP_LOGI("GPS", "GPS task started");

  while (1) {
    uint32_t ui_evt_bits = 0;  // reset noti bit
    uint32_t rtc_evt_bits = 0; // reset noti bit
    uint32_t dbg_evt_bits = 0; // reset noti bit
    /* ── Non-blocking: receive EVT_RTC_SYNC_DONE from rtc_task ───────────── */
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &events, 0) == pdTRUE) {
      if (events & EVT_RTC_SYNC_DONE) {
        last_sync_tick = xTaskGetTickCount();
        ESP_LOGI("GPS", "RTC sync done – stale timer reset");
      }
    }

    /* ... (UART reading logic) ... */
    int rx_len = uart_read_bytes(GPS_UART_PORT, rx_buf, sizeof(rx_buf),
                                 pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
    if (rx_len > 0) {
      nmea_parser_feed(&parser, rx_buf, (size_t)rx_len);
      TickType_t now = xTaskGetTickCount();

      /* ── Only process when a new complete NMEA sentence is parsed ──────── */
      if (parser.data.seq != last_data.seq) {
        last_parsed_tick = now;
        /* EMA Reset Check */
        bool ema_need_reset =
            gps_timeout || (parser.data.valid && !last_data.valid);

        // Reset EMA when recovering from timeout or acquiring a new fix
        if (ema_need_reset) {
          // IMPORTANT: Re-sync tick to prevent odometer jumps
          g_last_odo_tick = now;
          // Directly seed EMA with current speed to avoid filtering from a
          // stale zero value
          g_ema_speed_kmh = parser.data.valid ? parser.data.speed_kmh : 0;
          ESP_LOGI("GPS", "EMA/Odo Reset - Syncing tick to prevent jump");
        }

        /* Update odometer and EMA speed */
        if (parser.data.hdop < MIN_ODO_HDOP) {
          update_odometer_ema(parser.data.speed_kmh, parser.data.valid,
                              ema_need_reset);
        } else {
          // Poor signal or No Fix -> Assume stationary to protect odometer data
          update_odometer_ema(0, false, false);
        }

        /* Populate derived fields from parsed and computed values */
        parser.data.is_moving =
            parser.data.valid && (g_ema_speed_kmh > MIN_ODO_SPEED);
        parser.data.odometer_m = g_odometer_m;
        parser.data.valid_changed = (parser.data.valid != last_data.valid);
        parser.data.signal_level =
            parser.data.valid ? hdop_to_level(parser.data.hdop) : SIG_NOSIGNAL;

        /* State change event generation */
        if (parser.data.is_moving != last_data.is_moving) {
          ui_evt_bits |= parser.data.is_moving ? EVT_IS_MOVING_TO_MOVE
                                               : EVT_IS_MOVING_TO_STOP;
        }

        if (parser.data.valid_changed) {
          ui_evt_bits |= parser.data.valid ? EVT_VALID_CHANGE_TO_VALID
                                           : EVT_VALID_CHANGE_TO_INVALID;
        }

        if (parser.data.signal_level != last_data.signal_level) {
          ui_evt_bits |= EVT_SIGNAL_CHANGE;
        }

        /* Recovery: Notify UI only after a valid message is received, not just
         * raw bytes */
        if (gps_timeout) {
          gps_timeout = false;
          ui_evt_bits |= EVT_GPS_RESTORED;
          ESP_LOGI("GPS",
                   "[EVT_GPS_RESTORED -> ui], GPS restored – notifying UI");
          if (parser.data.valid) {
            rtc_evt_bits |= EVT_RTC_SYNC_REQUEST;
            ESP_LOGI("GPS", "[EVT_RTC_SYNC_REQUEST -> rtc]");
          }
          parser.data.is_moving =
              false; // Force is_moving=false on GPS restore to avoid stale
                     // moving→moving edge case; corrects itself on the next GPS
                     // frame
        }

        /* RTC Sync Decision (single source of truth) */
        bool need_rtc_sync = false;
        if (parser.data.valid_changed && parser.data.valid) {
          need_rtc_sync = true;
          ESP_LOGI("GPS",
                   "[EVT_RTC_SYNC_REQUEST -> rtc], GPS no fix → fix, request "
                   "RTC sync (seq=%lu)",
                   parser.data.seq);
        } else if (gps_rtc_should_sync(&parser.data)) {
          need_rtc_sync = true;
          ESP_LOGI("GPS",
                   "[EVT_RTC_SYNC_REQUEST -> rtc] RTC sync request (seq=%lu)",
                   parser.data.seq);
        }
        if (need_rtc_sync)
          rtc_evt_bits |= EVT_RTC_SYNC_REQUEST;

        /* Write to the inactive slot, then flip the index atomically. */
        uint8_t next = g_gps.index ^ 1;
        g_gps.buf[next] = parser.data;
        __atomic_store_n(&g_gps.index, next, __ATOMIC_RELEASE);

        /* Calculate EVT_GPS_UPDATE frequency */
        update_count_in_interval++;

        int64_t now_us = esp_timer_get_time();
        if (last_rate_calc_us == 0) {
          last_rate_calc_us = now_us;
        }

        /* Calculate rate every 1000ms */
        if (now_us - last_rate_calc_us >= STATS_DISPLAY_INTERVAL_MS * 1000) {
          float elapsed_sec = (float)(now_us - last_rate_calc_us) / 1000000.0f;
          current_rate_stats.update_rate_per_sec =
              (float)update_count_in_interval / elapsed_sec;
          current_rate_stats.total_updates += update_count_in_interval;
          current_rate_stats.last_update_count = update_count_in_interval;
          current_rate_stats.last_rate_calc_us = last_rate_calc_us;

          /* Write updated event rate stats to the shared double buffer */
          uint8_t next = g_event_rate_shared.index ^ 1;
          g_event_rate_shared.buf[next] = current_rate_stats;
          __atomic_store_n(&g_event_rate_shared.index, next, __ATOMIC_RELEASE);
          /* Reset counter */
          update_count_in_interval = 0;
          last_rate_calc_us = now_us;
        }

        ui_evt_bits |= EVT_GPS_UPDATE;

        // ESP_LOGI("GPS", "[EVT_GPS_UPDATE -> ui] New GPS sentence");
#if DEBUG_TASK
        ESP_LOGI("DEBUG", "[EVT_GPS_UPDATE -> debug]");
        dbg_evt_bits |= EVT_GPS_UPDATE;
#endif
        last_data = parser.data;
        /* ── Lấy thống kê tốc độ bản tin ────────────────────────────── */
        now_us = esp_timer_get_time();
        if (now_us - g_last_stats_display_us >
            STATS_DISPLAY_INTERVAL_MS * 1000) {
          nmea_parser_get_stats(&parser, &g_nmea_stats);
          /* Write NMEA stats to the shared double buffer for UI consumption */
          uint8_t next = g_nmea_stats_shared.index ^ 1;
          g_nmea_stats_shared.buf[next] = g_nmea_stats;
          __atomic_store_n(&g_nmea_stats_shared.index, next, __ATOMIC_RELEASE);
#if DEBUG_TASK
          ESP_LOGI("GPS_STATS", "=== NMEA Sentence Rate ===");
          ESP_LOGI("GPS_STATS", "Total : %.1f sentences/sec",
                   (double)g_nmea_stats.total_rate_per_sec);
          ESP_LOGI("GPS_STATS", "GGA   : %.1f sentences/sec",
                   (double)g_nmea_stats.gga_rate_per_sec);
          ESP_LOGI("GPS_STATS", "RMC   : %.1f sentences/sec",
                   (double)g_nmea_stats.rmc_rate_per_sec);
          ESP_LOGI("GPS_STATS", "Other : %.1f sentences/sec",
                   (double)g_nmea_stats.other_rate_per_sec);
          ESP_LOGI("GPS_EVT_RATE",
                   "EVT_GPS_UPDATE rate: %.1f updates/sec (total: %lu)",
                   (double)current_rate_stats.update_rate_per_sec,
                   current_rate_stats.total_updates);
#endif
          g_last_stats_display_us = now_us;
          /* Notify ui_task that updated rate counters are available */
          ui_evt_bits |= EVT_GPS_STATS_UPDATE;
        }
      }
    } else {
      /* No data in this window – yield for others tasks. */
      /* ── No data received: Check for timeout ─────────────────── */
      TickType_t now = xTaskGetTickCount();
      if (!gps_timeout &&
          (now - last_parsed_tick) > pdMS_TO_TICKS(GPS_COM_TIMEOUT_MS)) {
        gps_timeout = true;
        ui_evt_bits |= EVT_GPS_TIMEOUT;
        ESP_LOGW("GPS",
                 " [EVT_GPS_TIMEOUT -> ui] GPS timeout > %ds – notifying UI",
                 GPS_COM_TIMEOUT_MS / 1000);
        g_last_odo_tick =
            0; // Reset tick to prevent odometer jump when signal is restored
        g_ema_speed_kmh = 0;
      }
    }
    last_stale_check = xTaskGetTickCount();
    /* ── Stale check: Internal tick comparison ────────── */
    bool is_stale = (last_stale_check - last_sync_tick) >
                    pdMS_TO_TICKS(GPS_RTC_STALE_THRESHOLD_MS);
    if (is_stale != last_stale) {
      last_stale = is_stale;
      if (is_stale) {
        ui_evt_bits |= EVT_RTC_STALE;
        ESP_LOGW("GPS",
                 "[EVT_RTC_STALE -> ui] RTC stale – no sync for > %llums",
                 (uint64_t)GPS_RTC_STALE_THRESHOLD_MS);
        // Check gps_timeout: parser retains stale data during timeout, so skip
        // sync request if the GPS signal is currently lost
        if ((!gps_timeout) && (parser.data.valid)) {
          rtc_evt_bits |= EVT_RTC_SYNC_REQUEST;
          ESP_LOGI("GPS",
                   "[EVT_RTC_SYNC_REQUEST -> rtc] Force RTC sync due to stale "
                   "(seq=%lu)",
                   parser.data.seq);
        }
      } else {
        ui_evt_bits |= EVT_RTC_NOT_STALE;
        ESP_LOGI("GPS", "[EVT_RTC_NOT_STALE -> ui] RTC no longer stale");
      }
    }
    /* ── DISPATCH ALL AGGREGATED EVENTS ───────────────────────────── */
    if (ui_evt_bits) {
      xTaskNotify(ui_task_handle, ui_evt_bits, eSetBits);
    }
    if (rtc_evt_bits) {
      xTaskNotify(rtc_task_handle, rtc_evt_bits, eSetBits);
    }
#if DEBUG_TASK
    if (dbg_evt_bits) {
      xTaskNotify(dbg_task_handle, dbg_evt_bits, eSetBits);
    }
#endif
  }
}

/**
 * @brief Executes RTC synchronisation commands issued by gps_task.
 *
 * This task does not decide when to sync — all logic (should_sync, stale
 * detection, forced sync on fix restore) is handled by gps_task.
 * rtc_sync_task acts purely as an executor:
 *
 *   1. Waits for EVT_RTC_SYNC_REQUEST from gps_task (blocks indefinitely).
 *   2. Reads the latest GPS data from the double buffer.
 *   3. Calls gps_rtc_sync() to update the system time.
 *   4. Sends EVT_RTC_SYNC_DONE back to:
 *        - gps_task  : to reset its internal stale timer.
 *        - ui_task   : to briefly flash the sync icon for 2 seconds.
 *        - dbg_task  : for logging (only when DEBUG_TASK = 1).
 */
static void rtc_sync_task(void *arg) {
  ESP_LOGI("RTC", "RTC sync task started");
  gps_data_t d;
  uint32_t events;

  while (1) {
    /*
     * Wake up only on EVT_RTC_SYNC_REQUEST from gps_task.
     * All decisions about whether and when to sync are made in gps_task;
     * rtc_sync_task only executes the sync and reports completion.
     */
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &events, portMAX_DELAY) == pdTRUE) {
      if (events & EVT_RTC_SYNC_REQUEST) {
        gps_read_latest(&d);
        gps_rtc_sync(&d);

        /* Notify gps_task to reset its internal stale timer */
        xTaskNotify(gps_task_handle, EVT_RTC_SYNC_DONE, eSetBits);
        /* Notify ui_task to briefly flash the sync icon */
        xTaskNotify(ui_task_handle, EVT_RTC_SYNC_DONE, eSetBits);
        ESP_LOGI("RTC", "[EVT_RTC_SYNC_DONE -> gps, ui]");
        // Blinking Green LED
        gpio_set_level(LED_GREEN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GREEN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

#if DEBUG_TASK
        xTaskNotify(dbg_task_handle, EVT_RTC_SYNC_DONE, eSetBits);
        ESP_LOGI("DEBUG", "[RTC_SYNC_TASK -> debug]");
#endif
      }
    }
  }
}

/* ========================================================================== */
/*                              UI + LVGL TASK                                */
/* ========================================================================== */

/**
 * @brief LVGL rendering task – calls lv_timer_handler() at an adaptive rate.
 *
 * This task is a pure render loop: it does not process GPS data or read the
 * RTC. All business logic lives in ui_task; lvgl_port_task simply drives the
 * LVGL timer so that dirty widgets are flushed to the display.
 *
 * Adaptive delay:
 *   lv_timer_handler() returns the number of milliseconds until the next LVGL
 *   timer fires. That value is clamped to [LVGL_TASK_MIN_DELAY_MS,
 *   LVGL_TASK_MAX_DELAY_MS] to avoid busy-waiting while still guaranteeing
 *   a minimum wakeup rate of at least twice per second.
 */
static void lvgl_port_task(void *arg) {
  ESP_LOGI("LVGL", "Starting LVGL PORT task (rendering)");
  uint32_t time_till_next_ms = 0;
  while (1) {
    LVGL_LOCK();
    time_till_next_ms = lv_timer_handler();
    LVGL_UNLOCK();
    /* ── ADAPTIVE DELAY ─────────────────────────────────────────────── */
    /*
     * lv_timer_handler() returns ms until the next LVGL timer event.
     * Clamped: MIN avoids CPU spin, MAX guarantees a minimum wakeup frequency.
     */
    time_till_next_ms = LV_MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
    time_till_next_ms = LV_MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
  }
}

/**
 * @brief Updates the LVGL UI in response to events from gps_task and rtc_task.
 *
 * This task is a pure consumer – it does not calculate timeouts, detect stale
 * RTC, or decide when to sync. All state changes are computed by gps_task and
 * rtc_task and delivered via xTaskNotify.
 *
 * Loop structure per iteration:
 *   1. LVGL_LOCK
 *   2. Poll RTC clock – update hour/minute/second/date labels only when
 * changed. Each field is compared individually to suppress unnecessary redraws.
 *   3. xTaskNotifyWait(timeout = 0) – non-blocking event dispatch:
 *        EVT_GPS_TIMEOUT    → blank speed/position widgets, show "NO GPS", hide
 * speed unit. EVT_GPS_RESTORED   → restore previously hidden widgets.
 *        EVT_GPS_UPDATE     → update speed, satellites, signal icon, lat, lon.
 *        EVT_RTC_STALE      → dim clock label to grey.
 *        EVT_RTC_NOT_STALE  → restore clock label to yellow.
 *        EVT_RTC_SYNC_DONE  → flash sync icon for 2 seconds.
 *   4. LVGL_UNLOCK
 *   5. vTaskDelay(5 ms) – yield CPU to other tasks.
 *
 * All LVGL calls are made inside LVGL_LOCK/UNLOCK, including event handling,
 * ensuring no race condition with lvgl_port_task.
 *
 * RECOMMENDATION: If stack overflow occurs after adding more widgets, increase
 * LVGL_TASK_STACK_SIZE and verify headroom via uxTaskGetStackHighWaterMark()
 * in debug builds.
 */
static void ui_task(void *arg) {
  ESP_LOGI("UI", "UI task started");

  uint32_t events;
  gps_data_t d;
  int last_data_signal_level = 0xFF;
  uint8_t spinGps = 0;
  local_time_t current_time = {};
  local_time_t last_time = {};
  char lat_buf[32];
  char lon_buf[32];
  nmea_stats_t rate_counters;
  event_rate_stats_t event_rate;

  last_time.valid = true;

  static const char *WEEKDAY_STR[] = {"Chủ nhật", "Thứ 2", "Thứ 3", "Thứ 4",
                                      "Thứ 5",    "Thứ 6", "Thứ 7"};

  static const char *frames[] = {
      "⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏",
  };
  static const int frames_len = sizeof(frames) / sizeof(frames[0]);

  /* Register touch button event callbacks */
  lv_obj_add_event_cb(objects.btn_inc, btn_inc_cb, LV_EVENT_RELEASED, NULL);
  lv_obj_add_event_cb(objects.main_next_scr, btn_next_screen_cb,
                      LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(objects.main_prev_scr, btn_prev_screen_cb,
                      LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(objects.time_next_scr, btn_next_screen_cb,
                      LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(objects.time_prev_scr, btn_prev_screen_cb,
                      LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(objects.info_next_scr, btn_next_screen_cb,
                      LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(objects.info_prev_scr, btn_prev_screen_cb,
                      LV_EVENT_CLICKED, NULL);
  ESP_LOGI("UI", "Button callbacks registered");
  // Loading main screen before start task
  ESP_LOGI("UI", "Loading Main screen");
  loadScreen(SCREEN_ID_SRC_MAIN);
  /* Initialize default icons */
  lv_label_set_text(objects.icon_sync_rtc, SYNC_SYMBOL);
  lv_label_set_text(objects.signal_bar_icon, SIG_NONE_SYMBOL);
  lv_obj_set_style_text_color(objects.signal_bar_icon, lv_color_hex(0xff4c4c),
                              LV_PART_MAIN | LV_STATE_DEFAULT);

  while (1) {
    LVGL_LOCK();

    /* ── SECTION 1: RTC CLOCK DISPLAY ───────────────────────────────── */
    /*
     * Poll RTC on every iteration so the clock updates continuously even
     * without new GPS packets. Compare each field individually to suppress
     * unnecessary redraws.
     */
    gps_rtc_get_local_time(&current_time);

    if (current_time.valid != last_time.valid) {
      if (!current_time.valid) {
        lv_label_set_text(objects.hour_minute, "--:--");
        lv_label_set_text(objects.second, "--");
        lv_label_set_text(objects.date, "Waiting GPS");
      } else {
        /* Force a full redraw on the first valid time reading */
        last_time.hour = -1;
        last_time.minute = -1;
        last_time.second = -1;
        last_time.day = -1;
      }
    }

    if (current_time.valid) {
      if (!compare_hh_mm(&current_time, &last_time))
        lv_label_set_text_fmt(objects.hour_minute, "%02d:%02d",
                              current_time.hour, current_time.minute);

      if (!compare_ss(&current_time, &last_time))
        lv_label_set_text_fmt(objects.second, "%02d", current_time.second);

      if (!compare_dd(&current_time, &last_time))
        lv_label_set_text_fmt(objects.date, "%s, %02d/%02d/%04d",
                              WEEKDAY_STR[current_time.week_day],
                              current_time.day, current_time.month,
                              current_time.year);
    }

    last_time = current_time;

    /* ── SECTION 2: EVENT DISPATCH ───────────────────────────────────── */
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &events, 0) == pdTRUE) {
      /* ── EVT_GPS_TIMEOUT ─────────────────────────────────────────── */
      if (events & EVT_GPS_TIMEOUT) {
        lv_label_set_text(objects.sat_num, "NO GPS");
        lv_label_set_text(objects.fix_info, "NO GPS");
        lv_label_set_text(objects.sat_info, "SAT : ");
        lv_label_set_text(objects.hdop_info, "HDOP: ");
        lv_label_set_text(objects.lat_info, "LAT : ");
        lv_label_set_text(objects.long_info, "LONG: ");
        lv_label_set_text(objects.speed_after_adjust, "");
        lv_label_set_text(objects.moving, "IS_MOVING: ~");
        if (!lv_obj_has_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN))
          lv_obj_add_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN);
        last_data_signal_level = SIG_NOSIGNAL;
        lv_label_set_text(objects.signal_bar_icon,
                          signal_icon_table[SIG_NOSIGNAL]);
        lv_obj_set_style_text_color(objects.signal_bar_icon,
                                    lv_color_hex(0xff4c4c),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(objects.odometer_m, lv_color_hex(0xa0a0a0),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(objects.second, lv_color_hex(0xa0a0a0),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(objects.rate_counter_value, "0, 0, 0, 0");
      }

      /* ── EVT_IS_MOVING_CHANGE ──────────────────────────────────────────── */
      if (events & EVT_IS_MOVING_TO_MOVE) {
        lv_obj_set_style_text_color(objects.odometer_m, lv_color_hex(0xffffff),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(objects.moving, "IS_MOVING: 1");
      }

      if (events & EVT_IS_MOVING_TO_STOP) {
        // grey
        lv_obj_set_style_text_color(objects.odometer_m, lv_color_hex(0xa0a0a0),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(objects.moving, "IS_MOVING: 0");
      }

      /* ── EVT_VALID_CHANGE ──────────────────────────────────────────── */
      if (events & EVT_VALID_CHANGE_TO_VALID) {
        lv_obj_set_style_text_color(objects.second, lv_color_hex(0xffffff),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        ESP_LOGI("UI", "GPS --> Fix");
      }

      if (events & EVT_VALID_CHANGE_TO_INVALID) {
        ESP_LOGI("UI", "GPS --> No Fix");
        lv_obj_set_style_text_color(objects.second, lv_color_hex(0xa0a0a0),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(objects.signal_bar_icon,
                          signal_icon_table[SIG_NOSIGNAL]);
      }

      /* ── EVT_GPS_UPDATE ──────────────────────────────────────────── */
      if (events & EVT_GPS_UPDATE) {
        gps_read_latest(&d);
        /* ── EVT_GPS_RESTORED ────────────────────────────────────────── */
        if (events & EVT_GPS_RESTORED) {
          if (lv_obj_has_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN))
            lv_obj_clear_flag(objects.speed_unit, LV_OBJ_FLAG_HIDDEN);
          if (lv_obj_has_flag(objects.sat_num, LV_OBJ_FLAG_HIDDEN))
            lv_obj_clear_flag(objects.sat_num, LV_OBJ_FLAG_HIDDEN);
          gps_read_latest(&d);
          if (d.valid) {
            lv_obj_set_style_text_color(objects.second, lv_color_hex(0xffffff),
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
          } else {
            lv_obj_set_style_text_color(objects.second, lv_color_hex(0xa0a0a0),
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
          }
        }

        spinGps = (spinGps + 1) % frames_len;
        if (d.odometer_m < 10000) {
          lv_label_set_text_fmt(objects.odometer_m, "%.0f m", d.odometer_m);
        } else {
          lv_label_set_text_fmt(objects.odometer_m, "%.1f km",
                                d.odometer_m / 1000);
        }

        int speed_kmh = (int)(d.speed_kmh + 0.5f);
        if (speed_kmh < 1)
          speed_kmh = 0;
        else
          speed_kmh += speed_compensation;

        if (!d.valid) {
          lv_label_set_text(objects.speed_after_adjust, "");
          lv_label_set_text(objects.fix_info, "FIX : NO");
          lv_label_set_text(objects.sat_num, "NOT FIXED!");
          lv_label_set_text_fmt(objects.hdop_info, "HDOP: %.1f (LAST)", d.hdop);
          lv_label_set_text_fmt(objects.sat_info, "SAT : %d", d.satellites);
          lv_label_set_text(objects.lat_info, "LAT :");
          lv_label_set_text(objects.long_info, "LONG:");
          if (last_data_signal_level != SIG_NOSIGNAL) {
            last_data_signal_level = SIG_NOSIGNAL;
            lv_label_set_text(objects.signal_bar_icon,
                              signal_icon_table[SIG_NOSIGNAL]);
            lv_obj_set_style_text_color(objects.signal_bar_icon,
                                        lv_color_hex(0xff4c4c),
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
          }
        } else {
          /* ── EVT_SIGNAL_CHANGE ────────────────────────────────────────── */
          if (events & EVT_SIGNAL_CHANGE) {
            lv_label_set_text(objects.signal_bar_icon,
                              signal_icon_table[d.signal_level]);
            switch (d.signal_level) {
            case SIG_MODERATE:
              lv_obj_set_style_text_color(objects.signal_bar_icon,
                                          lv_color_hex(0xFFB300),
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
              break;
            case SIG_GOOD:
              lv_obj_set_style_text_color(objects.signal_bar_icon,
                                          lv_color_hex(0x8BC34A),
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
              break;
            case SIG_EXCELLENT:
              lv_obj_set_style_text_color(objects.signal_bar_icon,
                                          lv_color_hex(0x4CAF50),
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
              break;
            default:
              lv_obj_set_style_text_color(objects.signal_bar_icon,
                                          lv_color_hex(0xff4c4c),
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
              break;
            }
          }

          lv_label_set_text_fmt(objects.speed_after_adjust, "%d", speed_kmh);
          lv_label_set_text_fmt(objects.sat_num, "SAT: %d", d.satellites);
          lv_label_set_text(objects.fix_info, "FIX : YES");
          lv_label_set_text_fmt(objects.hdop_info, "HDOP: %.1f", d.hdop);
          lv_label_set_text_fmt(objects.sat_info, "SAT : %d", d.satellites);
          format_lat(d.latitude, lat_buf, sizeof(lat_buf));
          format_lon(d.longitude, lon_buf, sizeof(lon_buf));
          lv_label_set_text(objects.lat_info, lat_buf);
          lv_label_set_text(objects.long_info, lon_buf);
        }

        lv_label_set_text(objects.gps_render_loading_indicator,
                          frames[spinGps]);
        lv_label_set_text(objects.gps_render_loading_indicator_1,
                          frames[spinGps]);

        last_data_signal_level = d.signal_level;
      }

      if (events & EVT_GPS_STATS_UPDATE) {
        nmea_stats_read_latest(&rate_counters);
        event_rate_read_latest(&event_rate);

        /* Update rate counter labels in the UI */
        lv_label_set_text_fmt(objects.rate_counter_value, "%d, %d, %d, %d",
                              (int)(rate_counters.total_rate_per_sec + 0.5f),
                              (int)(rate_counters.gga_rate_per_sec + 0.5f),
                              (int)(rate_counters.rmc_rate_per_sec + 0.5f),
                              (int)(event_rate.update_rate_per_sec + 0.5f));
      }

      /* ── EVT_RTC_STALE ───────────────────────────────────────────── */
      if (events & EVT_RTC_STALE) {
        lv_obj_set_style_text_color(objects.hour_minute, lv_color_hex(0xa0a0a0),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
      }

      /* ── EVT_RTC_NOT_STALE ───────────────────────────────────────── */
      if (events & EVT_RTC_NOT_STALE) {
        lv_obj_set_style_text_color(objects.hour_minute, lv_color_hex(0xffbe00),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
      }

      /* ── EVT_RTC_SYNC_DONE ───────────────────────────────────────── */
      if (events & EVT_RTC_SYNC_DONE) {
        ui_show_label_2s(objects.icon_sync_rtc);
        ESP_LOGI("UI", "RTC synced – icon shown");
      }
    }

    LVGL_UNLOCK();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ========================================================================== */
/*                              DEBUG TASK (OPTIONAL)                         */
/* ========================================================================== */

#if DEBUG_TASK
/**
 * @brief Serial debug task – mirrors GPS events to the UART console.
 *
 * Enabled only when DEBUG_TASK = 1. Receives the same notifications as
 * ui_task but only logs to ESP_LOGI; does not touch LVGL.
 *
 * RECOMMENDATION: Disable in production builds (DEBUG_TASK 0) to recover
 * ~2 KB of task stack and eliminate serial log overhead at runtime.
 */
static void debug_task(void *arg) {
  ESP_LOGI("DEBUG", "Start debug task");
  uint32_t events;
  gps_data_t d;
  uint32_t last_seq = 0;

  while (1) {
    xTaskNotifyWait(0, 0xFFFFFFFF, &events, portMAX_DELAY);
    if (events & EVT_GPS_UPDATE) {
      gps_read_latest(&d);
      if (d.seq == last_seq)
        continue;
      last_seq = d.seq;
      print_gps_data(&d);
      UBaseType_t gps_stack = uxTaskGetStackHighWaterMark(gps_task_handle);
      UBaseType_t ui_stack = uxTaskGetStackHighWaterMark(ui_task_handle);
      UBaseType_t rtc_stack = uxTaskGetStackHighWaterMark(rtc_task_handle);
      UBaseType_t lvgl_stack = uxTaskGetStackHighWaterMark(lvgl_task_handle);
      ESP_LOGI("DEBUG", "Free GPS stack in bytes: %u", gps_stack);
      ESP_LOGI("DEBUG", "Free UI stack in bytes: %u", ui_stack);
      ESP_LOGI("DEBUG", "Free RTC stack in bytes: %u", rtc_stack);
      ESP_LOGI("DEBUG", "Free LVGL stack in bytes: %u", lvgl_stack);
    }
    if (events & EVT_RTC_SYNC_DONE) {
      ESP_LOGI("DEBUG", "[EVT_RTC_SYNC_DONE] Event received: RTC synced");
    }
  }
}
#endif

/* ========================================================================== */
/*                              HARDWARE INITIALISATION                       */
/* ========================================================================== */

/**
 * @brief Initialise UART2 for receiving NMEA sentences from the ATGM336H.
 *
 * No event queue is used – gps_task polls the driver ring buffer directly,
 * which keeps the ISR path minimal.
 */
static int gps_uart_init(void) {
  const uart_config_t uart_cfg = {
      .baud_rate = TARGET_GPS_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_GPIO, GPS_RX_GPIO,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(
      uart_driver_install(GPS_UART_PORT, UART_RX_RING_BUF, 0, 0, NULL, 0));
  /* Auto-detect and configure the GPS module baudrate */
  if (!gps_init_with_auto_baudrate()) {
    ESP_LOGW("GPS", "Auto baudrate detection failed!");
    return -1;
  }
  ESP_LOGI("GPS", "UART%d init OK – RX=GPIO%d | TX=GPIO%d – %d baud",
           GPS_UART_PORT, GPS_RX_GPIO, GPS_TX_GPIO, TARGET_GPS_BAUD_RATE);
  return 0;
}

/* ========================================================================== */
/*                              APPLICATION ENTRY POINT                       */
/* ========================================================================== */

/**
 * @brief Main application entry point.
 *
 * Initialisation order:
 *   1. LVGL mutex
 *   2. GPIO (backlight, RGB LED)
 *   3. SPI bus (shared by LCD + touch)
 *   4. LCD panel IO → ST7796 panel driver → panel on
 *   5. LVGL core, display, DMA draw buffers, tick timer, flush callback
 *   6. XPT2046 touch controller → LVGL input device
 *   7. Rotation + UI layout (under mutex)
 *   8. RTC module, GPS UART
 *   9. FreeRTOS tasks
 *
 * Task pinning:
 *   All tasks are pinned to core 0. This is intentional – LVGL is not
 *   thread-safe across cores without a proper inter-core lock. Keeping
 *   everything on core 0 avoids cache-coherency surprises on the ESP32
 *   dual-core architecture.
 *
 * RECOMMENDATION: Consider moving gps_task to core 1 (tskNO_AFFINITY or
 * xTaskCreatePinnedToCore(..., 1)) to offload UART polling and NMEA parsing
 * away from the rendering core. The double-buffer + atomic swap design
 * already makes the GPS data handoff safe across cores.
 */
void app_main(void) {
  ESP_LOGI("MAIN", "=== GPS HUD – ATGM336H / ESP32 ===");
  ESP_LOGI("MAIN", "IDF version: %s", esp_get_idf_version());

  /* --- LVGL mutex -------------------------------------------------------- */
  s_lvgl_mutex = xSemaphoreCreateMutex();
  configASSERT(s_lvgl_mutex);

  /* --- GPIO: backlight + RGB LED (active-low) ---------------------------- */
  gpio_config_t bk_gpio_config = {
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT) | (1ULL << LED_RED) |
                      (1ULL << LED_BLUE) | (1ULL << LED_GREEN),
  };
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
  gpio_set_level(LED_RED, 1); /* LEDs off */
  gpio_set_level(LED_BLUE, 1);
  gpio_set_level(LED_GREEN, 1);
  gpio_set_level(PIN_NUM_BK_LIGHT, 0); /* Backlight off until panel is ready */

  /* --- SPI bus (LCD + touch share SPI2) ---------------------------------- */
  spi_bus_config_t buscfg = {
      .sclk_io_num = PIN_NUM_SCLK,
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = PIN_NUM_MISO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = LCD_HOR_RES * 80 * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  /* --- LCD panel IO (SPI → ST7796 command/data protocol) ----------------- */
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = PIN_NUM_LCD_DC,
      .cs_gpio_num = PIN_NUM_LCD_CS,
      .pclk_hz = LCD_PIXEL_CLOCK_HZ,
      .lcd_cmd_bits = LCD_CMD_BITS,
      .lcd_param_bits = LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_handle));

  /* --- ST7796 panel driver ----------------------------------------------- */
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_NUM_LCD_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
      .bits_per_pixel = 16,
  };
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  gpio_set_level(PIN_NUM_BK_LIGHT, 1); /* Panel ready – enable backlight */

  /* --- LVGL initialisation ----------------------------------------------- */
  lv_init();
  lv_display_t *display = lv_display_create(LCD_HOR_RES, LCD_VER_RES);

  /*
   * Allocate draw buffers from DMA-capable memory so the SPI peripheral can
   * read them directly without an extra memcpy through the CPU.
   */
  size_t draw_buffer_sz =
      LCD_HOR_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
  void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
  void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
  configASSERT(buf1 && buf2);

  lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz,
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_user_data(display, panel_handle);
  lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(display, lvgl_flush_cb);
  /* --- LVGL 1 ms tick timer (esp_timer ISR) ------------------------------ */
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &increase_lvgl_tick,
      .name = "lvgl_tick",
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

  /* --- Register DMA-done callback so LVGL knows when a flush completes --- */
  const esp_lcd_panel_io_callbacks_t cbs = {
      .on_color_trans_done = notify_lvgl_flush_ready,
  };
  ESP_ERROR_CHECK(
      esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

  /* --- XPT2046 touch controller ------------------------------------------ */
  esp_lcd_panel_io_handle_t tp_io_handle = NULL;
  esp_lcd_panel_io_spi_config_t tp_io_config = {
      .cs_gpio_num = PIN_NUM_TOUCH_CS,
      .dc_gpio_num = GPIO_NUM_NC,
      .spi_mode = 0,
      .pclk_hz = 2 * 1000 * 1000,
      .trans_queue_depth = 3,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &tp_io_config, &tp_io_handle));

  esp_lcd_touch_config_t tp_cfg = {
      .x_max = LCD_HOR_RES,
      .y_max = LCD_VER_RES,
      .rst_gpio_num = -1,
      .int_gpio_num = PIN_NUM_TOUCH_INT,
      .flags = {.swap_xy = 0, .mirror_x = 1, .mirror_y = 1},
  };
  esp_lcd_touch_handle_t tp = NULL;
  ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_display(indev, display);
  lv_indev_set_user_data(indev, tp);
  lv_indev_set_read_cb(indev, lvgl_touch_cb);

  /* Set rotation and load initial UI while holding the mutex (app_main
   * context). */
  LVGL_LOCK();
  lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);
  lvgl_port_update_callback(display);
  ui_init();
  LVGL_UNLOCK();

  /* --- LVGL Port task must start 1st to have a working display ------------ */

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
  // multi core
  xTaskCreatePinnedToCore(lvgl_port_task, "lvgl_port", LVGL_TASK_STACK_SIZE,
                          NULL, LVGL_TASK_PRIORITY, &lvgl_task_handle, 1);
#else
  // single core
  xTaskCreate(lvgl_port_task, "lvgl_port", LVGL_TASK_STACK_SIZE, NULL,
              LVGL_TASK_PRIORITY, &lvgl_task_handle);
#endif
  LVGL_LOCK();
  lv_label_set_text(objects.boot_text, "Initilize LCD");
  lv_bar_set_value(objects.boot_percent, 20, LV_ANIM_ON);
  LVGL_UNLOCK();
  vTaskDelay(500);
  /* --- RTC module and GPS UART ------------------------------------------- */
  gps_rtc_init();

  LVGL_LOCK();
  lv_label_set_text(objects.boot_text, "Initialize RTC");
  lv_bar_set_value(objects.boot_percent, 40, LV_ANIM_ON);
  LVGL_UNLOCK();
  vTaskDelay(500);

  int uart_check = gps_uart_init();
  // Wait 100ms after UART INIT
  vTaskDelay((pdMS_TO_TICKS(100)));
  if (uart_check == -1) {
    LVGL_LOCK();
    lv_obj_set_style_text_color(objects.boot_text, lv_color_hex(0xff4c4c),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(objects.boot_text, "BAUDRATE ERR!");
    LVGL_UNLOCK();
    ESP_LOGW("ERR", "Hanging forever!, blinking red led.");
    while (1) {
      gpio_set_level(LED_RED, 0);
      vTaskDelay(pdMS_TO_TICKS(500));
      gpio_set_level(LED_RED, 1);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
  LVGL_LOCK();
  lv_bar_set_value(objects.boot_percent, 100, LV_ANIM_ON);
  lv_label_set_text(objects.boot_text, "All Done. Start now...");
  LVGL_UNLOCK();
  vTaskDelay(200);

  /* --- Start others tasks ----------------------------------------- */
  //  * Task pinning:
  //  *   lvgl_port_task and ui_task run on core 1 to isolate rendering from
  //  *   I/O. gps_task and rtc_sync_task run on core 0 to handle UART and
  //  *   RTC independently of the render loop. The double-buffer + atomic
  //  *   swap guarantees a safe handoff between the two cores.
  //  *
  //  *   Core 0: gps_task (priority 6), rtc_sync_task (priority 4)
  //  *   Core 1: lvgl_port_task (priority 6), ui_task (priority 5),
  //  *           debug_task (priority 2)
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
  // multi core
  ESP_LOGI("MAIN", "Using both core for tasks!");
  xTaskCreatePinnedToCore(ui_task, "ui_task", 6144, NULL, UI_TASK_PRIORITY,
                          &ui_task_handle, 1);
  xTaskCreatePinnedToCore(gps_task, "gps_task", 3072, NULL, GPS_TASK_PRIORITY,
                          &gps_task_handle, 0);
  xTaskCreatePinnedToCore(rtc_sync_task, "rtc_sync_task", 2048, NULL,
                          RTC_TASK_PRIORITY, &rtc_task_handle, 0);
#if DEBUG_TASK
  xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL,
                          DEBUG_TASK_PRIORITY, &dbg_task_handle, 1);
#endif
#else
  // single core
  ESP_LOGI("MAIN", "Single core chip, all tasks in one core!");
  xTaskCreate(ui_task, "ui_task", 6144, NULL, UI_TASK_PRIORITY,
              &ui_task_handle);
  xTaskCreate(gps_task, "gps_task", 3072, NULL, GPS_TASK_PRIORITY,
              &gps_task_handle);
  xTaskCreate(rtc_sync_task, "rtc_sync_task", 2048, NULL, RTC_TASK_PRIORITY,
              &rtc_task_handle);

#if DEBUG_TASK
  xTaskCreate(debug_task, "debug_task", 2048, NULL, DEBUG_TASK_PRIORITY,
              &dbg_task_handle);
#endif
#endif
}