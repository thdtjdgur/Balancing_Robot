#include "rtk_bridge.h"

#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gnss.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define UM982_RX_BUFFER_SIZE 4096
#define UM982_TX_BUFFER_SIZE 4096
#define UM982_EVENT_QUEUE_SIZE 20
#define NMEA_LINE_SIZE 384
#define DEG_TO_RAD 0.01745329251994329577f

// If dual antennas are mounted left/right, set this to +90.0f or -90.0f
// after checking whether the UM982 heading vector points left or right.
#define GNSS_HEADING_OFFSET_DEG -48.0f

static const char *TAG = "RTK_BRIDGE";
#define RTK_MISC_LOG_ENABLED 0

static QueueHandle_t um982_uart_queue;
static bool bridge_initialized;

static uint32_t forwarded_frames;
static uint32_t forwarded_bytes;
static uint32_t dropped_frames;
static uint32_t um982_received_bytes;
static uint32_t um982_lines;
static uint32_t gga_lines;
static uint32_t nmea_overflows;
static uint32_t um982_rx_overflow_bytes;

static char nmea_line[NMEA_LINE_SIZE];
static size_t nmea_line_length;
static uint8_t gga_quality;
static uint32_t last_gga_tick_ms;
static float heading_deg;
static float heading_rad;
static bool heading_valid;
static uint32_t last_heading_tick_ms;
static uint32_t last_gga_log_tick_ms;
static uint32_t last_heading_log_tick_ms;
static RTK_Bridge_GGA latest_gga;

static uint32_t rtk_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static float normalize_degrees(float degrees)
{
    while (degrees >= 360.0f) {
        degrees -= 360.0f;
    }
    while (degrees < 0.0f) {
        degrees += 360.0f;
    }
    return degrees;
}

static float wrap_to_pi_local(float angle)
{
    while (angle > (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < -(float)M_PI) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

static bool nmea_sentence_is(const char *line, const char *sentence)
{
    return line != NULL &&
           sentence != NULL &&
           strlen(line) >= 7 &&
           line[0] == '$' &&
           strncmp(&line[3], sentence, 3) == 0 &&
           line[6] == ',';
}

static uint8_t split_csv_line(char *line, char **fields, uint8_t max_fields)
{
    char *cursor = line;
    uint8_t field_count = 0;

    while (field_count < max_fields && cursor != NULL) {
        char *comma = strchr(cursor, ',');
        fields[field_count++] = cursor;
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
    }

    return field_count;
}

static bool parse_float_field(const char *text, float *value_out)
{
    char *end = NULL;
    float value;

    if (text == NULL || text[0] == '\0' || value_out == NULL) {
        return false;
    }

    value = strtof(text, &end);
    if (end == text) {
        return false;
    }

    *value_out = value;
    return true;
}

void Debug_Log(const char *fmt, ...)
{
    char buffer[384];
    va_list args;

    va_start(args, fmt);
    int length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0) {
        return;
    }

    if ((size_t)length >= sizeof(buffer)) {
        length = (int)sizeof(buffer) - 1;
        buffer[length] = '\0';
    }

    esp_log_write(ESP_LOG_INFO, TAG, "%s", buffer);
}

static const char *GGA_StatusName(uint8_t quality)
{
    switch (quality) {
    case 0: return "INVALID";
    case 1: return "GNSS FIX";
    case 2: return "DGNSS/DGPS";
    case 4: return "RTK FIXED";
    case 5: return "RTK FLOAT";
    case 6: return "ESTIMATED";
    default: return "OTHER";
    }
}

static bool nmea_coordinate_to_degrees(const char *text, char hemisphere, float *degrees_out)
{
    char *end = NULL;
    float raw = strtof(text, &end);

    if (text == NULL || text[0] == '\0' || end == text) {
        return false;
    }

    int degrees = (int)(raw / 100.0f);
    float minutes = raw - ((float)degrees * 100.0f);
    float value = (float)degrees + (minutes / 60.0f);

    if (hemisphere == 'S' || hemisphere == 'W') {
        value = -value;
    } else if (hemisphere != 'N' && hemisphere != 'E') {
        return false;
    }

    *degrees_out = value;
    return true;
}

static void parse_gga_line(char *line, bool print_log)
{
    char *fields[16] = {0};
    uint8_t field_count = 0;
    uint32_t quality_value = 0;
    float latitude = 0.0f;
    float longitude = 0.0f;
    float differential_age = NAN;
    bool latitude_ok = false;
    bool longitude_ok = false;

    field_count = split_csv_line(line, fields, 16);

    if (field_count < 10) {
        if (print_log && RTK_MISC_LOG_ENABLED) {
            ESP_LOGW(TAG, "[UM982] GGA parse error: fields=%u", (unsigned)field_count);
        }
        return;
    }

    quality_value = (uint32_t)strtoul(fields[6], NULL, 10);
    gga_quality = (uint8_t)quality_value;
    last_gga_tick_ms = rtk_millis();
    gga_lines++;

    if (fields[2][0] != '\0' && fields[3][0] != '\0') {
        latitude_ok = nmea_coordinate_to_degrees(fields[2], fields[3][0], &latitude);
    }
    if (fields[4][0] != '\0' && fields[5][0] != '\0') {
        longitude_ok = nmea_coordinate_to_degrees(fields[4], fields[5][0], &longitude);
    }
    if (field_count > 13 && fields[13][0] != '\0') {
        (void)parse_float_field(fields[13], &differential_age);
    }

    if (print_log) {
        ESP_LOGI(TAG,
                 "G,%lu,%s,%.8f,%.8f,%s,%lu,%s,%s,%s,%s",
                 (unsigned long)last_gga_tick_ms,
                 fields[1][0] ? fields[1] : "N/A",
                 latitude_ok ? latitude : NAN,
                 longitude_ok ? longitude : NAN,
                 fields[9][0] ? fields[9] : "nan",
                 (unsigned long)quality_value,
                 fields[7][0] ? fields[7] : "0",
                 fields[8][0] ? fields[8] : "nan",
                 (field_count > 13 && fields[13][0]) ? fields[13] : "nan",
                 GGA_StatusName((uint8_t)quality_value));
    }

    if (latitude_ok && longitude_ok) {
        latest_gga.valid = true;
        latest_gga.latitude = latitude;
        latest_gga.longitude = longitude;
        latest_gga.quality = gga_quality;
        latest_gga.differential_age = differential_age;
        latest_gga.received_ms = last_gga_tick_ms;
    }

    if (latitude_ok && longitude_ok && (gga_quality == 4 || gga_quality == 5)) {
        update_gnss_position(latitude, longitude);
    }
}

static void parse_heading_line(char *line, bool print_log)
{
    char *fields[4] = {0};
    uint8_t field_count = split_csv_line(line, fields, 4);
    float raw_heading_deg = 0.0f;
    float robot_heading_deg;
    float local_psi_deg;

    if (field_count < 2 || !parse_float_field(fields[1], &raw_heading_deg)) {
        if (print_log && RTK_MISC_LOG_ENABLED) {
            ESP_LOGW(TAG, "[HEADING] parse error: %s", line);
        }
        return;
    }

    raw_heading_deg = normalize_degrees(raw_heading_deg);
    robot_heading_deg = normalize_degrees(raw_heading_deg + GNSS_HEADING_OFFSET_DEG);

    // UM982 heading: north=0 deg, clockwise positive.
    // MPPI local psi: east=0 rad, counter-clockwise positive.
    local_psi_deg = 90.0f - robot_heading_deg;

    heading_deg = robot_heading_deg;
    heading_rad = wrap_to_pi_local(local_psi_deg * DEG_TO_RAD);
    heading_valid = true;
    last_heading_tick_ms = rtk_millis();

    if (print_log) {
        ESP_LOGI(TAG,
                 "[HEADING] raw=%.2f deg offset=%.1f deg robot=%.2f deg psi=%.4f rad",
                 raw_heading_deg,
                 GNSS_HEADING_OFFSET_DEG,
                 heading_deg,
                 heading_rad);
    }
}

static void handle_nmea_line(char *line)
{
    uint32_t now = rtk_millis();

    if (nmea_sentence_is(line, "GGA")) {
        bool print_log = (last_gga_log_tick_ms == 0) ||
                         ((uint32_t)(now - last_gga_log_tick_ms) >= 1000U);
        if (print_log) {
            last_gga_log_tick_ms = now;
            ESP_LOGI(TAG, "[UM982 RAW] %s", line);
        }
        parse_gga_line(line, print_log);
    } else if (nmea_sentence_is(line, "HDT") || nmea_sentence_is(line, "THS")) {
        bool print_log = (last_heading_log_tick_ms == 0) ||
                         ((uint32_t)(now - last_heading_log_tick_ms) >= 1000U);
        if (print_log) {
            last_heading_log_tick_ms = now;
            ESP_LOGI(TAG, "[UM982 RAW] %s", line);
        }
        parse_heading_line(line, print_log);
    }
}

static void process_um982_byte(uint8_t byte)
{
    if (byte == '\n') {
        if (nmea_line_length > 0) {
            if (nmea_line[nmea_line_length - 1] == '\r') {
                nmea_line_length--;
            }
            nmea_line[nmea_line_length] = '\0';
            um982_lines++;
            handle_nmea_line(nmea_line);
            nmea_line_length = 0;
        }
        return;
    }

    if (nmea_line_length < (NMEA_LINE_SIZE - 1)) {
        nmea_line[nmea_line_length++] = (char)byte;
    } else {
        nmea_overflows++;
        nmea_line_length = 0;
    }
}

static void um982_uart_task(void *pvParameters)
{
    (void)pvParameters;

    uart_event_t event;
    uint8_t *buffer = malloc(512);
    if (buffer == NULL) {
        if (RTK_MISC_LOG_ENABLED) {
            ESP_LOGE(TAG, "UM982 RX buffer alloc failed");
        }
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (xQueueReceive(um982_uart_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type == UART_DATA) {
            int remaining = event.size;
            while (remaining > 0) {
                int chunk = remaining > 512 ? 512 : remaining;
                int rx_len = uart_read_bytes(UM982_UART_PORT, buffer, chunk, pdMS_TO_TICKS(20));
                if (rx_len <= 0) {
                    break;
                }
                um982_received_bytes += (uint32_t)rx_len;
                for (int i = 0; i < rx_len; i++) {
                    process_um982_byte(buffer[i]);
                }
                remaining -= rx_len;
            }
        } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            um982_rx_overflow_bytes++;
            if (RTK_MISC_LOG_ENABLED) {
                ESP_LOGW(TAG, "[UM982] UART overflow event=%d", event.type);
            }
            uart_flush_input(UM982_UART_PORT);
            xQueueReset(um982_uart_queue);
            nmea_line_length = 0;
        }
    }
}

esp_err_t RTK_Bridge_Init(void)
{
    if (bridge_initialized) {
        return ESP_OK;
    }

    uart_config_t uart_config = {
        .baud_rate = UM982_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(UM982_UART_PORT,
                                            UM982_RX_BUFFER_SIZE,
                                            UM982_TX_BUFFER_SIZE,
                                            UM982_EVENT_QUEUE_SIZE,
                                            &um982_uart_queue,
                                            0),
                        TAG, "UM982 uart driver install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(UM982_UART_PORT, &uart_config),
                        TAG, "UM982 uart param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(UM982_UART_PORT,
                                     UM982_TX_PIN,
                                     UM982_RX_PIN,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG, "UM982 uart pin config failed");

    nmea_line_length = 0;
    um982_lines = 0;
    gga_lines = 0;
    nmea_overflows = 0;
    gga_quality = 0;
    last_gga_tick_ms = 0;
    heading_deg = 0.0f;
    heading_rad = 0.0f;
    heading_valid = false;
    last_heading_tick_ms = 0;
    last_gga_log_tick_ms = 0;
    last_heading_log_tick_ms = 0;
    memset(&latest_gga, 0, sizeof(latest_gga));
    latest_gga.differential_age = NAN;

    BaseType_t ok = xTaskCreate(um982_uart_task, "um982_uart_task", 4096, NULL, 4, NULL);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    bridge_initialized = true;
    if (RTK_MISC_LOG_ENABLED) {
        ESP_LOGI(TAG, "[UART] UM982 ESP_TX=%d ESP_RX=%d, %d 8N1; USB debug 115200",
                 UM982_TX_PIN, UM982_RX_PIN, UM982_BAUD_RATE);
    }
    return ESP_OK;
}

bool RTK_Bridge_ForwardRTCM(const uint8_t *frame, uint16_t length)
{
    if (!bridge_initialized || frame == NULL || length == 0) {
        dropped_frames++;
        return false;
    }

    int written = uart_write_bytes(UM982_UART_PORT, frame, length);
    if (written != length) {
        dropped_frames++;
        if (RTK_MISC_LOG_ENABLED) {
            ESP_LOGW(TAG, "[UM982 TX] DROP/PARTIAL: requested=%u written=%d",
                     (unsigned)length, written);
        }
        return false;
    }

    forwarded_frames++;
    forwarded_bytes += length;
    return true;
}

uint32_t RTK_Bridge_GetForwardedFrames(void)
{
    return forwarded_frames;
}

uint32_t RTK_Bridge_GetForwardedBytes(void)
{
    return forwarded_bytes;
}

uint32_t RTK_Bridge_GetDroppedFrames(void)
{
    return dropped_frames;
}

uint32_t RTK_Bridge_GetUM982Bytes(void)
{
    return um982_received_bytes;
}

uint32_t RTK_Bridge_GetUM982Lines(void)
{
    return um982_lines;
}

uint32_t RTK_Bridge_GetGGALines(void)
{
    return gga_lines;
}

uint32_t RTK_Bridge_GetNMEAOverflows(void)
{
    return nmea_overflows;
}

uint32_t RTK_Bridge_GetUM982RxOverflowBytes(void)
{
    return um982_rx_overflow_bytes;
}

uint8_t RTK_Bridge_GetGGAQuality(void)
{
    return gga_quality;
}

bool RTK_Bridge_HasRTK(void)
{
    return gga_quality == 4 || gga_quality == 5;
}

uint32_t RTK_Bridge_GetGGAAgeMs(void)
{
    if (last_gga_tick_ms == 0) {
        return UINT32_MAX;
    }
    return rtk_millis() - last_gga_tick_ms;
}

bool RTK_Bridge_GetLatestGGA(RTK_Bridge_GGA *gga_out)
{
    if (gga_out == NULL || !latest_gga.valid) {
        return false;
    }

    *gga_out = latest_gga;
    return true;
}

bool RTK_Bridge_GetHeadingRad(float *heading_rad_out)
{
    if (!heading_valid || heading_rad_out == NULL) {
        return false;
    }

    *heading_rad_out = heading_rad;
    return true;
}

float RTK_Bridge_GetHeadingDeg(void)
{
    return heading_deg;
}

bool RTK_Bridge_HasHeading(void)
{
    return heading_valid;
}

uint32_t RTK_Bridge_GetHeadingAgeMs(void)
{
    if (!heading_valid || last_heading_tick_ms == 0) {
        return UINT32_MAX;
    }
    return rtk_millis() - last_heading_tick_ms;
}

bool RTK_Bridge_HasFreshRTK(uint32_t max_age_ms)
{
    return RTK_Bridge_HasRTK() && RTK_Bridge_GetGGAAgeMs() <= max_age_ms;
}

bool RTK_Bridge_HasFreshHeading(uint32_t max_age_ms)
{
    return heading_valid && RTK_Bridge_GetHeadingAgeMs() <= max_age_ms;
}
