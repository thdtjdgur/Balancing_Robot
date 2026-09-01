#include "rtk_bridge.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gnss.h"

#define UM982_RX_BUFFER_SIZE 4096
#define UM982_TX_BUFFER_SIZE 4096
#define UM982_EVENT_QUEUE_SIZE 20
#define NMEA_LINE_SIZE 384

static const char *TAG = "RTK_BRIDGE";

static QueueHandle_t um982_uart_queue;
static bool bridge_initialized;

static uint32_t forwarded_frames;
static uint32_t forwarded_bytes;
static uint32_t dropped_frames;
static uint32_t um982_received_bytes;
static uint32_t um982_rx_overflow_bytes;

static char nmea_line[NMEA_LINE_SIZE];
static size_t nmea_line_length;
static uint8_t gga_quality;

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

static void parse_gga_line(char *line)
{
    char raw[NMEA_LINE_SIZE];
    char *fields[16] = {0};
    char *cursor = line;
    uint8_t field_count = 0;
    uint32_t quality_value = 0;
    float latitude = 0.0f;
    float longitude = 0.0f;
    bool latitude_ok = false;
    bool longitude_ok = false;

    snprintf(raw, sizeof(raw), "%s", line);

    while (field_count < 16 && cursor != NULL) {
        char *comma = strchr(cursor, ',');
        fields[field_count++] = cursor;
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
    }

    if (field_count < 10) {
        ESP_LOGW(TAG, "GGA parse error: too few fields (%u)", (unsigned)field_count);
        return;
    }

    quality_value = (uint32_t)strtoul(fields[6], NULL, 10);
    gga_quality = (uint8_t)quality_value;

    if (fields[2][0] != '\0' && fields[3][0] != '\0') {
        latitude_ok = nmea_coordinate_to_degrees(fields[2], fields[3][0], &latitude);
    }
    if (fields[4][0] != '\0' && fields[5][0] != '\0') {
        longitude_ok = nmea_coordinate_to_degrees(fields[4], fields[5][0], &longitude);
    }

    ESP_LOGI(TAG,
             "GGA UTC=%s Lat=%.8f Lon=%.8f Alt=%s m Quality=%lu (%s) Sats=%s HDOP=%s DiffAge=%s s",
             fields[1][0] ? fields[1] : "N/A",
             latitude_ok ? latitude : 0.0f,
             longitude_ok ? longitude : 0.0f,
             fields[9][0] ? fields[9] : "N/A",
             (unsigned long)quality_value,
             GGA_StatusName((uint8_t)quality_value),
             fields[7][0] ? fields[7] : "N/A",
             fields[8][0] ? fields[8] : "N/A",
             (field_count > 13 && fields[13][0]) ? fields[13] : "N/A");

    //if (latitude_ok && longitude_ok && (gga_quality == 4 || gga_quality == 5)) {
    if (latitude_ok && longitude_ok && gga_quality == 4) {
        update_gnss_position(latitude, longitude);
    }
}

static void handle_nmea_line(char *line)
{
    if (strncmp(line, "$GNGGA,", 7) != 0 && strncmp(line, "$GPGGA,", 7) != 0) {
        return;
    }

    ESP_LOGI(TAG, "UM982 RAW %s", line);
    parse_gga_line(line);
}

static void process_um982_byte(uint8_t byte)
{
    if (byte == '\n') {
        if (nmea_line_length > 0) {
            if (nmea_line[nmea_line_length - 1] == '\r') {
                nmea_line_length--;
            }
            nmea_line[nmea_line_length] = '\0';
            handle_nmea_line(nmea_line);
            nmea_line_length = 0;
        }
        return;
    }

    if (nmea_line_length < (NMEA_LINE_SIZE - 1)) {
        nmea_line[nmea_line_length++] = (char)byte;
    } else {
        nmea_line_length = 0;
    }
}

static void um982_uart_task(void *pvParameters)
{
    (void)pvParameters;

    uart_event_t event;
    uint8_t *buffer = malloc(512);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "UM982 RX buffer alloc failed");
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
            ESP_LOGW(TAG, "UM982 UART overflow event=%d", event.type);
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
    gga_quality = 0;

    BaseType_t ok = xTaskCreate(um982_uart_task, "um982_uart_task", 4096, NULL, 4, NULL);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    bridge_initialized = true;
    ESP_LOGI(TAG, "UM982 UART2 init done: TX=%d RX=%d baud=%d",
             UM982_TX_PIN, UM982_RX_PIN, UM982_BAUD_RATE);
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
        ESP_LOGW(TAG, "UM982 RTCM write dropped: requested=%u written=%d",
                 (unsigned)length, written);
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
