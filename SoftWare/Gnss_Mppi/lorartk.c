#include "lorartk.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gnss.h"
#include "lora.h"
#include "rtk_bridge.h"

#define TTGO_PACKET_SINGLE        0xA1U
#define TTGO_PACKET_FRAGMENT      0xA2U
#define TTGO_PACKET_DOWNLINK_END  0xA3U
#define GROUND_STATION_ID         0xFFU
#define WAYPOINT_PACKET_TYPE      0xF2U
#define DRONE_PACKET_ID           0xFEU
#define DRONE_GPS_MESSAGE         0xF3U
#define DRONE_GPS_POSITION_COUNT  1U
#define DRONE_GPS_PACKET_SIZE     14U
#define TTGO_SINGLE_HEADER_SIZE   2U
#define TTGO_FRAGMENT_HEADER_SIZE 4U
#define WAYPOINT_HEADER_SIZE      3U
#define WAYPOINT_COORD_SIZE       8U
#define WAYPOINT_COORD_SCALE      10000000.0f
#define GROUND_MAX_WAYPOINTS      15U
#define RTCM_MAX_FRAME_SIZE       1029U
#define FRAGMENT_TIMEOUT_MS       1500U
#define GPS_RESPONSE_GUARD_MS     30U
#define LORA_TX_TIMEOUT_MS        2000U
#define UM982_EXPECTED_GGA_HZ     10U

typedef struct {
    bool active;
    uint8_t sequence;
    uint8_t fragment_count;
    uint8_t next_fragment;
    uint16_t length;
    uint32_t last_fragment_tick;
    uint8_t frame[RTCM_MAX_FRAME_SIZE];
} ReassemblyState;

typedef struct {
    uint32_t lora_packets;
    uint32_t lora_bytes;
    uint32_t lora_phy_crc_errors;
    uint32_t completed_frames;
    uint32_t completed_bytes;
    uint32_t rtcm_crc_errors;
    uint32_t reassembly_errors;
    uint32_t fragment_timeouts;
    uint32_t sequence_misses;
    uint32_t duplicate_packets;
    uint32_t unsupported_packets;
    uint32_t waypoint_packets;
    uint32_t waypoint_errors;
    uint32_t downlink_end_packets;
    uint32_t gps_responses;
    uint32_t gps_response_errors;
    uint32_t rtcm_1005;
    uint32_t rtcm_1006;
    uint32_t rtcm_1077;
    uint32_t rtcm_1087;
    uint32_t rtcm_1097;
    uint32_t rtcm_1127;
    uint32_t rtcm_1230;
    uint32_t rtcm_other;
} Statistics;

static const char *TAG = "LORARTK";

static ReassemblyState reassembly;
static Statistics total;
static Statistics interval;
static bool sequence_initialized;
static uint8_t expected_sequence;
static uint32_t statistics_tick;
static uint32_t totals_tick;
static uint32_t last_forwarded_frames;
static uint32_t last_forwarded_bytes;
static uint32_t last_uart_drops;
static uint32_t last_um982_received_bytes;
static uint32_t last_um982_lines;
static uint32_t last_gga_lines;
static uint8_t gps_sequence;
static uint8_t latest_detected_flag;
static uint8_t latest_person_count;

static uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static uint32_t CRC24Q(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc <<= 1;
            if ((crc & 0x1000000U) != 0U) {
                crc ^= 0x1864CFBU;
            }
        }
    }

    return crc & 0xFFFFFFU;
}

static void reset_reassembly(void)
{
    reassembly.active = false;
    reassembly.length = 0;
    reassembly.next_fragment = 0;
    reassembly.fragment_count = 0;
}

static bool accept_frame_sequence(uint8_t sequence)
{
    if (!sequence_initialized) {
        sequence_initialized = true;
        expected_sequence = sequence;
    }

    if (sequence == expected_sequence) {
        expected_sequence = (uint8_t)(expected_sequence + 1U);
        return true;
    }

    if (sequence == (uint8_t)(expected_sequence - 1U)) {
        total.duplicate_packets++;
        interval.duplicate_packets++;
        ESP_LOGD(TAG, "duplicate frame ignored: %u", (unsigned)sequence);
        return false;
    }

    total.sequence_misses++;
    interval.sequence_misses++;
        ESP_LOGW(TAG, "[SEQ] Miss/out-of-order: RX=%u Expected=%u",
             (unsigned)sequence, (unsigned)expected_sequence);
    expected_sequence = (uint8_t)(sequence + 1U);
    return true;
}

static void count_rtcm_type(uint16_t type)
{
    uint32_t *total_counter = &total.rtcm_other;
    uint32_t *interval_counter = &interval.rtcm_other;

    switch (type) {
    case 1005: total_counter = &total.rtcm_1005; interval_counter = &interval.rtcm_1005; break;
    case 1006: total_counter = &total.rtcm_1006; interval_counter = &interval.rtcm_1006; break;
    case 1077: total_counter = &total.rtcm_1077; interval_counter = &interval.rtcm_1077; break;
    case 1087: total_counter = &total.rtcm_1087; interval_counter = &interval.rtcm_1087; break;
    case 1097: total_counter = &total.rtcm_1097; interval_counter = &interval.rtcm_1097; break;
    case 1127: total_counter = &total.rtcm_1127; interval_counter = &interval.rtcm_1127; break;
    case 1230: total_counter = &total.rtcm_1230; interval_counter = &interval.rtcm_1230; break;
    default: break;
    }

    (*total_counter)++;
    (*interval_counter)++;
}

static int32_t read_i32_le(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0]) |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24);
    return (int32_t)value;
}

static void write_i32_le(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static void drain_lora_irq_flags(void)
{
    while (lora_take_rx_flag()) {
    }
}

static void count_waypoint_error(const char *reason)
{
    total.waypoint_errors++;
    interval.waypoint_errors++;
    ESP_LOGW(TAG, "[WAYPOINT RX] rejected: %s", reason);
}

static void process_waypoint_packet(const uint8_t *packet, uint8_t length)
{
    uint8_t count;
    uint16_t expected_length;
    float station_packet[1 + (MAX_WAYPOINTS * 2)];

    if (length < WAYPOINT_HEADER_SIZE) {
        count_waypoint_error("too short");
        return;
    }

    count = packet[2];
    if (count == 0 || count > GROUND_MAX_WAYPOINTS || count > MAX_WAYPOINTS) {
        count_waypoint_error("invalid count");
        return;
    }

    expected_length = (uint16_t)(WAYPOINT_HEADER_SIZE + ((uint16_t)count * WAYPOINT_COORD_SIZE));
    if (length != expected_length) {
        count_waypoint_error("length mismatch");
        ESP_LOGW(TAG, "[WAYPOINT RX] rejected: count=%u length=%u expected=%u",
                 (unsigned)count, (unsigned)length, (unsigned)expected_length);
        return;
    }

    station_packet[0] = (float)count;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *coord = &packet[WAYPOINT_HEADER_SIZE + ((uint16_t)i * WAYPOINT_COORD_SIZE)];
        float latitude = (float)read_i32_le(coord) / WAYPOINT_COORD_SCALE;
        float longitude = (float)read_i32_le(coord + 4) / WAYPOINT_COORD_SCALE;

        if (latitude < -90.0f || latitude > 90.0f ||
            longitude < -180.0f || longitude > 180.0f) {
            count_waypoint_error("coordinate range");
            ESP_LOGW(TAG, "[WAYPOINT RX] invalid coord index=%u lat=%.7f lon=%.7f",
                     (unsigned)i, latitude, longitude);
            return;
        }

        station_packet[1 + ((uint16_t)i * 2U)] = latitude;
        station_packet[1 + ((uint16_t)i * 2U) + 1U] = longitude;
    }

    gnss_receive_complete(station_packet, 1 + ((int)count * 2));
    total.waypoint_packets++;
    interval.waypoint_packets++;

    ESP_LOGI(TAG, "[WAYPOINT RX] parsed/stored count=%u length=%u",
             (unsigned)count, (unsigned)length);

    char waypoint_log[512];
    int used = snprintf(waypoint_log, sizeof(waypoint_log), "W,%lu,%u",
                        (unsigned long)millis(), (unsigned)count);
    for (uint8_t i = 0; i < count && used > 0 && used < (int)sizeof(waypoint_log); i++) {
        used += snprintf(&waypoint_log[used], sizeof(waypoint_log) - (size_t)used,
                         ",%.7f,%.7f",
                         station_packet[1 + ((uint16_t)i * 2U)],
                         station_packet[1 + ((uint16_t)i * 2U) + 1U]);
    }
    ESP_LOGI(TAG, "%s", waypoint_log);
}

static void send_latest_gga_to_ground(void)
{
    RTK_Bridge_GGA gga;
    uint8_t packet[DRONE_GPS_PACKET_SIZE] = {0};
    int32_t latitude_scaled;
    int32_t longitude_scaled;
    bool sent;

    if (!RTK_Bridge_GetLatestGGA(&gga)) {
        total.gps_response_errors++;
        interval.gps_response_errors++;
        ESP_LOGW(TAG, "[GPS TX] skipped: no valid GGA stored");
        rx_set();
        ESP_LOGW(TAG, "[GPS TX] skipped -> RX");
        return;
    }

    latitude_scaled = (int32_t)lroundf(gga.latitude * WAYPOINT_COORD_SCALE);
    longitude_scaled = (int32_t)lroundf(gga.longitude * WAYPOINT_COORD_SCALE);

    packet[0] = DRONE_PACKET_ID;
    packet[1] = gps_sequence++;
    packet[2] = DRONE_GPS_MESSAGE;
    packet[3] = DRONE_GPS_POSITION_COUNT;
    write_i32_le(&packet[4], latitude_scaled);
    write_i32_le(&packet[8], longitude_scaled);
    packet[12] = latest_detected_flag;
    packet[13] = latest_person_count;

    vTaskDelay(pdMS_TO_TICKS(GPS_RESPONSE_GUARD_MS));
    drain_lora_irq_flags();
    sent = lora_send_packet(packet, DRONE_GPS_PACKET_SIZE, LORA_TX_TIMEOUT_MS);
    drain_lora_irq_flags();
    rx_set();

    if (sent) {
        uint32_t sample_age_ms = millis() - gga.received_ms;

        total.gps_responses++;
        interval.gps_responses++;
        if (isfinite(gga.differential_age)) {
            ESP_LOGI(TAG,
                     "[GPS TX] lat=%.8f, lon=%.8f, quality=%u, age=%.2f s, sample_age=%" PRIu32 " ms, det=%u, count=%u",
                     gga.latitude,
                     gga.longitude,
                     (unsigned)gga.quality,
                     gga.differential_age,
                     sample_age_ms,
                     (unsigned)latest_detected_flag,
                     (unsigned)latest_person_count);
        } else {
            ESP_LOGI(TAG,
                     "[GPS TX] lat=%.8f, lon=%.8f, quality=%u, age=nan, sample_age=%" PRIu32 " ms, det=%u, count=%u",
                     gga.latitude,
                     gga.longitude,
                     (unsigned)gga.quality,
                     sample_age_ms,
                     (unsigned)latest_detected_flag,
                     (unsigned)latest_person_count);
        }
        ESP_LOGI(TAG, "[GPS TX] done -> RX");
    } else {
        total.gps_response_errors++;
        interval.gps_response_errors++;
        ESP_LOGW(TAG, "[GPS TX] failed -> RX");
    }
}

static void log_rtcm_result(uint8_t sequence, uint16_t rtcm_type,
                            uint16_t frame_length, bool crc_ok,
                            uint8_t fragment_count, bool forwarded,
                            const char *status)
{
    ESP_LOGI(TAG, "T,%lu,%u,%u,%u,%u,%u,%u,%s",
             (unsigned long)millis(),
             (unsigned)sequence,
             (unsigned)rtcm_type,
             (unsigned)frame_length,
             crc_ok ? 1U : 0U,
             (unsigned)fragment_count,
             forwarded ? 1U : 0U,
             status);
}

static void validate_and_forward_rtcm(const uint8_t *frame, uint16_t length,
                                      uint8_t sequence, uint8_t fragment_count)
{
    uint16_t payload_length;
    uint16_t expected_length;
    uint16_t message_type = 0;
    uint32_t received_crc;
    uint32_t calculated_crc;

    total.completed_frames++;
    interval.completed_frames++;
    total.completed_bytes += length;
    interval.completed_bytes += length;

    if (length >= 5) {
        message_type = (uint16_t)(((uint16_t)frame[3] << 4) | (frame[4] >> 4));
    }

    if (length < 8 || frame[0] != 0xD3 || ((frame[1] & 0xFCU) != 0U)) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[RTCM] Invalid header/preamble, length=%u", (unsigned)length);
        log_rtcm_result(sequence, message_type, length, false, fragment_count, false, "FORMAT_ERROR");
        return;
    }

    payload_length = (uint16_t)(((uint16_t)(frame[1] & 0x03U) << 8) | frame[2]);
    expected_length = (uint16_t)(payload_length + 6U);
    if (length != expected_length) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[RTCM] Length mismatch: received=%u header=%u",
                 (unsigned)length, (unsigned)expected_length);
        log_rtcm_result(sequence, message_type, length, false, fragment_count, false, "LENGTH_ERROR");
        return;
    }

    received_crc = ((uint32_t)frame[length - 3U] << 16) |
                   ((uint32_t)frame[length - 2U] << 8) |
                   frame[length - 1U];
    calculated_crc = CRC24Q(frame, (uint16_t)(length - 3U));
    if (received_crc != calculated_crc) {
        total.rtcm_crc_errors++;
        interval.rtcm_crc_errors++;
        ESP_LOGW(TAG, "[RTCM] Type=%u Length=%u CRC=ERROR rx=%06" PRIX32 " calc=%06" PRIX32,
                 (unsigned)message_type, (unsigned)length, received_crc, calculated_crc);
        log_rtcm_result(sequence, message_type, length, false, fragment_count, false, "CRC_ERROR");
        return;
    }

    count_rtcm_type(message_type);
    ESP_LOGI(TAG, "[RTCM] Type=%u Frame Length=%u CRC=OK",
             (unsigned)message_type, (unsigned)length);
    if (RTK_Bridge_ForwardRTCM(frame, length)) {
        ESP_LOGI(TAG, "[UM982 TX] Queued RTCM type=%u, %u bytes",
                 (unsigned)message_type, (unsigned)length);
        log_rtcm_result(sequence, message_type, length, true, fragment_count, true, "OK");
    } else {
        ESP_LOGW(TAG, "[UM982 TX] DROP/PARTIAL: type=%u, %u bytes",
                 (unsigned)message_type, (unsigned)length);
        log_rtcm_result(sequence, message_type, length, true, fragment_count, false, "UART_DROP");
    }
}

static void process_single_packet(const uint8_t *packet, uint8_t length)
{
    uint8_t sequence;

    if (length <= TTGO_SINGLE_HEADER_SIZE) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        return;
    }

    sequence = packet[1];
    if (!accept_frame_sequence(sequence)) {
        return;
    }

    if (reassembly.active) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[REASM] Incomplete sequence %u discarded by single %u",
                 (unsigned)reassembly.sequence, (unsigned)sequence);
        reset_reassembly();
    }

    ESP_LOGI(TAG, "[LORA] Single sequence=%u payload=%u",
             (unsigned)sequence,
             (unsigned)(length - TTGO_SINGLE_HEADER_SIZE));

    validate_and_forward_rtcm(&packet[TTGO_SINGLE_HEADER_SIZE],
                              (uint16_t)(length - TTGO_SINGLE_HEADER_SIZE),
                              sequence,
                              1);
}

static void process_fragment_packet(const uint8_t *packet, uint8_t length)
{
    uint8_t sequence;
    uint8_t index;
    uint8_t count;
    uint16_t data_length;

    if (length <= TTGO_FRAGMENT_HEADER_SIZE) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        return;
    }

    sequence = packet[1];
    index = packet[2];
    count = packet[3];
    data_length = (uint16_t)(length - TTGO_FRAGMENT_HEADER_SIZE);

    ESP_LOGI(TAG, "[FRAG] Sequence=%u Index=%u/%u Data=%u",
             (unsigned)sequence, (unsigned)index, (unsigned)count, (unsigned)data_length);

    if (count < 2 || index >= count) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[FRAG] Invalid header");
        return;
    }

    if (index == 0) {
        if (reassembly.active) {
            if (reassembly.sequence == sequence && reassembly.next_fragment > 0) {
                total.duplicate_packets++;
                interval.duplicate_packets++;
                ESP_LOGD(TAG, "[FRAG] Duplicate first fragment ignored");
                return;
            }
            total.reassembly_errors++;
            interval.reassembly_errors++;
            ESP_LOGW(TAG, "[REASM] Previous sequence=%u discarded", (unsigned)reassembly.sequence);
            reset_reassembly();
        }

        if (!accept_frame_sequence(sequence)) {
            return;
        }

        reassembly.active = true;
        reassembly.sequence = sequence;
        reassembly.fragment_count = count;
        reassembly.next_fragment = 0;
        reassembly.length = 0;
        ESP_LOGI(TAG, "[REASM] Start sequence=%u fragments=%u timeout=%lu ms",
                 (unsigned)sequence,
                 (unsigned)count,
                 (unsigned long)FRAGMENT_TIMEOUT_MS);
    } else if (!reassembly.active || reassembly.sequence != sequence) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[FRAG] No active frame for seq=%u index=%u",
                 (unsigned)sequence, (unsigned)index);
        return;
    }

    if (count != reassembly.fragment_count) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[FRAG] Fragment-count changed (%u -> %u)",
                 (unsigned)reassembly.fragment_count, (unsigned)count);
        reset_reassembly();
        return;
    }
    if (index < reassembly.next_fragment) {
        total.duplicate_packets++;
        interval.duplicate_packets++;
        ESP_LOGD(TAG, "[FRAG] Duplicate fragment ignored: %u", (unsigned)index);
        return;
    }
    if (index != reassembly.next_fragment) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[FRAG] Missing/out-of-order: RX=%u Expected=%u",
                 (unsigned)index, (unsigned)reassembly.next_fragment);
        reset_reassembly();
        return;
    }
    if ((uint32_t)reassembly.length + data_length > RTCM_MAX_FRAME_SIZE) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "[FRAG] Reassembly buffer overflow");
        reset_reassembly();
        return;
    }

    memcpy(&reassembly.frame[reassembly.length], &packet[TTGO_FRAGMENT_HEADER_SIZE], data_length);
    reassembly.length = (uint16_t)(reassembly.length + data_length);
    reassembly.next_fragment++;
    reassembly.last_fragment_tick = millis();

    if (reassembly.next_fragment == reassembly.fragment_count) {
        ESP_LOGI(TAG, "[REASM] Complete sequence=%u fragments=%u length=%u",
                 (unsigned)reassembly.sequence,
                 (unsigned)reassembly.fragment_count,
                 (unsigned)reassembly.length);
        validate_and_forward_rtcm(reassembly.frame,
                                  reassembly.length,
                                  reassembly.sequence,
                                  reassembly.fragment_count);
        reset_reassembly();
    }
}

static void process_lora_packet(void)
{
    uint8_t packet[255];
    uint8_t length = 0;
    uint8_t irq = 0;
    int result = lora_receive_packet(packet, &length, &irq);

    rx_set();

    if (result < 0) {
        total.lora_phy_crc_errors++;
        interval.lora_phy_crc_errors++;
        ESP_LOGW(TAG, "[LORA] Payload CRC error, IRQ=0x%02X", irq);
        return;
    }
    if (result == 0) {
        ESP_LOGD(TAG, "LoRa event without RxDone irq=0x%02X", irq);
        return;
    }

    total.lora_packets++;
    interval.lora_packets++;
    total.lora_bytes += length;
    interval.lora_bytes += length;

    ESP_LOGI(TAG, "[LORA] RX length=%u RSSI=%d dBm SNRx100=%d",
             (unsigned)length, lora_packet_rssi_dbm(), lora_packet_snr_x100());

    if (length == 0) {
        total.unsupported_packets++;
        interval.unsupported_packets++;
    } else if (packet[0] == TTGO_PACKET_SINGLE) {
        process_single_packet(packet, length);
    } else if (packet[0] == TTGO_PACKET_FRAGMENT) {
        process_fragment_packet(packet, length);
    } else if (packet[0] == TTGO_PACKET_DOWNLINK_END) {
        total.downlink_end_packets++;
        interval.downlink_end_packets++;
        ESP_LOGI(TAG, "[DOWNLINK_END] received");
        send_latest_gga_to_ground();
    } else if (length >= 2 &&
               packet[0] == GROUND_STATION_ID &&
               packet[1] == WAYPOINT_PACKET_TYPE) {
        process_waypoint_packet(packet, length);
    } else {
        total.unsupported_packets++;
        interval.unsupported_packets++;
        ESP_LOGW(TAG, "[LORA] Unsupported packet type=0x%02X", packet[0]);
    }
}

static void print_one_second_statistics(void)
{
    uint32_t forwarded_frames = RTK_Bridge_GetForwardedFrames();
    uint32_t forwarded_bytes = RTK_Bridge_GetForwardedBytes();
    uint32_t uart_drops = RTK_Bridge_GetDroppedFrames();
    uint32_t um982_bytes = RTK_Bridge_GetUM982Bytes();
    uint32_t um982_lines = RTK_Bridge_GetUM982Lines();
    uint32_t gga_lines = RTK_Bridge_GetGGALines();
    uint32_t nmea_overflows = RTK_Bridge_GetNMEAOverflows();
    uint32_t interval_um982_bytes = um982_bytes - last_um982_received_bytes;
    uint32_t interval_um982_lines = um982_lines - last_um982_lines;
    uint32_t interval_gga_lines = gga_lines - last_gga_lines;
    uint32_t gga_age_ms = RTK_Bridge_GetGGAAgeMs();
    bool gga_rate_ok = interval_gga_lines >= (UM982_EXPECTED_GGA_HZ - 1U) &&
                       interval_gga_lines <= (UM982_EXPECTED_GGA_HZ + 1U);

    ESP_LOGI(TAG,
             "[1s] LoRa=%" PRIu32 " pkt/%" PRIu32 " B RTCM=%" PRIu32 " frame/%" PRIu32 " B FWD=%" PRIu32 "/%" PRIu32 " B "
             "CRCerr=%" PRIu32 " PHYerr=%" PRIu32 " ReasmErr=%" PRIu32 " SeqMiss=%" PRIu32 " Dup=%" PRIu32 " "
             "FilterDrop=0 UARTdrop=%" PRIu32 " UM982=%" PRIu32 " B/s,%" PRIu32 " lines/s "
             "GGA=%" PRIu32 " Hz(%s),age=%" PRIu32 " ms,NMEAovf=%" PRIu32 " "
             "GGAq=%u Hdg=%.1f HAge=%" PRIu32 " ms WP=%" PRIu32 " WPErr=%" PRIu32 " A3=%" PRIu32 " GPS_TX=%" PRIu32 " GPSErr=%" PRIu32,
             interval.lora_packets,
             interval.lora_bytes,
             interval.completed_frames,
             interval.completed_bytes,
             forwarded_frames - last_forwarded_frames,
             forwarded_bytes - last_forwarded_bytes,
             interval.rtcm_crc_errors,
             interval.lora_phy_crc_errors,
             interval.reassembly_errors,
             interval.sequence_misses,
             interval.duplicate_packets,
             uart_drops - last_uart_drops,
             interval_um982_bytes,
             interval_um982_lines,
             interval_gga_lines,
             gga_rate_ok ? "OK" : "WARN",
             gga_age_ms,
             nmea_overflows,
             (unsigned)RTK_Bridge_GetGGAQuality(),
             RTK_Bridge_GetHeadingDeg(),
             RTK_Bridge_GetHeadingAgeMs(),
             interval.waypoint_packets,
             interval.waypoint_errors,
             interval.downlink_end_packets,
             interval.gps_responses,
             interval.gps_response_errors);

    last_forwarded_frames = forwarded_frames;
    last_forwarded_bytes = forwarded_bytes;
    last_uart_drops = uart_drops;
    last_um982_received_bytes = um982_bytes;
    last_um982_lines = um982_lines;
    last_gga_lines = gga_lines;
    memset(&interval, 0, sizeof(interval));
}

static void print_ten_second_totals(void)
{
    ESP_LOGI(TAG,
             "[TOTAL] LoRa=%" PRIu32 " pkt/%" PRIu32 " B RTCM=%" PRIu32 " frame/%" PRIu32 " B Forwarded=%" PRIu32 " "
             "frame/%" PRIu32 " B UM982_RX=%" PRIu32 " B GGA=%" PRIu32 " Lines=%" PRIu32 " NMEAovf=%" PRIu32 " "
             "CRCerr=%" PRIu32 " Timeout=%" PRIu32 " SeqMiss=%" PRIu32 " FilterDrop=0 Unsupported=%" PRIu32 " "
             "WP=%" PRIu32 " WPErr=%" PRIu32 " A3=%" PRIu32 " GPS_TX=%" PRIu32 " GPSErr=%" PRIu32,
             total.lora_packets,
             total.lora_bytes,
             total.completed_frames,
             total.completed_bytes,
             RTK_Bridge_GetForwardedFrames(),
             RTK_Bridge_GetForwardedBytes(),
             RTK_Bridge_GetUM982Bytes(),
             RTK_Bridge_GetGGALines(),
             RTK_Bridge_GetUM982Lines(),
             RTK_Bridge_GetNMEAOverflows(),
             total.rtcm_crc_errors,
             total.fragment_timeouts,
             total.sequence_misses,
             total.unsupported_packets,
             total.waypoint_packets,
             total.waypoint_errors,
             total.downlink_end_packets,
             total.gps_responses,
             total.gps_response_errors);
    ESP_LOGI(TAG,
             "[TYPE] 1005=%" PRIu32 " 1006=%" PRIu32 " 1077=%" PRIu32 " 1087=%" PRIu32 " 1097=%" PRIu32 " 1127=%" PRIu32 " 1230=%" PRIu32 " other=%" PRIu32,
             total.rtcm_1005,
             total.rtcm_1006,
             total.rtcm_1077,
             total.rtcm_1087,
             total.rtcm_1097,
             total.rtcm_1127,
             total.rtcm_1230,
             total.rtcm_other);
}

static void lorartk_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        uint32_t now = millis();

        if (lora_take_rx_flag() || lora_irq_pending()) {
            process_lora_packet();
        }

        now = millis();
        if (reassembly.active && (uint32_t)(now - reassembly.last_fragment_tick) >= FRAGMENT_TIMEOUT_MS) {
            total.fragment_timeouts++;
            interval.fragment_timeouts++;
            total.reassembly_errors++;
            interval.reassembly_errors++;
            ESP_LOGW(TAG, "[REASM] Timeout sequence=%u received=%u/%u length=%u",
                     (unsigned)reassembly.sequence,
                     (unsigned)reassembly.next_fragment,
                     (unsigned)reassembly.fragment_count,
                     (unsigned)reassembly.length);
            log_rtcm_result(reassembly.sequence, 0, reassembly.length, false,
                            reassembly.fragment_count, false, "TIMEOUT");
            reset_reassembly();
        }

        if ((uint32_t)(now - statistics_tick) >= 1000U) {
            statistics_tick += 1000U;
            print_one_second_statistics();
        }
        if ((uint32_t)(now - totals_tick) >= 10000U) {
            totals_tick += 10000U;
            print_ten_second_totals();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void init_lorartk(void)
{
    memset(&reassembly, 0, sizeof(reassembly));
    memset(&total, 0, sizeof(total));
    memset(&interval, 0, sizeof(interval));
    sequence_initialized = false;
    expected_sequence = 0;
    last_forwarded_frames = 0;
    last_forwarded_bytes = 0;
    last_uart_drops = 0;
    last_um982_received_bytes = 0;
    last_um982_lines = 0;
    last_gga_lines = 0;
    gps_sequence = 0;
    latest_detected_flag = 0;
    latest_person_count = 0;

    esp_err_t err = RTK_Bridge_Init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RTK bridge init failed: %s", esp_err_to_name(err));
        return;
    }

    err = lora_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed: %s", esp_err_to_name(err));
        return;
    }

    statistics_tick = millis();
    totals_tick = statistics_tick;

    ESP_LOGI(TAG, "=== LORARTK ESP32-S3 Drone Receiver ===");
    ESP_LOGI(TAG, "[PIN] CS=%d SCK=%d MISO=%d MOSI=%d EN=%d RST=%d DIO0=%d",
             LORA_PIN_CS, LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI,
             LORA_PIN_EN, LORA_PIN_RST, LORA_PIN_DIO0);
    ESP_LOGI(TAG,
             "[RADIO] SX1276 version=0x%02X, 922.1 MHz, SF7, BW125, CR4/5, Explicit, CRC ON",
             lora_read(0x42));
    ESP_LOGI(TAG,
             "[PACKET] A1=[type,seq,RTCM], A2=[type,seq,index,count,data], A3=DOWNLINK_END, Waypoint=[FF,F2,count,lat/lon...], uplink=[FE,seq,F3,1,lat,lon,det,count]");
    ESP_LOGI(TAG, "[UART] UM982 ESP_TX=%d ESP_RX=%d, %d 8N1; USB debug 115200",
             UM982_TX_PIN, UM982_RX_PIN, UM982_BAUD_RATE);
    ESP_LOGI(TAG, "[GNSS] Expected GGA=%u Hz, RX buffer=4096 bytes",
             (unsigned)UM982_EXPECTED_GGA_HZ);

    BaseType_t ok = xTaskCreate(lorartk_task, "lorartk_task", 8192, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "lorartk task create failed");
    }
}

void LORARTK_SetDetection(uint8_t detected, uint8_t person_count)
{
    latest_detected_flag = detected ? 1U : 0U;
    latest_person_count = person_count;
}
