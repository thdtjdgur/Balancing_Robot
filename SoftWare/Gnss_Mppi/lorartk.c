#include "lorartk.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "rtk_bridge.h"

#define TTGO_PACKET_SINGLE        0xA1U
#define TTGO_PACKET_FRAGMENT      0xA2U
#define TTGO_SINGLE_HEADER_SIZE   2U
#define TTGO_FRAGMENT_HEADER_SIZE 4U
#define RTCM_MAX_FRAME_SIZE       1029U
#define FRAGMENT_TIMEOUT_MS       1500U

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
    ESP_LOGW(TAG, "sequence miss/out-of-order: rx=%u expected=%u",
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
        ESP_LOGW(TAG, "RTCM format error: length=%u", (unsigned)length);
        log_rtcm_result(sequence, message_type, length, false, fragment_count, false, "FORMAT_ERROR");
        return;
    }

    payload_length = (uint16_t)(((uint16_t)(frame[1] & 0x03U) << 8) | frame[2]);
    expected_length = (uint16_t)(payload_length + 6U);
    if (length != expected_length) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "RTCM length mismatch: received=%u header=%u",
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
        ESP_LOGW(TAG, "RTCM type=%u length=%u CRC error rx=%06" PRIX32 " calc=%06" PRIX32,
                 (unsigned)message_type, (unsigned)length, received_crc, calculated_crc);
        log_rtcm_result(sequence, message_type, length, false, fragment_count, false, "CRC_ERROR");
        return;
    }

    count_rtcm_type(message_type);
    if (RTK_Bridge_ForwardRTCM(frame, length)) {
        ESP_LOGI(TAG, "RTCM type=%u length=%u CRC=OK -> UM982",
                 (unsigned)message_type, (unsigned)length);
        log_rtcm_result(sequence, message_type, length, true, fragment_count, true, "OK");
    } else {
        ESP_LOGW(TAG, "RTCM type=%u length=%u UART drop",
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
        ESP_LOGW(TAG, "discard incomplete sequence=%u by single frame=%u",
                 (unsigned)reassembly.sequence, (unsigned)sequence);
        reset_reassembly();
    }

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

    ESP_LOGD(TAG, "fragment sequence=%u index=%u/%u data=%u",
             (unsigned)sequence, (unsigned)index, (unsigned)count, (unsigned)data_length);

    if (count < 2 || index >= count) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "invalid fragment header");
        return;
    }

    if (index == 0) {
        if (reassembly.active) {
            if (reassembly.sequence == sequence && reassembly.next_fragment > 0) {
                total.duplicate_packets++;
                interval.duplicate_packets++;
                ESP_LOGD(TAG, "duplicate first fragment ignored");
                return;
            }
            total.reassembly_errors++;
            interval.reassembly_errors++;
            ESP_LOGW(TAG, "previous sequence=%u discarded", (unsigned)reassembly.sequence);
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
    } else if (!reassembly.active || reassembly.sequence != sequence) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "no active frame for sequence=%u index=%u",
                 (unsigned)sequence, (unsigned)index);
        return;
    }

    if (count != reassembly.fragment_count) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "fragment count changed: %u -> %u",
                 (unsigned)reassembly.fragment_count, (unsigned)count);
        reset_reassembly();
        return;
    }
    if (index < reassembly.next_fragment) {
        total.duplicate_packets++;
        interval.duplicate_packets++;
        ESP_LOGD(TAG, "duplicate fragment ignored: %u", (unsigned)index);
        return;
    }
    if (index != reassembly.next_fragment) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "fragment out-of-order: rx=%u expected=%u",
                 (unsigned)index, (unsigned)reassembly.next_fragment);
        reset_reassembly();
        return;
    }
    if ((uint32_t)reassembly.length + data_length > RTCM_MAX_FRAME_SIZE) {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        ESP_LOGW(TAG, "reassembly buffer overflow");
        reset_reassembly();
        return;
    }

    memcpy(&reassembly.frame[reassembly.length], &packet[TTGO_FRAGMENT_HEADER_SIZE], data_length);
    reassembly.length = (uint16_t)(reassembly.length + data_length);
    reassembly.next_fragment++;
    reassembly.last_fragment_tick = millis();

    if (reassembly.next_fragment == reassembly.fragment_count) {
        ESP_LOGI(TAG, "reassembly complete seq=%u fragments=%u length=%u",
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
        ESP_LOGW(TAG, "LoRa payload CRC error irq=0x%02X", irq);
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

    ESP_LOGI(TAG, "LoRa RX length=%u RSSI=%d dBm SNRx100=%d",
             (unsigned)length, lora_packet_rssi_dbm(), lora_packet_snr_x100());

    if (length == 0) {
        total.unsupported_packets++;
        interval.unsupported_packets++;
    } else if (packet[0] == TTGO_PACKET_SINGLE) {
        process_single_packet(packet, length);
    } else if (packet[0] == TTGO_PACKET_FRAGMENT) {
        process_fragment_packet(packet, length);
    } else {
        total.unsupported_packets++;
        interval.unsupported_packets++;
        ESP_LOGW(TAG, "unsupported packet type=0x%02X", packet[0]);
    }
}

static void print_one_second_statistics(void)
{
    uint32_t forwarded_frames = RTK_Bridge_GetForwardedFrames();
    uint32_t forwarded_bytes = RTK_Bridge_GetForwardedBytes();
    uint32_t uart_drops = RTK_Bridge_GetDroppedFrames();

    ESP_LOGI(TAG,
             "1s LoRa=%" PRIu32 "pkt/%" PRIu32 "B RTCM=%" PRIu32 "frame/%" PRIu32 "B FWD=%" PRIu32 "/%" PRIu32 "B CRCerr=%" PRIu32 " PHYerr=%" PRIu32 " ReasmErr=%" PRIu32 " SeqMiss=%" PRIu32 " Dup=%" PRIu32 " UARTdrop=%" PRIu32 " GGAq=%u",
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
             (unsigned)RTK_Bridge_GetGGAQuality());

    last_forwarded_frames = forwarded_frames;
    last_forwarded_bytes = forwarded_bytes;
    last_uart_drops = uart_drops;
    memset(&interval, 0, sizeof(interval));
}

static void print_ten_second_totals(void)
{
    ESP_LOGI(TAG,
             "TOTAL LoRa=%" PRIu32 "pkt/%" PRIu32 "B RTCM=%" PRIu32 "frame/%" PRIu32 "B Forwarded=%" PRIu32 "frame/%" PRIu32 "B UM982_RX=%" PRIu32 "B CRCerr=%" PRIu32 " Timeout=%" PRIu32 " SeqMiss=%" PRIu32 " Unsupported=%" PRIu32,
             total.lora_packets,
             total.lora_bytes,
             total.completed_frames,
             total.completed_bytes,
             RTK_Bridge_GetForwardedFrames(),
             RTK_Bridge_GetForwardedBytes(),
             RTK_Bridge_GetUM982Bytes(),
             total.rtcm_crc_errors,
             total.fragment_timeouts,
             total.sequence_misses,
             total.unsupported_packets);
    ESP_LOGI(TAG,
             "TYPE 1005=%" PRIu32 " 1006=%" PRIu32 " 1077=%" PRIu32 " 1087=%" PRIu32 " 1097=%" PRIu32 " 1127=%" PRIu32 " 1230=%" PRIu32 " other=%" PRIu32,
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
            ESP_LOGW(TAG, "reassembly timeout seq=%u received=%u/%u length=%u",
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

    ESP_LOGI(TAG, "LORARTK receiver started");
    ESP_LOGI(TAG, "LoRa 922.1MHz SF7 BW125 CR4/5 explicit CRC-on DIO0=GPIO%d", LORA_PIN_DIO0);
    ESP_LOGI(TAG, "Packets: A1=[type,seq,RTCM], A2=[type,seq,index,count,data]");
    ESP_LOGI(TAG, "UM982 UART2 TX%d/RX%d %d 8N1", UM982_TX_PIN, UM982_RX_PIN, UM982_BAUD_RATE);

    BaseType_t ok = xTaskCreate(lorartk_task, "lorartk_task", 8192, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "lorartk task create failed");
    }
}
