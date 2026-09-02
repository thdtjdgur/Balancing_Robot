#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#define LORA_SPI_HOST SPI2_HOST
#define LORA_PIN_CS   8
#define LORA_PIN_SCK  12
#define LORA_PIN_MISO 13
#define LORA_PIN_MOSI 11
#define LORA_PIN_EN   6
#define LORA_PIN_RST  14
#define LORA_PIN_DIO0 38

#define LORA_WRITE_BIT 0x80
#define LORA_READ_BIT  0x00
#define LORA_SLEEP     0x80
#define LORA_STANDBY   0x81
#define LORA_TX        0x83
#define LORA_RX_CONT   0x85

esp_err_t lora_init(void);
void lora_setup(void);
void lora_freq(void);
void packet_set(void);
void fifo_set(void);
void rx_set(void);

void lora_write(uint8_t reg, uint8_t value);
void lora_write_burst(uint8_t reg, const uint8_t *data, uint8_t length);
uint8_t lora_read(uint8_t reg);

int lora_receive_packet(uint8_t *buffer, uint8_t *length, uint8_t *irq_flags);
bool lora_send_packet(const uint8_t *buffer, uint8_t length, uint32_t timeout_ms);
bool lora_take_rx_flag(void);
bool lora_irq_pending(void);

int16_t lora_current_rssi_dbm(void);
int16_t lora_packet_rssi_dbm(void);
int16_t lora_packet_snr_x100(void);
