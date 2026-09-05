#include "lora.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PAYLOAD_LENGTH       0x22
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_DIO_MAPPING_1        0x40

#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_RX_DONE              0x40
#define IRQ_TX_DONE              0x08

static const char *TAG = "LORA";

static spi_device_handle_t lora_spi;
static portMUX_TYPE lora_irq_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t lora_rx_irq_count;
static bool lora_initialized;

static esp_err_t lora_spi_transfer(uint8_t *txrx, size_t length)
{
    if (lora_spi == NULL || txrx == NULL || length == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t trans = {
        .length = length * 8,
        .tx_buffer = txrx,
        .rx_buffer = txrx,
    };
    return spi_device_transmit(lora_spi, &trans);
}

static esp_err_t lora_spi_write(const uint8_t *data, size_t length)
{
    if (lora_spi == NULL || data == NULL || length == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t trans = {
        .length = length * 8,
        .tx_buffer = data,
    };
    return spi_device_transmit(lora_spi, &trans);
}

static void IRAM_ATTR lora_dio0_isr(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&lora_irq_mux);
    lora_rx_irq_count++;
    portEXIT_CRITICAL_ISR(&lora_irq_mux);
}

void lora_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(LORA_WRITE_BIT | reg), value };
    esp_err_t err = lora_spi_write(tx, sizeof(tx));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
}

void lora_write_burst(uint8_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t tx[256];

    if (data == NULL || length == 0) {
        return;
    }

    tx[0] = (uint8_t)(LORA_WRITE_BIT | reg);
    memcpy(&tx[1], data, length);

    esp_err_t err = lora_spi_write(tx, (size_t)length + 1U);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "burst write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
}

uint8_t lora_read(uint8_t reg)
{
    uint8_t txrx[2] = { (uint8_t)(LORA_READ_BIT | reg), 0x00 };

    esp_err_t err = lora_spi_transfer(txrx, sizeof(txrx));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return 0;
    }

    return txrx[1];
}

static void lora_read_burst(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t tx[256] = {0};
    uint8_t rx[256] = {0};

    if (data == NULL || length == 0) {
        return;
    }

    tx[0] = (uint8_t)(LORA_READ_BIT | reg);

    spi_transaction_t trans = {
        .length = ((size_t)length + 1U) * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_transmit(lora_spi, &trans);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "burst read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        memset(data, 0, length);
        return;
    }

    memcpy(data, &rx[1], length);
}

void lora_setup(void)
{
    lora_write(REG_OP_MODE, LORA_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_write(REG_PA_CONFIG, 0x8F);
    lora_write(REG_OCP, 0x31);
    lora_write(REG_IRQ_FLAGS_MASK, 0x00);
    lora_write(REG_IRQ_FLAGS, 0xFF);
}

void lora_freq(void)
{
    const uint8_t frf[] = {0xE6, 0x86, 0x66}; // 922.1 MHz

    lora_write(REG_OP_MODE, LORA_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_write_burst(REG_FRF_MSB, frf, sizeof(frf));
}

void packet_set(void)
{
    lora_write(REG_MODEM_CONFIG_1, 0x72); // BW 125 kHz, CR 4/5, explicit header
    lora_write(REG_MODEM_CONFIG_2, 0x74); // SF7, payload CRC on
}

void fifo_set(void)
{
    lora_write(REG_OP_MODE, LORA_STANDBY);
    lora_write(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write(REG_FIFO_RX_BASE_ADDR, 0x00);
}

void rx_set(void)
{
    lora_write(REG_OP_MODE, LORA_STANDBY);
    lora_write(REG_IRQ_FLAGS, 0xFF);
    lora_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write(REG_FIFO_ADDR_PTR, 0x00);
    lora_write(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
    lora_write(REG_OP_MODE, LORA_RX_CONT);
}

esp_err_t lora_init(void)
{
    if (lora_initialized) {
        return ESP_OK;
    }

    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << LORA_PIN_EN) | (1ULL << LORA_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "gpio output config failed");

    gpio_set_level(LORA_PIN_EN, 1);
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_PIN_MISO,
        .mosi_io_num = LORA_PIN_MOSI,
        .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    esp_err_t err = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "spi_bus_initialize failed");
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2000000,
        .mode = 0,
        .spics_io_num = LORA_PIN_CS,
        .queue_size = 4,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(LORA_SPI_HOST, &devcfg, &lora_spi),
                        TAG, "spi_bus_add_device failed");

    gpio_config_t dio_cfg = {
        .pin_bit_mask = 1ULL << LORA_PIN_DIO0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dio_cfg), TAG, "dio0 gpio config failed");

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "gpio isr service install failed");
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(LORA_PIN_DIO0, lora_dio0_isr, NULL),
                        TAG, "dio0 isr add failed");

    lora_setup();
    lora_freq();
    packet_set();
    fifo_set();
    rx_set();

    lora_initialized = true;
    ESP_LOGI(TAG, "SX1276 init done: CS=%d EN=%d RST=%d DIO0=%d version=0x%02X",
             LORA_PIN_CS, LORA_PIN_EN, LORA_PIN_RST, LORA_PIN_DIO0, lora_read(0x42));
    return ESP_OK;
}

int lora_receive_packet(uint8_t *buffer, uint8_t *length, uint8_t *irq_flags)
{
    uint8_t irq = lora_read(REG_IRQ_FLAGS);
    uint8_t received_length;

    if (buffer == NULL || length == NULL || irq_flags == NULL) {
        return -1;
    }

    *irq_flags = irq;
    *length = 0;

    if ((irq & IRQ_PAYLOAD_CRC_ERROR) != 0U) {
        lora_write(REG_OP_MODE, LORA_STANDBY);
        lora_write(REG_IRQ_FLAGS, 0xFF);
        return -1;
    }

    if ((irq & IRQ_RX_DONE) == 0U) {
        return 0;
    }

    lora_write(REG_OP_MODE, LORA_STANDBY);
    received_length = lora_read(REG_RX_NB_BYTES);
    lora_write(REG_FIFO_ADDR_PTR, lora_read(REG_FIFO_RX_CURRENT_ADDR));
    lora_read_burst(REG_FIFO, buffer, received_length);
    *length = received_length;
    lora_write(REG_IRQ_FLAGS, 0xFF);

    return 1;
}

bool lora_send_packet(const uint8_t *buffer, uint8_t length, uint32_t timeout_ms)
{
    uint32_t start_tick;

    if (!lora_initialized || buffer == NULL || length == 0) {
        return false;
    }

    lora_write(REG_OP_MODE, LORA_STANDBY);
    lora_write(REG_IRQ_FLAGS, 0xFF);
    lora_write(REG_FIFO_ADDR_PTR, 0x80);
    lora_write_burst(REG_FIFO, buffer, length);
    lora_write(REG_PAYLOAD_LENGTH, length);
    lora_write(REG_DIO_MAPPING_1, 0x40); // DIO0 = TxDone
    lora_write(REG_OP_MODE, LORA_TX);

    start_tick = xTaskGetTickCount();
    while ((lora_read(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0U) {
        uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS);

        if (elapsed_ms >= timeout_ms) {
            lora_write(REG_OP_MODE, LORA_STANDBY);
            lora_write(REG_IRQ_FLAGS, 0xFF);
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    lora_write(REG_OP_MODE, LORA_STANDBY);
    lora_write(REG_IRQ_FLAGS, 0xFF);
    return true;
}

bool lora_take_rx_flag(void)
{
    bool ready = false;

    portENTER_CRITICAL(&lora_irq_mux);
    if (lora_rx_irq_count > 0) {
        lora_rx_irq_count--;
        ready = true;
    }
    portEXIT_CRITICAL(&lora_irq_mux);

    return ready;
}

bool lora_irq_pending(void)
{
    uint8_t irq = lora_read(REG_IRQ_FLAGS);
    return (irq & (IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR)) != 0U;
}

int16_t lora_current_rssi_dbm(void)
{
    return (int16_t)lora_read(REG_RSSI_VALUE) - 157;
}

int16_t lora_packet_rssi_dbm(void)
{
    return (int16_t)lora_read(REG_PKT_RSSI_VALUE) - 157;
}

int16_t lora_packet_snr_x100(void)
{
    int8_t snr_raw = (int8_t)lora_read(REG_PKT_SNR_VALUE);
    return (int16_t)snr_raw * 25;
}
