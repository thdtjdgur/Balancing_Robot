#include <stdio.h>
#include <math.h>
#include "encoder.h"
#include "variable.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm.h"

spi_transaction_t t_left;
spi_transaction_t t_right;
uint16_t tx_data = 0xFFFF;
uint16_t rx_data_left;
uint16_t rx_data_right;
encoder_handles_t enc_ctx;

spi_device_handle_t h_left;
spi_device_handle_t h_right;

SemaphoreHandle_t encoder_sem;

void IRAM_ATTR spi_post_callback(spi_transaction_t *t) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake the motor-control task as soon as the encoder SPI read finishes.
    xSemaphoreGiveFromISR(encoder_sem, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static bool timer_on_alarm_cb(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx
) {
    encoder_handles_t *handles = (encoder_handles_t *)user_ctx;

    spi_device_queue_trans(handles->left, &t_left, 0);
    spi_device_queue_trans(handles->right, &t_right, 0);

    return false;
}

void encoder_timer_init(void) {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = { .on_alarm = timer_on_alarm_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, &enc_ctx));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void encoder_init(void) {
    encoder_sem = xSemaphoreCreateBinary();

    spi_bus_config_t buscfg = {
        .miso_io_num = 13,
        .mosi_io_num = 11,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 1,
        .clock_speed_hz = 10000000,
        .queue_size = 7,
        .spics_io_num = 9,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .cs_ena_pretrans = 4,
        .post_cb = spi_post_callback,
    };

    spi_device_interface_config_t devcfg_right = devcfg;
    devcfg_right.spics_io_num = 10;

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &h_left);
    spi_bus_add_device(SPI2_HOST, &devcfg_right, &h_right);

    t_left.length = 16;
    t_left.tx_buffer = &tx_data;
    t_left.rx_buffer = &rx_data_left;

    t_right.length = 16;
    t_right.tx_buffer = &tx_data;
    t_right.rx_buffer = &rx_data_right;

    enc_ctx.left = h_left;
    enc_ctx.right = h_right;
}

void encoder_to_vcc_cal(void) {
    spi_transaction_t *ret_l;
    spi_transaction_t *ret_r;

    spi_device_get_trans_result(h_left, &ret_l, 0);
    spi_device_get_trans_result(h_right, &ret_r, 0);

    uint16_t raw_l = (rx_data_left << 8) | (rx_data_left >> 8);
    float angle_l = raw_l & 0x3FFF;
    angle_l = angle_l * 360.0f / 16384.0f;
    angle_l = angle_l * 2.0f * M_PI / 360.0f;

    uint16_t raw_r = (rx_data_right << 8) | (rx_data_right >> 8);
    float angle_r = raw_r & 0x3FFF;
    angle_r = angle_r * 360.0f / 16384.0f;
    angle_r = angle_r * 2.0f * M_PI / 360.0f;

    static int vel_calc_counter = 0;
    static float prev_angle_l = 0.0f;
    static float prev_angle_r = 0.0f;

    vel_calc_counter++;

    if (vel_calc_counter >= 10) {
        vel_calc_counter = 0;

        float diff_l = angle_l - prev_angle_l;
        if (diff_l > M_PI) {
            diff_l -= 2.0f * M_PI;
        } else if (diff_l < -M_PI) {
            diff_l += 2.0f * M_PI;
        }
        float omega_l = diff_l / 0.010f;

        float diff_r = angle_r - prev_angle_r;
        if (diff_r > M_PI) {
            diff_r -= 2.0f * M_PI;
        } else if (diff_r < -M_PI) {
            diff_r += 2.0f * M_PI;
        }
        float omega_r = diff_r / 0.010f;

        float raw_vel = ((omega_l - omega_r) / 2.0f) * WHEEL_RADIUS;

        // Simple low-pass filtering for the velocity estimate.
        current_vel = (0.5f * current_vel) + (0.5f * raw_vel);

        prev_angle_l = angle_l;
        prev_angle_r = angle_r;
        vel_calc_flag = 1;
    }

    float Vq_l = -Vq_left;
    float Vq_r = Vq_right;
    float Vd = 0.0f;

    int pole_pairs = 11;
    float offset_l = 2.7756f;
    float offset_r = 1.6345f;

    float ele_angle_l = (angle_l - offset_l) * (float)pole_pairs;
    float ele_angle_r = (angle_r - offset_r) * (float)pole_pairs;

    float V_alpha_l = Vd * cosf(ele_angle_l) - Vq_l * sinf(ele_angle_l);
    float V_beta_l = Vd * sinf(ele_angle_l) + Vq_l * cosf(ele_angle_l);

    float Va_l = V_alpha_l;
    float Vb_l = -0.5f * V_alpha_l + (sqrtf(3.0f) / 2.0f) * V_beta_l;
    float Vc_l = -0.5f * V_alpha_l - (sqrtf(3.0f) / 2.0f) * V_beta_l;

    float V_alpha_r = Vd * cosf(ele_angle_r) - Vq_r * sinf(ele_angle_r);
    float V_beta_r = Vd * sinf(ele_angle_r) + Vq_r * cosf(ele_angle_r);

    float Va_r = V_alpha_r;
    float Vb_r = -0.5f * V_alpha_r + (sqrtf(3.0f) / 2.0f) * V_beta_r;
    float Vc_r = -0.5f * V_alpha_r - (sqrtf(3.0f) / 2.0f) * V_beta_r;

    mcpwm_set_voltage(0, Va_l);
    mcpwm_set_voltage(1, Vb_l);
    mcpwm_set_voltage(2, Vc_l);

    mcpwm_set_voltage(3, Va_r);
    mcpwm_set_voltage(4, Vb_r);
    mcpwm_set_voltage(5, Vc_r);
}
