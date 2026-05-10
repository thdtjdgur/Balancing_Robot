#include <stdio.h>
#include "encoder.h"
#include "variable.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm.h"
#include <math.h>
#include "driver/uart.h"

// app_main.c에서 HILS 데이터를 가져오기 위한 선언
extern float hils_angle_l;
extern float hils_angle_r;

extern float Vq_left;
extern float Vq_right;

// SPI 관련 변수
spi_transaction_t t_left;
spi_transaction_t t_right;
uint16_t tx_data = 0xFFFF;
uint16_t rx_data_left;
uint16_t rx_data_right;
encoder_handles_t enc_ctx;

spi_device_handle_t h_left;
spi_device_handle_t h_right;

SemaphoreHandle_t encoder_sem;

/*
 * 실제 하드웨어 모드용 1kHz 타이머 콜백.
 * HILS에서는 app_main.c에서 encoder_timer_init()을 호출하지 않으므로 사용 안 됨.
 */
static bool timer_on_alarm_cb(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx
) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(encoder_sem, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken == pdTRUE);
}

void encoder_timer_init(void) {
    gptimer_handle_t gptimer = NULL;

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000,              // 1ms = 1kHz
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, &enc_ctx));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void encoder_init(void) {
    encoder_sem = xSemaphoreCreateBinary();

    /*
     * HILS에서는 실제 SPI 엔코더값을 쓰지 않음.
     * 그래도 기존 구조와 실제 하드웨어 모드 호환을 위해 초기화는 유지함.
     */
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
        .post_cb = NULL,
    };

    spi_device_interface_config_t devcfg_right = devcfg;
    devcfg_right.spics_io_num = 10;

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &h_left);
    spi_bus_add_device(SPI2_HOST, &devcfg_right, &h_right);
}

void encoder_to_vcc_cal(void) {
    /*
     * HILS에서는 MATLAB/Simulink에서 받은 가상 엔코더 각도를 그대로 사용.
     * 이 함수는 hils_task에서 1kHz로 호출됨.
     */
    float angle_l = hils_angle_l;
    float angle_r = hils_angle_r;

    /*
     * 속도 계산은 200Hz로 갱신.
     * 이유:
     * - angle_l/r는 1kHz로 들어오지만
     * - velocity/pitch 제어는 200Hz 기준이므로 5ms마다 속도를 갱신
     */
    static int vel_calc_counter = 0;
    static float prev_angle_l = 0.0f;
    static float prev_angle_r = 0.0f;

    vel_calc_counter++;

    if (vel_calc_counter >= 5) {
        vel_calc_counter = 0;

        float diff_l = angle_l - prev_angle_l;
        if (diff_l > M_PI) {
            diff_l -= 2.0f * M_PI;
        } else if (diff_l < -M_PI) {
            diff_l += 2.0f * M_PI;
        }

        float diff_r = angle_r - prev_angle_r;
        if (diff_r > M_PI) {
            diff_r -= 2.0f * M_PI;
        } else if (diff_r < -M_PI) {
            diff_r += 2.0f * M_PI;
        }

        float omega_l = diff_l / 0.005f;
        float omega_r = diff_r / 0.005f;

        /*
         * 기존 식 유지.
         * 만약 속도 부호가 반대면 여기서 omega_l - omega_r 순서를 바꾸면 됨.
         */
        float raw_vel = ((omega_l - omega_r) / 2.0f) * WHEEL_RADIUS;

        current_vel = (0.5f * current_vel) + (0.5f * raw_vel);

        prev_angle_l = angle_l;
        prev_angle_r = angle_r;

        vel_calc_flag = 1;
    }

    /*
     * PID 결과 Vq를 BLDC q축 전압으로 사용.
     * 기존 코드의 부호 유지.
     * 만약 pitch 보상 방향이 반대면 여기 부호를 먼저 의심해야 함.
     */
    float Vq_r = -Vq_right;
    float Vq_l = -Vq_left;
    float Vd = 0.0f;

    /*
     * 기계각 -> 전기각
     * angle_l/r는 라디안 기준.
     */
    int pole_pairs = 11;

    float offset_l = 0.0f;
    float offset_r = 0.0f;

    float ele_angle_l = (angle_l - offset_l) * (float)pole_pairs;
    float ele_angle_r = (angle_r - offset_r) * (float)pole_pairs;

    // ===================== 왼쪽 바퀴: inverse Park + Clarke =====================
    float V_alpha_l = Vd * cosf(ele_angle_l) - Vq_l * sinf(ele_angle_l);
    float V_beta_l  = Vd * sinf(ele_angle_l) + Vq_l * cosf(ele_angle_l);

    float Va_l = V_alpha_l;
    float Vb_l = -0.5f * V_alpha_l + (sqrtf(3.0f) / 2.0f) * V_beta_l;
    float Vc_l = -0.5f * V_alpha_l - (sqrtf(3.0f) / 2.0f) * V_beta_l;

    // ===================== 오른쪽 바퀴: inverse Park + Clarke =====================
    float V_alpha_r = Vd * cosf(ele_angle_r) - Vq_r * sinf(ele_angle_r);
    float V_beta_r  = Vd * sinf(ele_angle_r) + Vq_r * cosf(ele_angle_r);

    float Va_r = V_alpha_r;
    float Vb_r = -0.5f * V_alpha_r + (sqrtf(3.0f) / 2.0f) * V_beta_r;
    float Vc_r = -0.5f * V_alpha_r - (sqrtf(3.0f) / 2.0f) * V_beta_r;

    /*
     * 실제 PWM에도 출력.
     * HILS만 할 거면 MATLAB 송신값만 중요하지만,
     * 기존 구조 유지 위해 그대로 둠.
     */
    mcpwm_set_voltage(0, Va_l);
    mcpwm_set_voltage(1, Vb_l);
    mcpwm_set_voltage(2, Vc_l);

    mcpwm_set_voltage(3, Va_r);
    mcpwm_set_voltage(4, Vb_r);
    mcpwm_set_voltage(5, Vc_r);

    /*
     * MATLAB/Simulink로 보낼 3상 전압.
     * 기존 순서 유지:
     * 오른쪽 C, B, A / 왼쪽 C, B, A
     */
    hils_v_out[0] = Vc_r;
    hils_v_out[1] = Vb_r;
    hils_v_out[2] = Va_r;

    hils_v_out[3] = Vc_l;
    hils_v_out[4] = Vb_l;
    hils_v_out[5] = Va_l;
}