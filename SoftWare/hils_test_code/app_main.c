#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "imu.h"
#include "encoder.h"
#include "pwm.h"
#include "pid.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "rx28.h"
#include "variable.h"
#include "driver/uart.h"
#include "lidar.h"

// ===================== HILS 주기 설정 =====================
// MATLAB -> ESP32 패킷은 1kHz로 들어온다고 가정
// angle_l, angle_r : 1kHz 갱신
// pitch, yaw       : MATLAB에서 200Hz 갱신 후 ZOH로 1kHz 패킷에 실려 옴
#define HILS_UART_PORT          UART_NUM_1
#define HILS_BAUD_RATE          921600

#define HILS_FAST_HZ            1000
#define HILS_PID_HZ             200

#define HILS_FAST_DT            0.001f
#define HILS_PID_DT             0.005f
#define HILS_PID_DECIMATION     5

// ===================== 속도 PID 설정 =====================
// 0.50 rad = 약 28.6도
#define TARGET_PITCH_MAX_RAD    0.50f

// TX: ESP32 -> MATLAB, 24바이트
typedef struct {
    float v_out[6];
} __attribute__((packed)) HILSTxPacket;

// RX: MATLAB -> ESP32, 16바이트
typedef struct {
    float angle_r;
    float angle_l;
    float sim_pitch;
    float sim_yaw;
} __attribute__((packed)) SimulinkRxPacket;

// ===================== extern =====================
extern SemaphoreHandle_t encoder_sem;
extern spi_device_handle_t h_left;
extern spi_transaction_t *ret_t;

// ===================== 전역 상태 변수 =====================
float current_vel = 0.0f;
float current_pitch = 0.0f;
float current_yaw = 0.0f;
float current_roll = 0.0f;
float roll_adj_mm = 0.0f;

float targetvel_vel = 0.0f;      // 목표 속도 [m/s]
float target_yaw_diff = 0.0f;

int vel_calc_flag = 0;

// HILS 전용 입력값
float hils_angle_l = 0.0f;
float hils_angle_r = 0.0f;
float hils_pitch = 0.0f;
float hils_yaw = 0.0f;

// HILS 전용 출력값
float hils_v_out[6] = {0.0f};

// PID 제어기
PIDController pitch_ctrl, yaw_ctrl, vel_ctrl, roll_ctrl;

// BLDC q축 전압 명령
float Vq_left = 0.0f;
float Vq_right = 0.0f;

const float MOTOR_V_MIN = 0.0f;

// ===================== 유틸 함수 =====================
static float clamp_float(float x, float min_val, float max_val) {
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

static float wrap_pi(float x) {
    if (x > M_PI) {
        x -= 2.0f * M_PI;
    } else if (x < -M_PI) {
        x += 2.0f * M_PI;
    }
    return x;
}

// 실제 하드웨어 모드용. HILS에서는 사용 안 함.
void motor_control_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(encoder_sem, portMAX_DELAY) == pdTRUE) {
            encoder_to_vcc_cal();
        }
    }
}

// ===================== HILS UART Task =====================
void hils_task(void *pvParameters) {
    const uart_port_t uart_num = HILS_UART_PORT;

    uart_config_t uart_config = {
        .baud_rate = HILS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // UART1: TX 17, RX 18
    ESP_ERROR_CHECK(uart_set_pin(
        uart_num,
        17,
        18,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_ERROR_CHECK(uart_driver_install(
        uart_num,
        4096,
        4096,
        0,
        NULL,
        0
    ));

    SimulinkRxPacket rx_data;
    HILSTxPacket tx_packet = {0};

    esp_rom_printf("HILS: UART1 Set. 1kHz packet loop start.\n");

    // MATLAB 쪽 call-response 시작용 0V 패킷
    uart_write_bytes(uart_num, (const char *)&tx_packet, sizeof(HILSTxPacket));
    esp_rom_printf("HILS: Priming packet sent.\n");

    int pid_counter = HILS_PID_DECIMATION;

    // 속도 계산용 변수
    static int vel_init = 0;
    static float prev_angle_l = 0.0f;
    static float prev_angle_r = 0.0f;

    while (1) {
        int len = uart_read_bytes(
            uart_num,
            (uint8_t *)&rx_data,
            sizeof(SimulinkRxPacket),
            portMAX_DELAY
        );

        if (len != sizeof(SimulinkRxPacket)) {
            continue;
        }

        // ===================== 쓰레기값 방어 =====================
        if (!isfinite(rx_data.angle_l) ||
            !isfinite(rx_data.angle_r) ||
            !isfinite(rx_data.sim_pitch) ||
            !isfinite(rx_data.sim_yaw) ||
            rx_data.sim_pitch > 10.0f ||
            rx_data.sim_pitch < -10.0f) {

            uart_flush_input(uart_num);

            // 싱크 유지용으로 마지막 정상 전압 또는 0V 패킷 송신
            uart_write_bytes(uart_num, (const char *)&tx_packet, sizeof(HILSTxPacket));
            continue;
        }

        // ===================== 1kHz 입력 업데이트 =====================
        // 엔코더 각도는 매 패킷마다 새 값이라고 가정
        hils_angle_r = rx_data.angle_r;
        hils_angle_l = rx_data.angle_l;

        // pitch/yaw는 MATLAB 쪽에서 200Hz ZOH 되어 들어온다고 가정
        hils_pitch = rx_data.sim_pitch;
        hils_yaw = rx_data.sim_yaw;

        current_pitch = hils_pitch;
        current_yaw = hils_yaw;

        // ===================== 200Hz PID 계산 =====================
        // 1kHz 패킷 5번마다 PID 1번
        pid_counter++;

        if (pid_counter >= HILS_PID_DECIMATION) {
            pid_counter = 0;

            // ===================== 속도 계산 =====================
            // angle_l/r은 rad 기준
            if (vel_init == 0) {
                prev_angle_l = hils_angle_l;
                prev_angle_r = hils_angle_r;
                current_vel = 0.0f;
                vel_init = 1;
            } else {
                float diff_l = wrap_pi(hils_angle_l - prev_angle_l);
                float diff_r = wrap_pi(hils_angle_r - prev_angle_r);

                float omega_l = diff_l / HILS_PID_DT;
                float omega_r = diff_r / HILS_PID_DT;

                // 기존 encoder.c 방식 유지
                float raw_vel = ((omega_l - omega_r) / 2.0f) * WHEEL_RADIUS;

                // 속도 필터
                current_vel = (0.5f * current_vel) + (0.5f * raw_vel);

                prev_angle_l = hils_angle_l;
                prev_angle_r = hils_angle_r;
            }

            // ===================== 속도 PID =====================
            // 현재는 속도 PID 계산만 하고 target_pitch에는 안 쓰는 상태
            float vel_out = pid_calculate(
                &vel_ctrl,
                targetvel_vel,
                current_vel,
                HILS_PID_DT
            );

            vel_out = clamp_float(
                vel_out,
                -TARGET_PITCH_MAX_RAD,
                TARGET_PITCH_MAX_RAD
            );

            // 균형 테스트용: 목표 pitch = 0 rad
            // 속도 PID까지 적용하려면 아래 줄을 target_pitch = vel_out; 로 바꾸면 됨
            float target_pitch = 0.0;//vel_out/////////////////////////////////////////////////////

            // ===================== pitch PID =====================
            float p_out = pid_calculate(
                &pitch_ctrl,
                target_pitch,
                current_pitch,
                HILS_PID_DT
            );

            // ===================== yaw PID =====================
            // 목표 yaw = 0 rad
            float target_yaw = 0.0f;

            float y_out = pid_calculate(
                &yaw_ctrl,
                target_yaw,
                current_yaw,
                HILS_PID_DT
            );

            // yaw 출력 제한
            float MAX_TURN_V = 1.5f;
            if (y_out > MAX_TURN_V) {
                y_out = MAX_TURN_V;
            } else if (y_out < -MAX_TURN_V) {
                y_out = -MAX_TURN_V;
            }

            // ===================== 좌우 Vq 분배 =====================
            Vq_left = p_out + y_out;
            Vq_right = p_out - y_out;
        }

        // ===================== 1kHz 3상 전압 계산 =====================
        // 최신 angle_l/r 기준으로 BLDC 전기각 계산
        // PID 출력 Vq는 200Hz마다 갱신되고, 그 사이에는 유지됨
        encoder_to_vcc_cal();

        // ===================== 1kHz MATLAB 송신 =====================
        for (int i = 0; i < 6; i++) {
            tx_packet.v_out[i] = hils_v_out[i];
        }

        uart_write_bytes(
            uart_num,
            (const char *)&tx_packet,
            sizeof(HILSTxPacket)
        );
    }
}

// ===================== app_main =====================
void app_main(void) {
    esp_rom_printf("\n\n=== 1. APP MAIN START ===\n\n");

    // HILS에서는 실제 IMU 세마포어는 안 써도 되지만 기존 구조 유지
    imu_sem = xSemaphoreCreateBinary();

    // ===================== PID 초기화 =====================
    // 속도 PID 출력 제한 = 목표 pitch 제한과 비슷하게 둠
    pid_init(&vel_ctrl, 0.5f, 0.0f, 0.0f, TARGET_PITCH_MAX_RAD);

    /*
     * HILS 균형 테스트용.
     * PID 계산 주기는 200Hz이므로 dt = 0.005f로 들어감.
     */
    pid_init(&pitch_ctrl, 90.0f, 0.0f, 0.6f, 100.0f);

    /*
     * yaw 목표값은 0 rad.
     * yaw_ctrl 출력은 좌우 Vq 차이로 들어감.
     */
    pid_init(&yaw_ctrl, 20.0f, 0.0f, 0.01f, 5.0f);

    pid_init(&roll_ctrl, 2.0f, 0.08f, 0.01f, 150.0f);

    // HILS에서 일단 제자리 속도 목표
    targetvel_vel = 0.0f;

    esp_rom_printf("=== 2. Motor Init ===\n");

    /*
     * HILS에서는 실제 IMU값 안 씀.
     * MATLAB에서 pitch/yaw를 받음.
     */
    // i2c_master_dev_handle_t imu_handle = imu_init();

    encoder_init();
    init_mcpwm_bldc();

    esp_rom_printf("=== 3. Motor Init Done ===\n");

    /*
     * HILS 중에는 UART 자원 충돌 방지 때문에 꺼둠.
     */
    // init_hc06();
    // init_rx28();
    // init_lidar();

    /*
     * HILS 핵심:
     * MATLAB 패킷이 1kHz로 들어오므로 UART 수신 루프 자체가 1kHz 기준 클럭 역할을 함.
     */
    xTaskCreate(hils_task, "HILS_Task", 4096, NULL, 10, NULL);

    esp_rom_printf("=== 4. HILS Task Created ===\n");

    /*
     * HILS에서는 실제 encoder/imu timer를 켜지 않음.
     * 이유:
     * - 엔코더 각도는 MATLAB에서 1kHz로 들어옴
     * - IMU pitch/yaw도 MATLAB에서 들어옴
     * - encoder_to_vcc_cal()은 hils_task 안에서 1kHz로 직접 호출함
     */
    // imu_timer_init();
    // encoder_timer_init();

    // 실제 모터 제어 태스크도 HILS에서는 사용 안 함
    // xTaskCreate(motor_control_task, "Motor_Task", 4096, NULL, 5, NULL);

    // xTaskCreate(rx28_task, "RX28_Task", 4096, NULL, 4, NULL);

    esp_rom_printf("=== 5. While Loop Start ===\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}