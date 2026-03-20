#include <stdio.h>
#include "imu.h"
#include "variable.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "driver/uart.h"

static const char *TAG = "WT901";

// 링커가 찾는 세마포어 변수의 실체
SemaphoreHandle_t imu_sem;

// 수신 버퍼 (각 데이터 2바이트씩)
static uint8_t pitch_buf[2], yaw_buf[2], gyro_buf[2], roll_buf[2];

// [핵심] 타이머 인터럽트 콜백 함수
// alarm_count가 5000에 도달하면 ESP32의 하드웨어 타이머 컨트롤러가 이 함수를 호출함
static bool IRAM_ATTR imu_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    // 메인 루프에 "데이터 읽어라"라고 신호(세마포어)를 줌
    xSemaphoreGiveFromISR(imu_sem, &high_task_awoken);
    //rtos는 encoder_sem(세마포어)주문서와 imu_sem(세마포어)주문서를 기다리는 요리사(쓰레드) 각각의 우선순위를 확인하고 
    //encoder_sem를 기다리는 요리사의 우선순위가 높기때문에 high_task_awoken에 false를 적음
    //나중에 encoder_sem(세마포어)주문서를 받는 요리사가 xSemaphoreTake에 의해 잠들었을때 실행됨
    
    // bool 값을 반환하여 컨텍스트 스위칭 필요 여부를 알림
    return high_task_awoken == pdTRUE;
}

i2c_master_dev_handle_t imu_init(void) {
    // I2C 마스터 버스 설정 로직 포함
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_cfg, &bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = WT901_I2C_ADDR, // 주소만 WT901으로
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // WT901 센서(imu) 출력 속도를 200Hz로 설정
    uint8_t set_rate[] = {WT901_REG_RRATE, WT901_RRATE_200HZ};
    i2c_master_transmit(dev_handle, set_rate, sizeof(set_rate), -1);
    vTaskDelay(pdMS_TO_TICKS(50)); // 설정 저장 시간 대기
    
    return dev_handle;
}

void imu_timer_init(void) {
    gptimer_handle_t timer = NULL; // gptimer 변수 선언
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,      // Counting direction is up
        .resolution_hz = IMU_TIMER_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timer));

    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,                  // 알람 발생 시 타이머를 자동으로 0으로 리로드
        .alarm_count = IMU_ALARM_COUNT,     // 200Hz 설정
        .flags.auto_reload_on_alarm = true, // auto-reload 기능 활성화
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    // 콜백 함수 등록 (alarm_count가 5000에 도달해 알람이 울리면 imu_timer_cb 실행)
    gptimer_event_callbacks_t cbs = { .on_alarm = imu_timer_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer)); // 타이머 시작
    
    ESP_LOGI(TAG, "200Hz Timer Initialized.");
}

void imu_data_cal(i2c_master_dev_handle_t dev_handle) {
    // WT901은 레지스터 방식: 먼저 읽을 레지스터 주소를 쓰고, 2바이트 읽기
    uint8_t r_p = WT901_REG_PITCH, r_y = WT901_REG_YAW, r_g = WT901_REG_GYRO, r_r = WT901_REG_ROLL;
    
    i2c_master_transmit_receive(dev_handle, &r_p, 1, pitch_buf, 2, -1);
    i2c_master_transmit_receive(dev_handle, &r_y, 1, yaw_buf, 2, -1);
    i2c_master_transmit_receive(dev_handle, &r_g, 1, gyro_buf, 2, -1);
    i2c_master_transmit_receive(dev_handle, &r_r, 1, roll_buf, 2, -1);

    // WT901 데이터는 Low byte, High byte 순서 (Little Endian 복원)
    int16_t p_raw = (int16_t)((pitch_buf[1] << 8) | pitch_buf[0]);
    int16_t y_raw = (int16_t)((yaw_buf[1] << 8) | yaw_buf[0]);
    int16_t g_raw = (int16_t)((gyro_buf[1] << 8) | gyro_buf[0]);
    int16_t r_raw = (int16_t)((roll_buf[1] << 8) | roll_buf[0]);

    // 데이터시트 변환 공식: raw / 32768 * 180
    current_pitch = (float)p_raw / 32768.0f * M_PI; // 랴디안 // current_yaw   = (float)y_raw / 32768.0f * 180.0f;//도
    current_yaw   = (float)y_raw / 32768.0f * M_PI;   // 라디안
    current_roll  = (float)r_raw / 32768.0f * 180.0f + 1.43f;
    float gyro  = (float)g_raw / 32768.0f * 180.0f;  // 도/초(deg/s)

    
    static int bt_send_cnt = 0;
    /*
    // 20번 루프 돌 때마다 1번씩 전송 (200Hz / 20 = 10Hz, 1초에 10번 출력)
    if (bt_send_cnt++ >= 20) {
        char tx_buf[64]; // 문자를 담을 빈 그릇
        
        // 테라텀은 줄바꿈 시 \r\n 을 같이 써주는 것이 깔끔하게 나옵니다.
        int len = sprintf(tx_buf, "Pitch = %.2f, Yaw = %.2f, Gyro  = %.2f, Roll = %.2f\r\n", 
                          current_pitch*180/M_PI, current_yaw*180/M_PI, gyro, current_roll);
        
        // UART_NUM_2 (HC-06)로 문자열 쏘기
        uart_write_bytes(UART_NUM_2, tx_buf, len);
        
        // (선택) 컴퓨터 USB 시리얼 모니터로도 똑같이 보고 싶다면 아래 주석 해제
        // printf("%s", tx_buf); 
        
        bt_send_cnt = 0;
    }
    */
    
    //printf("Pitch = %.2f, Yaw = %.2f, Gyro = %.2f, Roll = %.2f\n", current_pitch*180/M_PI, current_yaw*180/M_PI, gyro, current_roll);
}