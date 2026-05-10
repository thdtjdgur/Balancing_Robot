#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "rx28.h"
#include "variable.h"
#include <math.h>

// ========== [추가된 부분 시작: 수학 함수 및 LUT 제어 변수] ==========
extern float roll_adj_mm; // app_main.c에서 PID로 계산된 보상 길이(mm)

// 질문자님이 구하신 황금 데이터 (0도 포함 7개 포인트)
const float knee_deg[7] = {0.0f, 16.0f, 33.0f, 49.0f, 65.0f, 82.0f, 98.0f};
const int16_t hip_rx_data[7]  = {0, 34, 68, 102, 136, 170, 205};
const int16_t knee_rx_data[7] = {0, 53, 111, 167, 223, 280, 334};

float leg_heights[7]; // 각 포인트일 때의 다리 전체 길이(mm) 저장용

// 높이(H)를 입력받아 보간된 골반/무릎 RX값을 반환하는 함수
void get_balanced_angles(float target_H, int16_t *out_hip, int16_t *out_knee) {
    if (target_H >= leg_heights[0]) {
        *out_hip = hip_rx_data[0]; *out_knee = knee_rx_data[0]; return;
    }
    if (target_H <= leg_heights[6]) {
    //계산된 골반과 무릎사이의 거리가 제일 굽혔을때의 허벅지와 무릎 사이의 각도보다 더 작을때
        *out_hip = hip_rx_data[6]; *out_knee = knee_rx_data[6]; return;
    }

    // 현재 목표 높이가 어느 구간에 있는지 찾아서 선형 보간
    for (int i = 0; i < 6; i++) {
        if (target_H <= leg_heights[i] && target_H > leg_heights[i+1]) {
            float ratio = (leg_heights[i] - target_H) / (leg_heights[i] - leg_heights[i+1]);
            //ratio는 현재 계산된 골반과 바퀴 사이의 거리가 이 구간에서 몇% 지점에 위치하는가에 대한 0.0~1.0 사이의 값임
            *out_hip = hip_rx_data[i] + (int16_t)((hip_rx_data[i+1] - hip_rx_data[i]) * ratio);
            *out_knee = knee_rx_data[i] + (int16_t)((knee_rx_data[i+1] - knee_rx_data[i]) * ratio);
            return;
        }
    }
}
// ========== [추가된 부분 끝] ==========

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

void init_rx28()
{
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);//MAX485의 DE, RE핀에 연결되는 GPIO2핀
    gpio_set_level(GPIO_NUM_2, 0); // GPIO2초기설정

    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);//UART드라이버 설치
     
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {//통신 파라미터 설정
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 10,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));//ESP32보드 핀의 UART핀 설정
}

//2초마다 RX-28 패킷을 쏘는 전담 요리사(쓰레드)
void rx28_task(void *pvParameters) {
    // ========== [추가된 부분 시작: 태스크 시작 시 다리 높이 초기 계산] ==========
    for(int i = 0; i < 7; i++) {
        float inner_angle_rad = (180.0f - knee_deg[i]) * M_PI / 180.0f;
        leg_heights[i] = sqrtf(111.1f*111.1f + 113.0f*113.0f - 2.0f*111.1f*113.0f * cosf(inner_angle_rad));
        //삼각형에서 두 변의 거리와 끼인각을 알때 끼인각과 마주보는 변의 길이 구하는 공식
        //leg_heights는 6개의 무릎 각도에 대해 허벅지, 종아리, 골반과 바퀴사이의 거리에 대한 삼각형에서의 미리 계산된 골반과 바퀴사이의 거리임
    }
    float H_max = leg_heights[0]; // 다리를 완전히 폈을 때 골반에서 바퀴까지의 거리
    // ========== [추가된 부분 끝] ==========

    // 1초 주기로 계속 반복해서 패킷 전송
    while(1) {
        // ========== [추가 및 수정된 부분 시작: 실시간 제어를 위해 패킷 생성부를 while문 안으로 이동 및 보간 로직 적용] ==========
        float H_left = H_max;
        float H_right = H_max;
        
        
        // roll_adj_mm에 따라 다리 길이 조절 (양수면 왼쪽 굽힘, 음수면 오른쪽 굽힘)
        if (roll_adj_mm > 0.0f) {
            H_right = H_max - roll_adj_mm; //roll_adj_mms는 roll_pid를 통해서 땅과 바퀴가 떨어진 거리로 실시간 갱신중임
        } else if (roll_adj_mm < 0.0f) { //즉 허벅지, 골반, 허벅지와 골반사이의 거리에 대한 삼각형에서 H_right는 바퀴와 골반사이의 거리(이만큼 굽혀야하는 거리)가 실시간 계산됨
            H_left = H_max - (-roll_adj_mm); 
        }
        

        // 보간 함수를 통해 좌/우 독립적인 골반, 무릎 RX 값 획득
        int16_t up_l = 0, down_l = 0;
        int16_t up_r = 0, down_r = 0;
        get_balanced_angles(H_left, &up_l, &down_l);
        get_balanced_angles(H_right, &up_r, &down_r);

        // 기존 코드의 주석 보존
        // int16_t  up_rx = 0;  //양수일때 골반이 뒤로꺽임. 골반각도 205(60도), 170(50도), 136(40도), 102(30도), 68(20도), 34(10도), (0도)
        // int16_t  down_rx = 0;//양수일때 무릎 앞으로꺽임. 무릎각도 334(98도), 280(82도), 223(65도), 167(49도), 111(33도), 53(16도), (0도)
        // up_rx = (int16_t)(up_rx * 3.41f);
        // down_rx = (int16_t)(down_rx * 3.41f);

        // 여기서 원하는 위치값(0~1023)변경 (좌/우 독립 변수 up_l, up_r, down_l, down_r 적용)
        uint16_t pos_id0 = CLAMP(512 + up_l, 0, 1023); // 0번 왼쪽골반모터 512-1
        uint16_t pos_id1 = CLAMP(512 + 60 - up_r, 0, 1023); // 1번 오른쪽골반모터 512 + 61

        uint16_t pos_id2 = CLAMP(512 - 28 - down_l, 0, 1023); // 2번 왼쪽무릎모터 512 - 28
        uint16_t pos_id3 = CLAMP(512 + 30 + down_r, 0, 1023); // 3번 오른쪽무릎모터 512 + 30

        // ==================================================================================
        // [SYNC WRITE 교체 부분 시작] 기존 12번의 개별 통신을 3번의 동기화 통신으로 압축
        // ==================================================================================

        // 1. 토크 인에이블(0x18) SYNC WRITE 패킷 (모터 4개 동시 켬)
        uint8_t sync_tq[] = {
            0xFF, 0xFF, 0xFE, 0x0C, 0x83, 0x18, 0x01,
            0x00, 0x01, // ID 0
            0x01, 0x01, // ID 1
            0x02, 0x01, // ID 2
            0x03, 0x01, // ID 3
            0x00        // Checksum
        };
        uint32_t sum_tq = 0;
        for(int i = 2; i < 15; i++) sum_tq += sync_tq[i];
        sync_tq[15] = ~(sum_tq) & 0xFF;

        // 2. 이동 속도(0x20) SYNC WRITE 패킷 (모터 4개 속도 동시 0x0064로 설정)
        uint8_t sync_spd[] = {
            0xFF, 0xFF, 0xFE, 0x10, 0x83, 0x20, 0x02,
            0x00, 0x96, 0x00, // ID 0
            0x01, 0x96, 0x00, // ID 1
            0x02, 0x96, 0x00, // ID 2
            0x03, 0x96, 0x00, // ID 3
            0x00              // Checksum
        };
        uint32_t sum_spd = 0;
        for(int i = 2; i < 19; i++) sum_spd += sync_spd[i];
        sync_spd[19] = ~(sum_spd) & 0xFF;

        // 3. 목표 위치(0x1E) SYNC WRITE 패킷 (모터 4개 각도 동시 설정)
        uint8_t sync_pos[] = {
            0xFF, 0xFF, 0xFE, 0x10, 0x83, 0x1E, 0x02,
            0x00, (uint8_t)(pos_id0 & 0xFF), (uint8_t)((pos_id0 >> 8) & 0xFF), // ID 0
            0x01, (uint8_t)(pos_id1 & 0xFF), (uint8_t)((pos_id1 >> 8) & 0xFF), // ID 1
            0x02, (uint8_t)(pos_id2 & 0xFF), (uint8_t)((pos_id2 >> 8) & 0xFF), // ID 2
            0x03, (uint8_t)(pos_id3 & 0xFF), (uint8_t)((pos_id3 >> 8) & 0xFF), // ID 3
            0x00 // Checksum
        };
        uint32_t sum_pos = 0;
        for(int i = 2; i < 19; i++) sum_pos += sync_pos[i];
        sync_pos[19] = ~(sum_pos) & 0xFF;

        // --- 패킷 일괄 전송 ---
        
        // 토크 인에이블 전송
        gpio_set_level(GPIO_NUM_2, 1); 
        uart_write_bytes(UART_NUM_0, (const char*)sync_tq, sizeof(sync_tq)); 
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50)); 
        gpio_set_level(GPIO_NUM_2, 0); 
        vTaskDelay(pdMS_TO_TICKS(5));

        // 속도 패킷 전송
        gpio_set_level(GPIO_NUM_2, 1); 
        uart_write_bytes(UART_NUM_0, (const char*)sync_spd, sizeof(sync_spd)); 
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50)); 
        gpio_set_level(GPIO_NUM_2, 0); 
        vTaskDelay(pdMS_TO_TICKS(5));

        // 목표 위치 전송
        gpio_set_level(GPIO_NUM_2, 1); 
        uart_write_bytes(UART_NUM_0, (const char*)sync_pos, sizeof(sync_pos)); 
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50)); 
        gpio_set_level(GPIO_NUM_2, 0); 

        // ==================================================================================
        // [SYNC WRITE 교체 부분 끝] 
        // ==================================================================================

        // 1초 대기 (2000ms) 후 다시 처음부터 전송
        // ========== [수정된 부분 시작: 실시간 제어를 위해 딜레이 단축] ==========
        // 기존: vTaskDelay(pdMS_TO_TICKS(2000));
        vTaskDelay(pdMS_TO_TICKS(5));
        // ========== [수정된 부분 끝] ==========
    }
}