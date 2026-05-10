#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "hc06.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "variable.h"
#include <math.h>

// 로그 태그 및 핀 설정
static const char *TAG = "HC06";
#define TX_PIN 17
#define RX_PIN 18

// UART 이벤트를 전달받을 큐 (전역 변수)
QueueHandle_t uart_queue;

// 수신된 데이터를 저장할 구조체
typedef struct {
    float x;
    float y;
    float angle_diff;
} RobotCommand;

// [기능 1] 문자열 데이터를 숫자로 변환하는 파싱 함수
void parse_joystick_data(char* data) {
    RobotCommand cmd = {0, 0, 0};

    // 앱인벤터 데이터 포맷: "S,x,y,diff,E" (예: S,50,80,-30,E)
    // sscanf가 쉼표를 건너뛰고 숫자 3개를 추출합니다.
    if (sscanf(data, "S,%f,%f,%f,E", &cmd.x, &cmd.y, &cmd.angle_diff) == 3) {
        
        //  좌표계 변환 (매핑)
        // X축: 0~400 -> -200~+200 (중앙을 0으로)
        cmd.x = cmd.x - 200;
        // Y축: 0~400 -> +200~-200 (스마트폰은 아래가 +지만, 로봇은 위가 +여야 함)
        cmd.y = cmd.y - 200; 

        target_yaw_diff = cmd.angle_diff;

        // --- [직진 데드밴드] ---
        // 조이스틱을 앞으로 밀었을 때(각도 -20 ~ 20도 사이) target_yaw_diff를 0으로 고정
        if (target_yaw_diff >= -20.0f && target_yaw_diff <= 20.0f) {
            target_yaw_diff = 0.0f;
        }

        // 조이스틱이 중앙 근처(-30 ~ +30)에 오면 완전히 놓은 것으로 간주
        if(sqrtf(cmd.x * cmd.x + cmd.y * cmd.y) < 30.0f) {
            target_yaw_diff = 0.0f;
        }

        // 속도 목표치 설정
        if (sqrtf(cmd.x * cmd.x + cmd.y * cmd.y) < 85.0f) {
            targetvel_vel = 0.0f;
        } else {
            targetvel_vel = 0.5f;
        }
    }
}

// [기능 2] 데이터 감시 태스크 (수신)
void hc06_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(1024); 
    
    for(;;) {
        if(xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, 1024);

            switch(event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
                    parse_joystick_data((char*)dtmp);
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "FIFO 버퍼 가득 참! 클리어 실행");
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart_queue);
                    break;

                default:
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

// [기능 3] HC-06 초기화
void init_hc06()
{
    const int uart_buffer_size = (1024 * 2); // 데이터 버퍼크기 2048
    const uart_port_t uart_num = UART_NUM_2;

    // 1. 드라이버 설치 (이벤트 큐 연결)
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // 2. 통신 파라미터 설정
    uart_config_t uart_config = {
        .baud_rate = 115200,                
        .data_bits = UART_DATA_8_BITS,      
        .parity = UART_PARITY_DISABLE,      
        .stop_bits = UART_STOP_BITS_1,      
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,        
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // 3. 핀 설정 (TX:17, RX:18)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 4. 데이터 감시 태스크 생성
    xTaskCreate(hc06_event_task, "hc06_event_task", 4096, NULL, 12, NULL);
    
    ESP_LOGI(TAG, "HC-06 초기화 완료 (Baud: 115200, TX:17, RX:18)");
}

// [기능 4] 로봇 각도 앱으로 전송 (송신)
void data_tx()
{
    static int send_cnt = 0;
    // 100번 루프(0.1초)마다 전송
    if (send_cnt++ >= 100) { 
        char tx_buffer[64]; 
            
        // "Y:각도\n" 형식 (예: Y:120.5)
        int len = sprintf(tx_buffer, "Y:%.1f\n", current_yaw*180/M_PI);
            
        uart_write_bytes(UART_NUM_2, tx_buffer, len);
        send_cnt = 0; 
    }
}