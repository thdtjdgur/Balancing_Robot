#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h" // [추가됨] 풀업 저항 설정을 위해 추가
#include "lidar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "variable.h"
#include <math.h>

static const char *TAG = "LIDAR_G2";
// [복구됨] 43, 44는 ESP32-S3의 기본 콘솔 핀과 충돌할 위험이 커서 다시 18, 17로 복구했습니다.
#define TX_PIN 41
#define RX_PIN 42

// UART 이벤트를 전달받을 큐
QueueHandle_t lidar_uart_queue;

// 외부에서 사용할 전역 거리 배열
float distance_map[360] = {0.0f, };

void parse_lidar_packet(uint8_t *buf, int len) {
    uint8_t CT = buf[2];
    uint8_t LSN = buf[3]; 
    uint16_t FSA = buf[4] | (buf[5] << 8); 
    uint16_t LSA = buf[6] | (buf[7] << 8); 
    uint16_t CS  = buf[8] | (buf[9] << 8); 

    // --- [수정됨: 매뉴얼 규칙에 맞춘 정확한 XOR 체크섬 검사] ---
    uint16_t calc_CS = 0x55AA; // PH (C1)
    calc_CS ^= (CT | (LSN << 8));
    calc_CS ^= FSA;
    calc_CS ^= LSA;
    
    for(int i = 0; i < LSN; i++) {
        // Si의 첫 번째 바이트는 상위 8비트를 0으로 채워서 XOR (매뉴얼 규칙)
        uint16_t s1_word1 = buf[10 + i*3 + 0]; 
        // Si의 두 번째, 세 번째 바이트를 합쳐서 XOR
        uint16_t s1_word2 = buf[10 + i*3 + 1] | (buf[10 + i*3 + 2] << 8);
        
        calc_CS ^= s1_word1;
        calc_CS ^= s1_word2;
    }

    if(calc_CS != CS) {
        // printf("체크섬 에러! 버림\n"); // 주석 해제 시 에러 발생 여부 확인 가능
        return; 
    }

    // [추가됨: 생존 신고] 체크섬 통과 후 1초에 한두 번만 패킷 수신 상태 출력
    static int alive_cnt = 0;
    //if (alive_cnt++ % 20 == 0) {
    //    printf("라이다 데이터 정상 수신 중! (샘플: %d개)\n", LSN);
    //}

    // 2. 1단계 각도 분석
    float fsa_deg = (FSA >> 1) / 64.0f;
    float lsa_deg = (LSA >> 1) / 64.0f;
    
    float diff_angle = lsa_deg - fsa_deg;
    if(diff_angle < 0) diff_angle += 360.0f;

    // 3. 개별 샘플(S node) 거리 및 2단계 각도 보정
    for(int i = 0; i < LSN; i++) {
        uint8_t s2 = buf[10 + i*3 + 1];
        uint8_t s3 = buf[10 + i*3 + 2];

        float dist_mm = (float)((s3 << 6) + (s2 >> 2)); 

        if (dist_mm == 0.0f) continue; 

        float angle_i = fsa_deg;
        if (LSN > 1) {
            angle_i += (diff_angle / (LSN - 1)) * i;
        }

        float ratio = 21.8f * (155.3f - dist_mm) / (155.3f * dist_mm);
        float ang_correct_rad = atanf(ratio); 
        float ang_correct_deg = ang_correct_rad * (180.0f / M_PI); 

        float final_angle = angle_i + ang_correct_deg;
        
        while(final_angle >= 360.0f) final_angle -= 360.0f;
        while(final_angle < 0.0f) final_angle += 360.0f;

        int angle_idx = (int)(final_angle + 0.5f) % 360;
        distance_map[angle_idx] = dist_mm;

        // 0도(정면) 데이터 출력
        if (angle_idx == 0) {
            printf("정면(0도) 거리: %.1f mm\n", dist_mm);
        }
    }
}


// --- [라이다 UART 수신 태스크 (버퍼 폭발 방어 적용)] ---
void lidar_event_task(void *pvParameters)
{
    uart_event_t event;
    // 버퍼 크기를 1024 -> 2048로 확장 (오버플로우 방어)
    uint8_t* rx_buf = (uint8_t*) malloc(8192); 
    uint8_t* packet_buf = (uint8_t*) malloc(8192); 
    int head = 0;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // [핵심 해결책 3] 라이다 기절 방지용 Soft Restart (0xA5 0x40) 먼저 발사
    uint8_t reset_cmd[] = {0xA5, 0x40};
    uart_write_bytes(UART_NUM_1, reset_cmd, 2);
    uart_wait_tx_done(UART_NUM_1, portMAX_DELAY); // ★ 수면 모드 방지: 전송 완료까지 CPU 대기!
    ESP_LOGI(TAG, "Soft Restart Command Sent (0xA5 0x40)");
    
    vTaskDelay(1500 / portTICK_PERIOD_MS); // 라이다 재부팅 시간 1.5초 대기

    // 시스템 커맨드: 라이다 스캔 시작 (0xA5 0x60)
    uint8_t start_cmd[] = {0xA5, 0x60};
    uart_write_bytes(UART_NUM_1, start_cmd, 2);
    uart_wait_tx_done(UART_NUM_1, portMAX_DELAY); // ★ 수면 모드 방지: 전송 완료까지 CPU 대기!
    ESP_LOGI(TAG, "Scan Start Command Sent (0xA5 0x60)");

    for(;;) {
        if(xQueueReceive(lidar_uart_queue, (void * )&event, pdMS_TO_TICKS(2000))) {
        //lidar_uart_queue라는 큐(우체통)에서 한 이벤트를 꺼낸 후 event안에 그 내용이 복사됨.
        //처음에 portMAX_DELAY라는 옵션때문에(현재는 2초 대기로 수정됨) 이벤트가 올때까지 이 줄에서 코드를 멈추고 sleep상태에 들어감.
        //이벤트가 발생하면(event에 값이 담기면) 이 줄 다음으로 실행됨.
            if(event.type == UART_DATA) 
            {
                // printf("수신 알림! 뭔가 데이터가 들어왔음: %d 바이트\n", event.size);
                if(event.size > 2000) event.size = 2000;
                //event.size는 122바이트일거라고 예상하지만 보통 그것보다 큼. 그 이유는 엔코더, imu, pid계산을 하느라
                //122바이트가 차서 알림은 가지만 cpu가 재때 읽지 못해서 큐(lidar_uart_queue)계속 쌓임
                //나중에 cpu가 시간이 나면 큐에 쌓인 데이터를 한꺼번에 event로 넘겨서 122바이트보다 큰 값이 넘어가는거임.

                int rx_len = uart_read_bytes(UART_NUM_1, rx_buf, event.size, portMAX_DELAY);
                //lidar_uart_queue에서 event.size만큼 데이터(바이트)를 rx_buf배열로 가져와라.
                //rx_len는 실제로 몇바이트를 복사하는데 성공했는지가 저장됨. 정상적인 상황이면 event.size와 같은 숫자가 저장됨.

                // 데이터 프레이밍 (조립)
                for(int i = 0; i < rx_len; i++) {
                    // 패킷 배열 인덱스 초과 방어
                    if (head >= 2000) head = 0;
                    
                    packet_buf[head++] = rx_buf[i];

                    if(head >= 10) {//for문이 계속 돌다가 head가 10이 되어 여기로 들어옴
                        if(packet_buf[0] != 0xAA || packet_buf[1] != 0x55) {
                            for(int k = 1; k < head; k++) packet_buf[k-1] = packet_buf[k];
                            //packet_buf의 값을 왼쪽으로 한칸씩 시프트
                            head--;
                            continue;
                        }

                        int LSN = packet_buf[3]; 
                        int expected_len = 10 + (LSN * 3);
                        //PH(2B) + CT(1B) + LSN(1B) + FSA(2B) + LSA(2B) + CS(2B) == 총 10바이트
                        //Si는 1개당 3바이트.

                        // 쓰레기값 때문에 길이를 엄청나게 길게 인식하는 것 방어
                        if (expected_len > 1024) {
                            head = 0; 
                            continue;
                        }

                        if(head >= expected_len) {
                            //packet_buf에 쌓인 데이터의 총 개수가 expected_len보다 크거나 같아짐.
                            //즉 온전한 패킷1개가 완성되었다는것
                            parse_lidar_packet(packet_buf, expected_len);
                            //실제 거리와 각도를 계산하는 함수

                            int remain = head - expected_len;
                            for(int k = 0; k < remain; k++) packet_buf[k] = packet_buf[expected_len + k];
                            head = remain; 
                        }
                    }
                }
            }
            else if(event.type == UART_FIFO_OVF)
            {
                ESP_LOGW(TAG, "Lidar UART FIFO Overflow!");
                uart_flush_input(UART_NUM_1);
                xQueueReset(lidar_uart_queue);
            }
            else
            {
                // [핵심 해결책 2] 에러 폭주로 인한 CPU 기절 방어 (강제 휴식 및 큐 청소)
                // ESP_LOGW(TAG, "UART 에러 발생! 타입 번호: %d (무시하고 대기)", event.type);
                //uart_flush_input(UART_NUM_1); 
                //xQueueReset(lidar_uart_queue);
            }
        }
        else 
        {
            // 2초 동안 큐에 아무 이벤트(데이터)가 안 들어왔다면 라이다가 멈춰있다고 판단하고 강제로 스캔 명령 재전송
            int sent_bytes = uart_write_bytes(UART_NUM_1, start_cmd, 2);
            uart_wait_tx_done(UART_NUM_1, portMAX_DELAY); // 수면 모드 방지: 전송 완료까지 CPU 대기
            ESP_LOGW(TAG, "라이다 침묵 중... 명령 재전송 시도! (결과: %d 바이트 씀)", sent_bytes);
        }
    }
    free(rx_buf);
    free(packet_buf);
    vTaskDelete(NULL);

}

void init_lidar()
{
    const int uart_buffer_size = (1024 * 8);//수신버퍼 크기
    const uart_port_t uart_num = UART_NUM_1;

    gpio_reset_pin(TX_PIN);
    gpio_reset_pin(RX_PIN);

    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,//122
        // 절전 모드 진입 시 클럭이 차단되는 것을 방지하기 위해 DEFAULT 클럭으로 설정
        .source_clk = UART_SCLK_DEFAULT, 
    };
    
    // 순서 변경: 파라미터 셋팅 -> 핀 맵핑 -> 드라이버 설치 순으로 해야 핀이 안 꼬임!
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &lidar_uart_queue, 0));

    // [핵심 해결책 1] RX 핀이 0V로 죽어서 UART_BREAK 에러가 뜨는 것을 막기 위해 내부 풀업 저항 강제 활성화!
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);

    xTaskCreate(lidar_event_task, "lidar_event_task", 8192, NULL, 3, NULL);
}