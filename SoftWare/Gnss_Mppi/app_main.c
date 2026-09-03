#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "imu.h"
#include "encoder.h"
#include "pwm.h"
#include "pid.h"
#include "hc06.h"
#include "rx28.h"
#include "variable.h"
#include "driver/uart.h"
#include "lidar.h"
#include "MPPI.h"
#include "gnss.h"
#include "waypoint.h"
#include "lorartk.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

// extern 선언들
extern SemaphoreHandle_t encoder_sem;
extern spi_device_handle_t h_left;
extern spi_transaction_t *ret_t;

float current_vel = 0.0f;
float current_pitch = 0.0f;
float current_yaw = 0.0f;
float current_roll = 0.0f;
float gyro = 0.0f;

float roll_adj_mm = 0.0f;
float targetvel_vel = 0.0f;    
float target_yaw_diff = 0.0f;   
int vel_calc_flag;

float omega_l_meas = 0.0f;
float omega_r_meas = 0.0f;

// PID 제어기 및 전압 변수 선언
PIDController pitch_ctrl, vel_ctrl, roll_ctrl, yaw_ctrl;
float Vq_left = 0, Vq_right = 0;
const float MOTOR_V_MIN = 0.0f;//1.5 // 실험으로 찾을 최소 구동 전압


static volatile int station_packet_ready = 0;
static int station_packet_len = 0;
static float station_packet[1 + (MAX_WAYPOINTS * 2)];



void motor_control_task(void *pvParameters) {
    while (1) {
        // 엔코더 세마포어만 기다림. (IMU 눈치 안 봄)
        if (xSemaphoreTake(encoder_sem, portMAX_DELAY) == pdTRUE) {
            // Motor_Task(요리사 B)는 encoder_sem 세마포어(주문서)가 올 때까지 여기서 대기함.
            // portMAX_DELAY 때문에 세마포어가 올 때까지 사실상 무한 대기함.
            // 대기 중인 Motor_Task는 Blocked 상태라서 CPU를 사용하지 않음.

            // 즉 Motor_Task는 RTOS에게
            // "encoder_sem 세마포어가 오면 나를 깨워줘"라고 요청한 상태가 됨.

            // 세마포어가 도착하는 시점은 ISR 안에서
            // xSemaphoreGiveFromISR(encoder_sem, &xHigherPriorityTaskWoken);
            // 코드가 실행될 때임.

            // 이 순간 encoder_sem 세마포어가 주어지고,
            // encoder_sem을 기다리던 Motor_Task가 실행 가능 상태(Ready)가 됨.

            // 그 다음 ISR(spi_post_callbac) 안의 portYIELD_FROM_ISR()는
            // "방금 깨어난 task가 현재 task보다 우선순위가 높으면
            // 인터럽트(spi_post_callbac)가 끝난 직후 바로 그 task로 전환해라"라고 RTOS에 요청함.

            // 그래서 ISR(spi_post_callbac)이 끝난 뒤 Motor_Task가 우선순위상 가장 높으면
            // 바로 CPU를 차지하고, xSemaphoreTake()가 pdTRUE를 반환해서
            // 이 if문 안으로 들어오게 됨.

            
            // 엔코더 계산 및 모터 출력 (여기서 Vq_left값이 모터로 들어감)
            encoder_to_vcc_cal(); 
        }
    }
}

static void spi_shared_cs_idle_init(void)
{
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_8) | (1ULL << GPIO_NUM_9) | (1ULL << GPIO_NUM_10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&cs_cfg);

    gpio_set_level(GPIO_NUM_8, 1);   // LoRa CS 비선택
    gpio_set_level(GPIO_NUM_9, 1);   // 왼쪽 AS5048A CS 비선택
    gpio_set_level(GPIO_NUM_10, 1);  // 오른쪽 AS5048A CS 비선택
}

static void configure_console_log_filter(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set("RTK_BRIDGE", ESP_LOG_INFO);
}

void app_main(void) {
    configure_console_log_filter();

    //세마포어 생성 (인터럽트와 태스크 동기화용)
    vTaskPrioritySet(NULL, 4); //메인문 태스크 우선순위 4
    imu_sem = xSemaphoreCreateBinary();

    spi_shared_cs_idle_init();

    // PID 초기화
    pid_init(&vel_ctrl, 0.35f, 0.002f, 0.0001f, 0.3f);//10.0f는 (속도pid결과값 = 목표피치각도 한계값, 지금은 최대 3도)임. p=0.43, i=0.001, limit=0.3f
    pid_init(&pitch_ctrl, 30.0f, 0.0f, 0.21f, 15.0f);//속도 pid에서 0.174라디안(10도)이 넘어왔을때 kp=100을 곱하면 17.4가 나옴(p=60 d=0.4, limit=15.0f)
    pid_init(&yaw_ctrl, 0.4f, 0.0f, 0.0001f, 2.0f);//목표요를 0.4rad(20도)로 했을때 kp=20을 곱하면 8이 나옴
    pid_init(&roll_ctrl, 2.0f, 0.10f, 0.01f, 150.0f);//v_limit=50.0f (장애물 높이 150mm까지 대응하도록 제한)
     

    i2c_master_dev_handle_t imu_handle = imu_init();// IMU 디바이스 초기화 및 핸들 획득
    encoder_init();//encoder통신 초기설정 (내부에서 encoder_sem도 생성함)
    init_mcpwm_bldc();//mcpwm초기설정
    //init_hc06();///hc06 초기설정
    init_rx28();
    init_MPPI();
    init_gnss();
    init_lorartk();
    init_lidar();


    xTaskCreate(motor_control_task, "Motor_Task", 4096, NULL, 5, NULL);
    //motor_control_task라는 작업을 하는 Motor_Task라는 쓰레드(요리사b)가 채용되는(만들어지는) 코드임(Motor_Task라는 쓰레드는 우선순위 5임)
    //4096바이트(4KB)라는 ram메모리공간(요리사b전용 배낭)을 할당해서 코드를 수행하게 됨.
    //즉 encoder_to_vcc_cal함수 안에서 선언한 float angle_l, float pure_l같은 변수와 prinf같은 도구가 이 메모리공간에 저장됨
    //if, while, cosf같은 명령어와 전역변수 current_vel, encoder_sem는 flash메모리에 저장됨
    xTaskCreate(rx28_task, "RX28_Task", 4096, NULL, 3, NULL);
    xTaskCreate(MPPI_Task, "MPPI_Task", 4096, NULL, 2, NULL);
    imu_timer_init();// imu 타이머 모듈 초기화 (200Hz 인터럽트 시작)
    encoder_timer_init();// 엔코더 타이머 모듈 초기화 (1KHz 인터럽트 시작)


    // 1KHz 엔코더 제어 및 WDT 방지 루프
    while (1) {
        // 인터럽트가 세마포어를 줄 때까지 대기 (200Hz 주기에 맞춰 실행됨)
        
        if (xSemaphoreTake(imu_sem, portMAX_DELAY) == pdTRUE) {
        //main문이 실행될때 기본적으로 시스템이 xTaskCreate를 수행하여 app_main라는 기본요리사(스레드)한명을 채용함(우선운위1)
        //세마포어가 도착했을때 app_main라는 기본요리사가 수행할 main함수 안에 xSemaphoreTake조건문을 작성했기 때문에
        //app_main라는 기본 요리사가 rtos에게 imu_sem라는 세마포어(주문서)가 오면 깨워달라고 요청하게됨

            //실제 I2C 통신 수행 (ISR 밖이므로 안전)
            imu_data_cal(imu_handle);   

            //앞, 혹은 뒤로 20도 기울어지면 강제 속도, 각속도 0으로 만들기
            if (fabsf(current_pitch) > (20.0f * M_PI / 180.0f)) {
                targetvel_vel = 0.0f;
                target_yaw_diff = 0.0f;
            }
    
            //*****gnss웨이포인트관련 함수코드 작성한뒤 그 안에서 station_packet[], station_packet_len 채우기
            //*****또한 그 함수 안에서 마지막에 station_packet_ready를 1로 해줘야함
            //*****기지국 RTCM 보정값은 LoRa로 받아서 ESP32가 UM982로 전달함.
            //*****UM982는 RTCM을 이용해 RTK 위치를 계산하고, GGA 문장으로 현재 위도/경도를 ESP32에 보냄.
            //*****ESP32는 GGA를 받을 때 update_gnss_position()을 호출해서 gnss_x, gnss_y를 갱신함.
            //*****MPPI 주기가 10Hz이므로, UM982 GGA 출력도 10Hz로 맞추는 게 좋음.
            //*****단, 현재 UM982 10Hz 출력 설정은 아직 안 함.
            if (station_packet_ready) {
                gnss_receive_complete(station_packet, station_packet_len);
                //station_packet는 웨이포인트 패킷 데이터 배열
                //station_packet_len 패킷길이. ex) 패킷 = [3, lat0, lon0, lat1, lon1, lat2, lon2]이면 station_packet_len는 7임
                station_packet_ready = 0;
            }

            //convert_station_waypoints_to_local_xy 함수에서 station_waypoint_ready가 1이 됨
            //즉 웨이포인트가 로컬좌표로 변환 완료되면 station_waypoint_ready가 1이 됨
            if (station_waypoint_ready) {
                waypoint_start();
                station_waypoint_ready = 0;
            }

            //data_tx();   

            if (current_roll > -0.5f && current_roll < 0.5f) {
                current_roll = 0.0f;
            }

            //roll_adj_mm = pid_calculate(&roll_ctrl, 0.0f, current_roll, 0.005f);// rx28제어할거면 주석해제//////////////////////////////////////////

            static float target_pitch = 0.0f;   


            // 속도 제어 루프 -> 목표 피치 결정
            if (vel_calc_flag == 1) {

                if (fabsf(targetvel_vel) != 0.0f) {
                    vel_ctrl.err_sum = 0.0f;
                }

                target_pitch = pid_calculate(&vel_ctrl, targetvel_vel, current_vel, 0.01f);//목표속도0. 나중에 hc06으로 받아와야함
                static float smoothed_target_pitch = 0.0f;
                //Low Pass Filter
                // 이전 목표치 95% + 새로운 목표치 5% (미친듯한 진동을 쫀득하게 흡수함)
                // 피치pid만 했을때는 진동이 없지만 속도pid를 넣으면 발작하는 이유는 피치pid의 d항에 의해 속도pid에서 새로운 목표각도를 넘기면 
                // 피치pid의 d항이 0.01초만에 오차가 확 변했다라고 판단하고 전압이 팍팍변해서 진동함
                // 그래서 속도pid를 포함했을때 속도pid에 의해 새로운 목표각도가 들어와도 확 꺽지말고 과거의 목표각도를 95%유지하고
                // 새로들어온 목표각도를5%만 반영해서 천천히 목표각도까지 오르도록함.
                smoothed_target_pitch = (0.95f * smoothed_target_pitch) + (0.05f * target_pitch);

                target_pitch = smoothed_target_pitch;

                vel_calc_flag = 0;
            }

            // 피치 및 요 PID 계산 (순수 보정 전압)
            //float p_out = pid_calculate(&pitch_ctrl, target_pitch, current_pitch, 0.005f); //피치pid의 목표피치를 0으로 하고 안정화되면 아 주석 pid값 맞추기
            float p_out = pid_calculate(&pitch_ctrl, target_pitch, current_pitch, 0.005f);

            float y_out = pid_calculate(&yaw_ctrl, target_yaw_diff, gyro, 0.005f);  
            //float y_out = target_yaw_diff * 0.03f;//target_yaw_diff의 단위는 라디안임, 0.1f
            
            
            float MAX_TURN_V = 1.5f;
            if (y_out > MAX_TURN_V) {
                y_out = MAX_TURN_V;
            } else if (y_out < -MAX_TURN_V) {
                y_out = -MAX_TURN_V;
            }
            

            //float MAX_TURN_V = 1.5f;
            // float YAW_MIN_V  = 0.8f;
            // //MPPI구현할때 YAW출력전압을 0.8을 줬을때의 로봇의 각속도를 측정하여 그 값을
            // //최소 실현 가능한 각속도로 설정하자
            // float YAW_CMD_DEADBAND = 0.02f;   // 단위는 네 target_yaw_diff 단위에 맞춤

            // if (fabsf(target_yaw_diff) < YAW_CMD_DEADBAND) {
            //     y_out = 0.0f;
            // } 
            // else {
            //     if (y_out > 0.0f && y_out < YAW_MIN_V) {
            //         y_out = YAW_MIN_V;
            //     } else if (y_out < 0.0f && y_out > -YAW_MIN_V) {
            //         y_out = -YAW_MIN_V;
            //     }
            // }

            // if (y_out > MAX_TURN_V) y_out = MAX_TURN_V;
            // else if (y_out < -MAX_TURN_V) y_out = -MAX_TURN_V;
            
            // 좌우 전압 분배
            float pure_l = p_out + y_out;
            float pure_r = p_out - y_out;

            float MOTOR_V_MIN = 0.10f; // 모터가 꿈쩍하기 시작하는 최소 전압


            static int hold_sign_l = 0;
            static int hold_sign_r = 0;

            float STICK_V = 0.18f;
            float CMD_ON  = 0.005f;
            float CMD_OFF = 0.001f;

            if (fabsf(targetvel_vel) < 0.02f && fabsf(current_vel) < 0.03f) {

                if (pure_l > CMD_ON) hold_sign_l = 1;
                else if (pure_l < -CMD_ON) hold_sign_l = -1;
                else if (fabsf(pure_l) < CMD_OFF) hold_sign_l = 0;

                if (pure_r > CMD_ON) hold_sign_r = 1;
                else if (pure_r < -CMD_ON) hold_sign_r = -1;
                else if (fabsf(pure_r) < CMD_OFF) hold_sign_r = 0;

                Vq_left = pure_l + hold_sign_l * STICK_V;
                Vq_right = pure_r + hold_sign_r * STICK_V;
            } 
            else 
            {
                Vq_left = pure_l;
                Vq_right = pure_r;
            }

            static int plot_cnt = 0;
        }
    }
}
