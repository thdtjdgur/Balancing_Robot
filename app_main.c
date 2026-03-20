////ㅎㅇ
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
#include <math.h>

// extern 선언들
extern SemaphoreHandle_t encoder_sem;
extern spi_device_handle_t h_left;
extern spi_transaction_t *ret_t;

float current_vel = 0.0f;
float current_pitch = 0.0f;
float current_yaw = 0.0f;
float current_roll = 0.0f;
float roll_adj_mm = 0.0f;
float targetvel_vel = 0.0f;    
float target_yaw_diff = 0.0f;   
int vel_calc_flag;

// PID 제어기 및 전압 변수 선언
PIDController pitch_ctrl, yaw_ctrl, vel_ctrl, roll_ctrl;
float Vq_left = 0, Vq_right = 0;
const float MOTOR_V_MIN = 0.0f;//1.5 // 실험으로 찾을 최소 구동 전압

void motor_control_task(void *pvParameters) {
    while (1) {
        // 엔코더 세마포어만 기다림. (IMU 눈치 안 봄)
        if (xSemaphoreTake(encoder_sem, portMAX_DELAY) == pdTRUE) {
            //요리사b(쓰레드)는 encoder_sem라는 주문서가 올때까지 portMAX_DELAY때문에 영원히 기다림
            //encoder_sem라는 주문서가 안오면 그동안은 잠듬(cpu사용율 0%)
            //Motor_Task라는 요리사는 rtos에게 encoder_sem라는 세마포어(주문서)가 오면 깨워달라고함
            //세마포어가 도착했을때 Motor_Task라는 요리사가 수행할 motor_control_task함수 안에 xSemaphoreTake조건문을 작성했기 때문에
            //Motor_Task라는 요리사가 rtos에게 encoder_sem라는 세마포어(주문서)가 오면 깨워달라고 요청하게됨
            
            // 엔코더 계산 및 모터 출력 (여기서 Vq_left값이 모터로 들어감)
            encoder_to_vcc_cal(); 
        }
    }
}

void app_main(void) {
    //세마포어 생성 (인터럽트와 태스크 동기화용) 
    imu_sem = xSemaphoreCreateBinary();

    // PID 초기화
    pid_init(&vel_ctrl, 0.32f, 0.00f, 0.0f, 0.3f);//10.0f는 (속도pid결과값 = 목표피치각도 한계값, 지금은 최대 3도)임. p=0.43, i=0.001, limit=0.3f
    pid_init(&pitch_ctrl, 30.0f, 0.0f, 0.21f, 15.0f);//속도 pid에서 0.174라디안(10도)이 넘어왔을때 kp=100을 곱하면 17.4가 나옴(p=60 d=0.4, limit=15.0f)
    pid_init(&yaw_ctrl, 20.0f, 0.0f, 0.01f, 5.0f);//목표요를 0.4rad(20도)로 했을때 kp=20을 곱하면 8이 나옴
    pid_init(&roll_ctrl, 2.0f, 0.08f, 0.01f, 150.0f);//v_limit=50.0f (장애물 높으 120mm까지 대응하도록 제한)
     
    i2c_master_dev_handle_t imu_handle = imu_init();// IMU 디바이스 초기화 및 핸들 획득
    encoder_init();//encoder통신 초기설정 (내부에서 encoder_sem도 생성함)
    init_mcpwm_bldc();//mcpwm초기설정
    init_hc06();///hc06 초기설정
    init_rx28();
    //init_lidar();

    xTaskCreate(motor_control_task, "Motor_Task", 4096, NULL, 5, NULL);
    //motor_control_task라는 작업을 하는 Motor_Task라는 쓰레드(요리사b)가 채용되는 코드임(Motor_Task라는 쓰레드는 우선순위 5임)
    //4096바이트(4KB)라는 ram메모리공간(요리사b전용 배낭)을 할당해서 코드를 수행하게 됨.
    //즉 encoder_to_vcc_cal함수 안에서 선언한 float angle_l, float pure_l같은 변수와 prinf같은 도구가 이 메모리공간에 저장됨
    //if, while, cosf같은 명령어와 전역변수 current_vel, encoder_sem는 flash메모리에 저장됨

    xTaskCreate(rx28_task, "RX28_Task", 4096, NULL, 4, NULL);
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
            //data_tx();   

            if (current_roll > -0.5f && current_roll < 0.5f) {
                current_roll = 0.0f;
            }

            roll_adj_mm = pid_calculate(&roll_ctrl, 0.0f, current_roll, 0.005f);

            static float target_pitch = 0.0f;            

            // 속도 제어 루프 -> 목표 피치 결정
            if (vel_calc_flag == 1) {
                target_pitch = pid_calculate(&vel_ctrl, targetvel_vel, current_vel, 0.01f);//목표속도0. 나중에 hc06으로 받아와야함
                static float smoothed_target_pitch = 0.0f;
                
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

            float y_out = target_yaw_diff * 0.03f;//target_yaw_diff의 단위는 도임, 0.1f
            float MAX_TURN_V = 1.5f;
            if (y_out > MAX_TURN_V) {
                y_out = MAX_TURN_V;
            } else if (y_out < -MAX_TURN_V) {
                y_out = -MAX_TURN_V;
            }
            
            
            // 좌우 전압 분배
            float pure_l = p_out + y_out;
            float pure_r = p_out - y_out;

            float MOTOR_V_MIN = 0.10f; // 모터가 꿈쩍하기 시작하는 최소 전압

            /*
            // [핵심] 최종 출력 단계에서 최소 전압 보상 적용
            if (pure_l > 0.0f)       Vq_left = pure_l + MOTOR_V_MIN;
            else if (pure_l < -0.0f) Vq_left = pure_l - MOTOR_V_MIN;
            else                      Vq_left = 0;

            if (pure_r > 0.0f)     Vq_right = pure_r + MOTOR_V_MIN;
            else if (pure_r < -0.0f) Vq_right = pure_r - MOTOR_V_MIN;
            else                      Vq_right = 0;
            */
            
            Vq_left = pure_l;
            Vq_right = pure_r;

            static int plot_cnt = 0;
        }
    }
}