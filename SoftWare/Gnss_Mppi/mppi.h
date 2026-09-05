#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPPI_NUM_SECTORS 24  //라이다 섹터 개수
#define MPPI_SECTOR_WIDTH_DEG (360 / MPPI_NUM_SECTORS)  //색터 담당 각도범위

typedef struct {
    float x;      // 로봇의 로컬위치 x [m]
    float y;      // 로봇의 로컬위치 y [m]
    float psi;    // 로봇이 바라보는 방향 [rad]
    float v;      // 로봇의 현재 속도 [m/s]
    float w;      // 로봇의 현재 각속도 [rad/s]
} MPPI_State;

typedef struct {
    float v_ref;  // 목표속도 [m/s]
    float w_ref;  // 목표각속도 [rad/s]
} MPPI_Input;

typedef struct {
    float weight_goal_x;        //로봇의 위치와 목표 위치 x 오차를 각각 얼마나 중요하게 볼지
    float weight_goal_y;        //로봇의 위치와 목표 위치 y 오차를 각각 얼마나 중요하게 볼지
    float weight_heading;       //로봇이 바라보는 방향과 목표 방향과의 오차를 얼마나 중요하게 볼지
    float weight_obstacle;      //장애물 관련 비용을 얼마나 세게 줄지

    float weight_smooth_v;       //전진속도 명령 변화를 얼마나 싫어할지
    float weight_smooth_w;       //각속도 명령 변화를 얼마나 싫어할지

    float weight_input_v;        //속도 명령 자체 크기를 얼마나 싫어할지
    float weight_input_w;        //각속도 명령 자체 크기를 얼마나 싫어할지

    float dt;          // 한번의 제어주기의 시간간격
    int horizon;       // 하나의 명령줄에 들어가는 미래 명령 개수
    int num_samples;   // mppi로 계산할 명령줄 개수

    float v_min;       // 허용할 최소 전진속도 [m/s]
    float v_max;       // 허용할 최대 전진속도 [m/s]
    float w_min;       // 허용할 최소 각속도 [rad/s]
    float w_max;       // 허용할 최대 각속도 [rad/s]

    float lambda;      // MPPI 가중치 계산용 온도 파라미터

    float obs_safe_dist;   // 장애물과 최소한 유지하고 싶은 안전거리 [m]
} MPPI_Params;

extern MPPI_Params mppi_params;

void init_MPPI(void);
void MPPI_Task(void *pvParameters);
float wrap_to_pi(float angle);
