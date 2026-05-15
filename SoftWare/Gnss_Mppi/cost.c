#include <math.h>
#include "cost.h"
#include "gnss.h"
#include "mppi.h"
#include "lidar.h"


// 각 섹터의 대표 최소거리 [m]
static  float sector_distance_m[MPPI_NUM_SECTORS];

// 각 섹터별 가장 가까운 장애물이 잡힌 각도
static  float sector_closest_angle_rad[MPPI_NUM_SECTORS];


/////////////////////////////////////////////////////////////비용함수 4개(heading비용함수 제외 개수)
/////////////////////////////////////////////////////////////
// 목표 위치와의 x, y 오차 비용
float calc_goal_cost(const MPPI_State *state)
{
    float error_x = goal_x  - state->x;
    float error_y = goal_y - state->y;

    float cost_x = mppi_params.weight_goal_x * error_x * error_x;
    float cost_y = mppi_params.weight_goal_y * error_y * error_y;

    return cost_x + cost_y;
}


/*
// 목표점 방향을 제대로 바라보고 있는지 비용 계산
// 현재 위치에서 목표 위치를 향하는 각도를 psi_ref로 만들고,
// 현재 heading과의 차이를 제곱해서 비용으로 사용
float calc_heading_cost(const MPPI_State *state)
{
    float psi_ref = atan2f(goal_y  - state->y,
                           goal_x  - state->x);

    float error_heading = wrap_to_pi(state->psi - psi_ref);

    return mppi_params.weight_heading * error_heading * error_heading;
}
*/


//현재 로봇 위치에서 360도 라이다 거리맵을 24개 섹터의 대표 최소거리로 압축
//한 섹터 안에 여러 각도가 있으면 그중 가장 가까운 거리만 사용
void build_lidar_sectors(void)
{
    for (int i = 0; i < MPPI_NUM_SECTORS; i++) {
        sector_distance_m[i] = 999.0f;//999m, 장애물이 근처에 없다라고 초기화
        //sector_distance_m[i]는 각 섹터 대표 최소거리
    }

    for (int angle_deg = 0; angle_deg < 360; angle_deg++) {
        //측정 실패값은 무시
        if (distance_map[angle_deg] <= 0.0f) {
            continue;
        }

        float dist_m = distance_map[angle_deg] * 0.001f; // mm -> m
        int sector_idx = angle_deg / MPPI_SECTOR_WIDTH_DEG;
        //angle_deg가 몇번섹터에 속하는지 계산하는 식

        if (sector_idx >= MPPI_NUM_SECTORS) {
            sector_idx = MPPI_NUM_SECTORS - 1;
        }//유효 인덱스 제한코드

        if (dist_m < sector_distance_m[sector_idx]) {
            sector_distance_m[sector_idx] = dist_m;
            //한 섹터 안의 여러 점들 중 가장 가까운 장애물까지의 거리값이 sector_distance_m[sector_idx](섹터 대표 거리값)에 저장됨
            sector_closest_angle_rad[sector_idx] = angle_deg * M_PI / 180.0f;
            //가장 가까운 장애물이 위치한 각도를 sector_closest_angle_rad[sector_idx]에 저장
        }
    }
}

//명령줄 500중 각각에 대해 아래 함수를 실행해서 어떤 위치에 로봇이 있을때의 
//장애물까지의 거리를 아용해 그 명령줄에 대한 비용을 계산하는 함수 
//현재 스캔을 기준으로 각 섹터 장애물 점을 world 좌표로 만든 뒤,
//미래 예측 상태와의 최소거리를 이용해 장애물 비용 계산
float calc_sector_obstacle_cost(const MPPI_State *pred_state,//미래 로봇 상태
                const MPPI_State *scan_origin_state)//라이다 읽은순간 로봇상태
{
    float total_cost = 0.0f;

    for (int i = 0; i < MPPI_NUM_SECTORS; i++) {
        // 유효한 거리 없으면 무시
        if (sector_distance_m[i] >= 999.0f) {
            continue;
        }

        float obs_dist = sector_distance_m[i];
        float obs_angle_world = scan_origin_state->psi + sector_closest_angle_rad[i];

        // 현재 스캔 시점 기준 장애물 점의 world 좌표 구하는 공식
        float obs_x = scan_origin_state->x + obs_dist * cosf(obs_angle_world);//원점좌표 기준 장애물까지의 x좌표
        float obs_y = scan_origin_state->y + obs_dist * sinf(obs_angle_world);//원점좌표 기준 장애물까지의 y좌표

        float dx = obs_x - pred_state->x;//미래 로봇 위치와 장애물 x좌표의 차이
        float dy = obs_y - pred_state->y;//미래 로봇 위치와 장애물 y좌표의 차이
        float dist_to_obstacle = sqrtf(dx * dx + dy * dy);//미래 로봇 위치와 장애물 사이의 거리

        // 안전거리 안쪽으로 들어오면 비용 증가
        if (dist_to_obstacle < mppi_params.obs_safe_dist) {
            float error = mppi_params.obs_safe_dist - dist_to_obstacle;
            total_cost += mppi_params.weight_obstacle * error * error;
        }
    }

    return total_cost;
}


// 명령 자체가 너무 큰 것을 싫어하도록 하는 비용
// 너무 빠른 속도, 너무 큰 각속도 명령에 벌점 부여
float calc_input_cost(const MPPI_Input *input)
{
    float cost_v = mppi_params.weight_input_v * input->v_ref * input->v_ref;
    float cost_w = mppi_params.weight_input_w * input->w_ref * input->w_ref;

    return cost_v + cost_w;
}


// 이전 명령과 비교해서 새 명령이 갑자기 많이 바뀌면 벌점 부여
float calc_smooth_cost(const MPPI_Input *input, const MPPI_Input *prev_input)
{
    float dv = input->v_ref - prev_input->v_ref;
    float dw = input->w_ref - prev_input->w_ref;

    float cost_v = mppi_params.weight_smooth_v * dv * dv;
    float cost_w = mppi_params.weight_smooth_w * dw * dw;

    return cost_v + cost_w;
}
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////