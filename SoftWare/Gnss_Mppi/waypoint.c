#include <math.h>
#include "waypoint.h"
#include "gnss.h"
#include "variable.h"

#define WAYPOINT_REACH_RADIUS_M 4.0f //100cm이내로 들어오면 웨이포인트에 도착한걸로 간주
#define WAYPOINT_REACH_CONFIRM_COUNT 1
#undef WAYPOINT_REACH_RADIUS_M
#define WAYPOINT_REACH_RADIUS_M 4.0f // 4m 안으로 들어오면 웨이포인트 도착으로 판단

typedef struct {
    float x;
    float y;
} WaypointXY;

static int current_waypoint_idx = 0;
static int mission_active = 0;
static int goal_loaded = 0;
static int reach_confirm_count = 0;

//로봇과 웨이포인트 사이 거리함수
static float calc_goal_distance_m(void)
{
    float dx = goal_x - gnss_x;
    float dy = goal_y - gnss_y;
    return sqrtf(dx * dx + dy * dy);
}

void waypoint_reset(void)
{
    current_waypoint_idx = 0;//지금 목표로 삼고 있는 웨이포인트 번호. 0이면 첫 번째 점으로 가는중
    mission_active = 0;//웨이포인트 미션이 진행중인지 표시
    goal_loaded = 0;//현재 목표 웨이포인트가 goal_x, goal_y에 이미 반영됐는지 표시
    reach_confirm_count = 0;//목표점 반경 안에 몇 번 연속 들어왔는지 세는 카운트
}

void waypoint_start(void)
{   
    waypoint_reset();

    mission_active = 1;//미션 시작
}

void waypoint_update(void)
{
    if (!mission_active) {//mission_active는 모든웨이포인트 다 돌때까지 임1
        return;//처음 시작할때 한번만 들어옴
    }

    if (!goal_loaded) {//처음 시작할때 한번만 들어옴. current_waypoint_idx를 통해 목표위치를 변경해나감
        goal_x = station_x[current_waypoint_idx];
        goal_y = station_y[current_waypoint_idx];
        goal_loaded = 1;
        reach_confirm_count = 0;
        return;
    }

    if (calc_goal_distance_m() < WAYPOINT_REACH_RADIUS_M) 
    {
        reach_confirm_count++;
    } 
    else 
    {
        reach_confirm_count = 0;
    }

    //목표점에 일정시간동안 머물러야 도착한걸로 간주
    if (reach_confirm_count < WAYPOINT_REACH_CONFIRM_COUNT) {
        return;
    }

    reach_confirm_count = 0;
    current_waypoint_idx++;

    //최종 웨이포인트 끝까지 도달하면 그 자리에서 멈춤
    if (current_waypoint_idx >= station_count) {
        mission_active = 0;
        goal_loaded = 0;
        targetvel_vel = 0.0f;
        target_yaw_diff = 0.0f;
        return;
    }

    goal_x = station_x[current_waypoint_idx];
    goal_y = station_y[current_waypoint_idx];
}

int waypoint_is_active(void)
{
    return mission_active;
}

int waypoint_get_current_index(void)
{
    return current_waypoint_idx;
}

int waypoint_get_count(void)
{
    return station_count;
}
