#pragma once

#define MAX_WAYPOINTS 32

void waypoint_reset(void);
void waypoint_start(void);
void waypoint_update(void);

// 기지국에서 한 번에 여러 x/y 웨이포인트를 받았을 때 호출
void waypoint_load_mission_xy(const float *x_list, const float *y_list, int count);

int waypoint_is_active(void);
int waypoint_get_current_index(void);
int waypoint_get_count(void);
