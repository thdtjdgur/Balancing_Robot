#pragma once

#include "waypoint.h"

extern float gnss_x;//gnss_rtk로받은 위도경도를 이용한 로컬x좌표
extern float gnss_y;//gnss_rtk로받은 위도경도를 이용한 로컬y좌표

extern float goal_x;//웨이포인트의 위도경도를 이용한 x좌표
extern float goal_y;//웨이포인트의 위도경도를 이용한 y좌표

extern volatile int station_waypoint_ready;
extern int station_count;
extern float station_x[MAX_WAYPOINTS];
extern float station_y[MAX_WAYPOINTS];

void init_gnss(void);
int gnss_is_initialized(void);
void update_gnss_position(float latitude, float longitude);
void update_waypoint_position(float waypoint_latitude, float waypoint_longitude);
void gnss_receive_complete(const float *packet, int packet_len);

