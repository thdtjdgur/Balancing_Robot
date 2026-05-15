#include <math.h>
#include "gnss.h"
#include "waypoint.h"

//동쪽으로 이동하면 x좌표 증가
//북쪽으로 이동하면 y좌표 증가

float gnss_x = 0.0f;
float gnss_y = 0.0f;
float goal_x = 0.0f;
float goal_y = 0.0f;

volatile int station_waypoint_ready = 0;

int station_count = 0;//웨이포인트 xy좌표 쌍 개수
float station_x[MAX_WAYPOINTS];
float station_y[MAX_WAYPOINTS];

static float ref_latitude = 0.0f;//기준 위도
static float ref_longitude = 0.0f;//기준 경도
static int gnss_ref_initialized = 0;

void init_gnss(void)
{
    gnss_x = 0.0f;
    gnss_y = 0.0f;
    goal_x = 0.0f;
    goal_y = 0.0f;

    ref_latitude = 0.0f;
    ref_longitude = 0.0f;
    gnss_ref_initialized = 0;
}


//패킷안의 웨이포인트 좌표를 변수에 저장하고 패킷받았다는 플래그 1로 표시
void station_waypoint_packet_received(const float *packet, int packet_len)
{
    if (packet == 0 || packet_len <= 0) {
        return;
    }

    int count = (int)packet[0];
    //패킷 첫번째 값이 웨이포인트 x,y쌍의 개수라고 가정하고 count에 저장

    if (count < 0) {
        count = 0;
    }

    //웨이포인트 개수가 배열최대개수보다 많으면 잘라냄
    if (count > MAX_WAYPOINTS) {
        count = MAX_WAYPOINTS;
    }

    if (packet_len < (1 + count * 2)) {
        return;
    }

    station_count = count;//웨이포인트 xy좌표 쌍 개수를 station_count에 저장

    for (int i = 0; i < count; i++) {
        station_x[i] = packet[1 + (2 * i)];
        station_y[i] = packet[1 + (2 * i) + 1];
    }

    station_waypoint_ready = 1;
}



int gnss_is_initialized(void)
{
    return gnss_ref_initialized;
}

void update_gnss_position(float latitude, float longitude)//gnss rtk모듈에서 받은 위도, 경도가
{
    // 처음 들어온 위치를 기준점으로 사용하고 함수호출. 그다음부터 위도, 경도 들어오면 기준점 위도, 경도를 사용해서 이동 계산
    if (!gnss_ref_initialized) {
        ref_latitude = latitude;
        ref_longitude = longitude;
        gnss_ref_initialized = 1;
        gnss_x = 0.0f;
        gnss_y = 0.0f;
        return;
    }

    //위도, 경도 차이를 미터 단위 local x,y로 근사 변환
    //기준 위도만 라디안으로 바꿈
    float lat_rad = ref_latitude * (float)M_PI / 180.0f;

    //현재 위도, 경도에서 기준 위도, 경도를 뺌
    float d_lat = latitude - ref_latitude;//위도 각도차이
    float d_lon = longitude - ref_longitude;//경도 각도차이

    // 위도 1도 차이는 대략 111320m이다.
    // 위도 차이를 이용하면 남북 방향 거리(y)를 근사적으로 구할 수 있다.

    // 경도 1도 차이에 해당하는 실제 동서 방향 거리는 위도에 따라 달라진다.
    // 적도에서는 가장 크고, 북극이나 남극으로 갈수록 점점 작아진다.
    // 이는 위도가 높아질수록 해당 위도에서의 동서 방향 원의 반지름이 줄어들기 때문이다.

    // 그래서 경도 차이를 거리로 바꿀 때는 cos(위도)를 곱해 보정한다.
    // 위도가 90도 극지방이면 cos(90)=0이므로
    // 경도 차이가 있어도 동서 방향 거리 차이는 0에 가까워진다.
    gnss_y = d_lat * 111320.0f;
    gnss_x = d_lon * 111320.0f * cosf(lat_rad);
}

void update_waypoint_position(float waypoint_latitude, float waypoint_longitude)
{
    if (!gnss_ref_initialized) {
        goal_x = 0.0f;
        goal_y = 0.0f;
        return;
    }
    //로봇 위도, 경도가 업데이트 되면 아래 코드로 들어감
    float lat_rad = ref_latitude * (float)M_PI / 180.0f;
    float d_lat = waypoint_latitude - ref_latitude;         //웨이포인트 위도 - 로봇의 기준 위도
    float d_lon = waypoint_longitude - ref_longitude;       //웨이포인트 경도 - 로봇의 기준 경도

    goal_y = d_lat * 111320.0f;                             //로봇의 기준x좌표로부터 떨어진 거리(m)
    goal_x = d_lon * 111320.0f * cosf(lat_rad);             //로봇의 기준y좌표로부터 떨어진 거리(m)
}


void gnss_receive_complete(const float *packet, int packet_len)
{
    if (packet == 0 || packet_len <= 0) {
        return;
    }

    station_waypoint_packet_received(packet, packet_len);
}