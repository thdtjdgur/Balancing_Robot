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
float station_x[MAX_WAYPOINTS];//웨이포인트 xy좌표
float station_y[MAX_WAYPOINTS];

static float station_lat[MAX_WAYPOINTS];//웨이포인트 위도, 경도좌표
static float station_lon[MAX_WAYPOINTS];
static int station_waypoint_pending = 0;

static float ref_latitude = 0.0f;//로봇의 맨 처음 위도
static float ref_longitude = 0.0f;//로봇의 맨 처음 경도
static int gnss_ref_initialized = 0;


static void latlon_to_local_xy(float latitude, float longitude, float *x, float *y)
{
    float lat_rad = ref_latitude * (float)M_PI / 180.0f;
    float d_lat = latitude - ref_latitude;
    float d_lon = longitude - ref_longitude;

    *y = d_lat * 111320.0f;
    *x = d_lon * 111320.0f * cosf(lat_rad);
}


//로봇위치를 0,0기준으로 삼아서 웨이포인트 위도경도 좌표를 xy좌표로 변환하는 코드
static void convert_station_waypoints_to_local_xy(void)
{
    for (int i = 0; i < station_count; i++) {
        latlon_to_local_xy(station_lat[i], station_lon[i], &station_x[i], &station_y[i]);
    }

    station_waypoint_pending = 0;
    station_waypoint_ready = 1;
}


void init_gnss(void)
{
    gnss_x = 0.0f;
    gnss_y = 0.0f;
    goal_x = 0.0f;
    goal_y = 0.0f;

    ref_latitude = 0.0f;
    ref_longitude = 0.0f;
    gnss_ref_initialized = 0;
    station_count = 0;
    station_waypoint_ready = 0;
    station_waypoint_pending = 0;
}


//웨이포인트 패킷 받으면 여기로 들어옴
//패킷안의 웨이포인트 좌표를 변수에 저장하고 패킷받았다는 플래그 1로 표시
void station_waypoint_packet_received(const float *packet, int packet_len)
{
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

    station_count = count;//웨이포인트 위도경도좌표 쌍 개수를 station_count에 저장

    for (int i = 0; i < count; i++) {
        station_lat[i] = packet[1 + (2 * i)];
        station_lon[i] = packet[1 + (2 * i) + 1];
    }

    if (gnss_ref_initialized) { 
    // update_gnss_position()이 한 번 호출되어
    // 로봇의 첫 RTK 위치가 ref_latitude/ref_longitude에 저장된 상태.
    // 즉 로봇 시작 위치가 local 좌표계의 (0,0)으로 설정됐다는 뜻.
    // 따라서 기지국에서 받은 웨이포인트 위도/경도를
    // 이 기준점 기준으로 모든 웨이포인트 각각을 x,y[m] 좌표로 변환할 수 있음.
    convert_station_waypoints_to_local_xy();
    } else {
    // 웨이포인트는 받았지만 아직 로봇 기준점이 없어서
    // x,y 변환을 나중으로 미룸.
        station_waypoint_pending = 1;
    }
}



int gnss_is_initialized(void)
{
    return gnss_ref_initialized;
}

//gnss rtk값 받는 함수 만들어서 그 안에서 호출해야됨->추가함
void update_gnss_position(float latitude, float longitude)//매개변수: gnss rtk모듈에서 받은 로봇의 위도, 경도임
{
    // 처음 들어온 위치를 기준점으로 사용하고 함수호출. 그다음부터 위도, 경도 들어오면 기준점 위도, 경도를 사용해서 이동 계산
    if (!gnss_ref_initialized) {
        ref_latitude = latitude;//로봇의 시작 기준점
        ref_longitude = longitude;//로봇의 시작 기준점
        gnss_ref_initialized = 1;//지금 이후로 쭉 gnss_ref_initialized는 1임.
        gnss_x = 0.0f;
        gnss_y = 0.0f;
        if (station_waypoint_pending) {
            convert_station_waypoints_to_local_xy();
        }
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


//웨이포인트 패킷 받으면 여기로 들어옴
void gnss_receive_complete(const float *packet, int packet_len)
{
    if (packet == 0 || packet_len <= 0) {
        return;
    }

    station_waypoint_packet_received(packet, packet_len);
}