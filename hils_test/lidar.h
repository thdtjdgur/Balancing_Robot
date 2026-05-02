#pragma once

#include <stdint.h>

// 0~359도까지의 장애물 거리를 저장하는 전역 배열 (단위: mm)
// 장애물이 없거나 측정 실패한 각도는 0.0f가 들어갑니다.
extern float distance_map[360];

// 라이다 초기화 함수 선언
void init_lidar(void);
void lidar_event_task(void *pvParameters);
void parse_lidar_packet(uint8_t *buf, int len);