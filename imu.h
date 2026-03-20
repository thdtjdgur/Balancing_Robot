#pragma once

#include "freertos/FreeRTOS.h"  // 반드시 이게 먼저와야 함!
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gptimer.h"

// 공유 세마포어 선언 (인터럽트와 태스크를 연결하는 핸들)
extern SemaphoreHandle_t imu_sem;
extern i2c_master_bus_handle_t bus_handle;

// 함수 프로토타입
i2c_master_dev_handle_t imu_init(void);
void imu_timer_init(void);
void imu_data_cal(i2c_master_dev_handle_t dev_handle);