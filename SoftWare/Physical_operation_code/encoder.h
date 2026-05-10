#pragma once

#include "freertos/FreeRTOS.h"  // 반드시 이게 먼저와야 함!
#include "driver/mcpwm_prelude.h" // MCPWM 필수 헤더
#include "driver/spi_master.h"
#include "driver/gptimer.h"

// 두 엔코더의 핸들을 하나로 묶어 인터럽트에 전달하기 위한 구조체
typedef struct {
    spi_device_handle_t left;
    spi_device_handle_t right;
} encoder_handles_t;

extern spi_device_handle_t h_left;  // 왼쪽 엔코더 핸들
extern spi_device_handle_t h_right; // 오른쪽 엔코더 핸들
extern spi_transaction_t *ret_t;

void encoder_timer_init(void);
void encoder_init(void);
void encoder_to_vcc_cal(void);
void spi_post_callback(spi_transaction_t *t);
