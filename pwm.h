#pragma once

#include "freertos/FreeRTOS.h"  // 반드시 이게 먼저와야 함!
#include "driver/mcpwm_prelude.h" // MCPWM 필수 헤더
#include "driver/gptimer.h"

extern mcpwm_cmpr_handle_t comparators[6];

void init_mcpwm_bldc();
void mcpwm_set_voltage(int phase, float target_voltage);