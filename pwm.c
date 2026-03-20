#include <stdio.h>
#include "pwm.h"
#include "variable.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

mcpwm_cmpr_handle_t comparators[6];

void init_mcpwm_bldc() {
    mcpwm_timer_handle_t timer0 = NULL;
    mcpwm_timer_handle_t timer1 = NULL;

    mcpwm_timer_config_t timer_conf0 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_FREQ_HZ * PWM_PERIOD,//1초에 20M번 카운트함
        .period_ticks = PWM_PERIOD,//1000번 카운트하면 리셋. 즉 resolution_hz와 period_ticks에 의해 20khz가 완성됨
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    mcpwm_timer_config_t timer_conf1 = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_FREQ_HZ * PWM_PERIOD,//1초에 20M번 카운트함
        .period_ticks = PWM_PERIOD,//1000번 카운트하면 리셋. 즉 resolution_hz와 period_ticks에 의해 20khz가 완성됨
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    mcpwm_new_timer(&timer_conf0, &timer0);
    mcpwm_new_timer(&timer_conf1, &timer1);//MCPWM에 타이머 할당, timer핸들 생성

    int gpio_pins[6] = {7, 15, 16, 21, 47, 48};
    mcpwm_oper_handle_t operators[6];
    
    for (int i = 0; i < 6; i++) {
        int group = (i < 3) ? 0 : 1;
        mcpwm_operator_config_t oper_conf = { .group_id = group };
        mcpwm_new_operator(&oper_conf, &operators[i]);//operators[i]핸들 생성
        mcpwm_operator_connect_timer(operators[i], (group == 0) ? timer0 : timer1);//타이머설정을 오퍼레이터로 전달
        //오퍼레이터: 타이머신호를 받아 비교기와 제너레이터를 총괄 지휘하는 핵심엔진

        mcpwm_comparator_config_t cmpr_conf = { .flags.update_cmp_on_tez = true };
        //현재 주기가 끝날때까지 기다렸다가 타이머가 0이 되는 정박자에 기준선을 바꿔서 신호가 튀는사고를 막음
        mcpwm_new_comparator(operators[i], &cmpr_conf, &comparators[i]);

        mcpwm_gen_handle_t generator = NULL;
        mcpwm_generator_config_t gen_conf = { .gen_gpio_num = gpio_pins[i] };
        mcpwm_new_generator(operators[i], &gen_conf, &generator);
        //오퍼레이터가 쓸 출력도구를 만들어서 특정 gpio핀에 연결하는 작업

        mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        //업모드에서 타이머가 0이 되었을때 핀에 전기를 넣어라
        mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW));
        //업모드에서 비교값과 일치하면 핀의 전기를 끊어라
    }

    mcpwm_timer_enable(timer0);//타이머 전원을 켜고 준비상태 on
    mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP);//타이머가 실제로 숫자를 세기 시작하도록 만드는 명령

    mcpwm_timer_enable(timer1);
    mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP);
}

void mcpwm_set_voltage(int phase, float target_voltage) {
    if (target_voltage > 9.0f) target_voltage = 9.0f;   // +방향 최대 9V
    if (target_voltage < -9.0f) target_voltage = -9.0f; // -방향 최대 9V

    float duty_cycle = 0.5f + (target_voltage / BATTERY_VOLTAGE);
    //0.5더하는 이유: 배터리전압20v, 제어전압6v일때 0.5+6/20 = 0.8(80%)

    uint32_t compare_value = (uint32_t)(duty_cycle * PWM_PERIOD);

    mcpwm_comparator_set_compare_value(comparators[phase], compare_value);
}