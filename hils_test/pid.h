#pragma once

typedef struct {
    float kp, ki, kd;
    float err1, err2, err3, err4;
    float err_sum;
    float v_limit; // 출력 제한만 관리
    float meas1, meas2, meas3, meas4;
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd, float v_limit);
float pid_calculate(PIDController *pid, float setpoint, float measurement, float dt);