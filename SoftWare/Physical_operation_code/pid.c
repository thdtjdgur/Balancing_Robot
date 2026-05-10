#include "pid.h"

void pid_init(PIDController *pid, float kp, float ki, float kd, float v_limit) {
    pid->kp = kp; 
    pid->ki = ki; 
    pid->kd = kd;
    pid->v_limit = v_limit;
    pid->err1 = pid->err2 = pid->err3 = pid->err4 = pid->err_sum = 0.0f;
}

//pid_calculate(&vel_ctrl, 0.0f, current_vel, 0.005f);
//pid_calculate(&pitch_ctrl, target_pitch, current_pitch, 0.005f);
//pid_calculate(&yaw_ctrl, 0.0f, current_yaw, 0.005f);
float pid_calculate(PIDController *pid, float setpoint, float measurement, float dt) {
    pid->err4 = pid->err3;
    pid->err3 = pid->err2;
    pid->err2 = pid->err1;
    pid->err1 = setpoint - measurement;
    //목표기울기가 10도이고 현재 기울기가 0도이면 pid->err1는 0임
    pid->err_sum += pid->err1;

    pid->meas4 = pid->meas3;
    pid->meas3 = pid->meas2;
    pid->meas2 = pid->meas1;
    pid->meas1 = measurement;
    
    float max_i_sum = 1000.0f; // 이 값은 튜닝하며 조절 가능
    if (pid->err_sum > max_i_sum) pid->err_sum = max_i_sum;
    else if (pid->err_sum < -max_i_sum) pid->err_sum = -max_i_sum;

    float p_term = pid->kp * pid->err1;
    float i_term = pid->ki * pid->err_sum;
    //float d_term = -pid->kd * ((pid->meas1 - pid->meas4) / dt + 2.0f * (pid->meas2 - pid->meas3) / dt);
    float d_term = pid->kd * ((pid->err1 - pid->err4) / dt + 2.0f * (pid->err2 - pid->err3) / dt);

    float output = p_term + i_term + d_term;

    // Saturation
    if (output > pid->v_limit) output = pid->v_limit;
    else if (output < -pid->v_limit) output = -pid->v_limit;

    return output;
}