#pragma once

// imu WT901 I2C (7-bit) default address = 0x50
#define WT901_I2C_ADDR   0x50

// imu WT901 Register: Pitch, Yaw, Gyro 데이터 주소
#define WT901_REG_PITCH  0x3E // Pitch (Y axis Angle) = 0x3E (2 bytes, L then H)
#define WT901_REG_YAW    0x3F
#define WT901_REG_GYRO   0x39
#define WT901_REG_ROLL   0x3d

// imu 200Hz 출력을 위한 레지스터 주소와 데이터 값
#define WT901_REG_RRATE   0x03
#define WT901_RRATE_200HZ 0x0B

// imu I2C 핀 설정
#define I2C_MASTER_SDA_IO  5
#define I2C_MASTER_SCL_IO  4

// imu타이머 설정 (200Hz = 5000us)
#define IMU_TIMER_RES_HZ   1000000 // 1초에 타이머가 몇 번 올라갈 것인가 (1us 단위)
#define IMU_ALARM_COUNT    5000    // 5ms = 200Hz 마다 알람 발생

#define BATTERY_VOLTAGE 20.0f  // 배터리 전압 20V
#define PWM_FREQ_HZ     20000 // 20kHz
#define PWM_PERIOD      1000  // 해상도

extern float current_pitch;
extern float current_yaw;
extern float current_roll;
extern float current_vel;

extern float targetvel_vel;
extern float target_yaw_diff;

extern float Vq_left;
extern float Vq_right;

extern int vel_calc_flag;

extern float roll_adj_mm;

#define WHEEL_RADIUS 0.042f//미터