#ifndef BASE_PID_H
#define BASE_PID_H

#include "stm32f4xx.h"

extern int16_t CM_target[4];
extern float base_Kp;
extern float base_Ki;
extern float base_Kd;
//extern float Iout;
typedef struct{
	
float temp_integral;
float temp_derivative;
float pre_error;

}ControlMotor;

int16_t target_accel(int16_t target_vel,int8_t number);
float base_pid_cal(s32 target_speed, s32 current_speed,ControlMotor* motor);
void rc_target_motor(void);
void key_target_motor(void);
extern float power_temp_integral;
extern float power_pid_output;
extern float mul;
void power_pid(float current_power, float target_power);

#endif
