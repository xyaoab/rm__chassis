#include "stm32f4xx.h"

extern float gimbal_yaw_target;
extern int16_t gimbal_pid_output;

void gimbal_setyaw( float setpoint);
void gimbal_yaw_pid(float target,float current);