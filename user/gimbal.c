#include "gimbal.h"
#include "stm32f4xx.h"
#include "Dbus.h"

float gimbal_Kp=85;
float gimbal_Ki=0.03;
float gimbal_Kd=0;
float gimbal_temp_integral;
float gimbal_temp_derivative;
float gimbal_pre_error;

float gimbal_yaw_target;


int16_t gimbal_pid_output;

void gimbal_setyaw( float setpoint)
{
	gimbal_yaw_target=setpoint/15;
	
}
void gimbal_yaw_pid(float target,float current)
{
	float error = target-current;
	float Kout = error*gimbal_Kp;
	
	gimbal_temp_integral += error;
	
	float Iout = gimbal_temp_integral * gimbal_Ki;
	
	gimbal_temp_derivative = error - gimbal_pre_error;
	float Dout = gimbal_temp_derivative * gimbal_Kd;
	
	gimbal_pid_output= Iout + Dout + Kout;
	if(gimbal_pid_output>4000)
	{
		gimbal_pid_output=4000;
	}
	if(gimbal_pid_output<-4000)
	{
		gimbal_pid_output=-4000;
	}


}
