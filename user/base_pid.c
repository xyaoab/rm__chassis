#include "base_pid.h"
#include "Dbus.h"
#include "canBusProcess.h"
#include "yaw_gyro.h"
#include <cmath>

float base_Kp=35;//65//49//35//39
float base_Ki=5;//5//5//5
float base_Kd=47;//26//39//50//47
float current_accel=1.8; //1v/ms
//float Iout;

float base_pid_cal(s32 target_speed, s32 current_speed,ControlMotor *motor)
{
	float max=32000;
	float min=-32000;
	float error = target_speed - current_speed;
	
	float Kout = error * base_Kp; 
	
	motor->temp_integral += error;
	if(motor->temp_integral>(max/15)){
		motor->temp_integral=max/15;
	}
	else if(motor->temp_integral<-max/15)
	{
		motor->temp_integral=-max/15;
	}
	
	float Iout = motor->temp_integral * base_Ki;
	
	motor->temp_derivative = error - motor->pre_error;
	
	float Dout = motor->temp_derivative * base_Kd;
	
	float output_pid = Dout + Iout + Kout; 
	
	if(output_pid > max)
	{
		output_pid=max;
	}
	else if (output_pid < min)
	{
		output_pid=min;
	}

	return output_pid;
}

int16_t CM_target[4]={0,0,0,0};

int16_t current_vel[3]={0,0,0};


int16_t target_accel(int16_t target_vel,int8_t number)
{
	s8 dir=1;
	
	if(abs(target_vel-current_vel[number])> current_accel)
	{
		if(target_vel-current_vel[number]<0)
		{
			dir=-1;
		}
		current_vel[number]+=dir*current_accel;
	}
	else
	{
	current_vel[number]=target_vel;
	}
	return current_vel[number];
}


int16_t ch_target[3]={0,0,0};
float mul=1;
void rc_target_motor(void)
{	
	/*wheel_target[0]= DBUS_ReceiveData.rc.ch1 + DBUS_ReceiveData.rc.ch0 + yaw_pid_output_angle;
	wheel_target[1]= -DBUS_ReceiveData.rc.ch1 + DBUS_ReceiveData.rc.ch0 + yaw_pid_output_angle;
	wheel_target[2]= -DBUS_ReceiveData.rc.ch1 - DBUS_ReceiveData.rc.ch0 +yaw_pid_output_angle;
	wheel_target[3]= DBUS_ReceiveData.rc.ch1 - DBUS_ReceiveData.rc.ch0 + yaw_pid_output_angle;
	for (int i=0;i<4;i++){
	if(wheel_target[i]>1200){
	wheel_target[i]=1200;}
	else if(wheel_target[i]<-1200){
	wheel_target[i]=-1200;
	}
	*/
	ch_target[0]=target_accel((((float)DBUS_ReceiveData.rc.ch0)+660*(DBUS_CheckPush(KEY_F)-DBUS_CheckPush(KEY_S)))*1.48*mul,0);//F=right S=left
	ch_target[1]=target_accel((((float)DBUS_ReceiveData.rc.ch1)+660*(DBUS_CheckPush(KEY_E)-DBUS_CheckPush(KEY_D)))*1.48*mul,1);//E=forward D=backward 
	ch_target[2]=target_accel(yaw_pid_output_angle*1.48*mul,2);
	
	CM_target[0]= (ch_target[1] + ch_target[0]+ ch_target[2]);//*1.45*mul;
	
	CM_target[1]= (-ch_target[1] + ch_target[0]+ ch_target[2]);//*1.45*mul;
	
	CM_target[2]= (-ch_target[1]- ch_target[0] + ch_target[2]);//*1.45*mul;
	
	CM_target[3]= (ch_target[1] - ch_target[0]  + ch_target[2]);//*1.45*mul;
	
	for (int i=0;i<4;i++){
	if(CM_target[i]>1200){
	CM_target[i]=1200;}
	else if(CM_target[i]<-1200){
	CM_target[i]=-1200;
	}
}
}
float power_Kp=0.0076;//0.0066
float power_Ki=0.008;
float power_Kd=0;
float power_temp_integral;
float power_temp_derivative;
float power_pre_error;
float power_pid_output=0;
//buffer
void power_pid(float current_power, float target_power)
{
	float error = target_power - current_power;

	float Kout = error * power_Kp;
	
	power_temp_integral = power_temp_integral*0.7 + error;
	/*if (power_temp_integral<-1)
	{
		power_temp_integral=-1;
	}
	else if(power_temp_integral>1)
	{
		power_temp_integral=1;
	}
	*/
	float Iout = power_temp_integral * power_Ki;
	
	power_temp_derivative = error - power_pre_error;
	
	
	float Dout = power_temp_derivative * power_Kd;
	
	power_pid_output= Dout + Iout + Kout;
	if (power_pid_output<-1)
	{
		power_pid_output=-1;
	}
	else if(power_pid_output>1)
	{
		power_pid_output=1;
	}

}


