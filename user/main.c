#include "main.h"
#include "function_list.h"

static u32 ticks_msimg = (u32)-1;
//////
CanRxMsg * msg;
s32 target_angle=0;
int32_t max=0;
float w=60;//power limitation
ControlMotor motor[4];

///////

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,WHITE,BLACK,BLACK);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	
}

int main(void)
{	
	init();
	//buzzer_play_song(START_UP, 125, 0);
	
	while (1)  {	

		if(ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			//buzzer_check();	
			
			
			
			
			
			
	
			yaw_axis_pid_cal(target_angle,output_angle);
			
			rc_target_motor();
			if(DBUS_ReceiveData.rc.switch_left==2)
			{ 
			
				Set_CM_Speed(CAN2,base_pid_cal(CM_target[0]*(1),CM1Encoder.filter_rate,&motor[0]),base_pid_cal(CM_target[1]*(1),CM2Encoder.filter_rate,&motor[1]),base_pid_cal(CM_target[2]*(1),CM3Encoder.filter_rate,&motor[2]),base_pid_cal(CM_target[3]*(1),CM4Encoder.filter_rate,&motor[3]));
			}
			else if(DBUS_ReceiveData.rc.switch_left==1)
			{ 
					Set_CM_Speed(CAN2,0,0,0,0);
					for(int i=0;i<4;i++)
						{
							CM_target[i]=0;
						}	
						target_angle=output_angle;
					
			}
			
			
			
			if(ticks_msimg%3==0)
			{
			if (DBUS_ReceiveData.rc.ch2>2||DBUS_ReceiveData.rc.ch2<-2)
			{
				target_angle=(target_angle + (DBUS_ReceiveData.rc.ch2*0.01 +(DBUS_CheckPush(KEY_W)-DBUS_CheckPush(KEY_Q))));
			} 
			else{target_angle=(target_angle +(DBUS_CheckPush(KEY_W)-DBUS_CheckPush(KEY_Q)));}
		  }
			
			
			if(ticks_msimg%20==0)
			{	
				w +=(-(InfantryJudge.RealVoltage*InfantryJudge.RealCurrent - 80)*0.02);  //buffer=60J
				if(w>=60)
				{
					w=60;
					mul=1;
				}
				else if( w>0 && w<60)
				{
					power_pid(w,60); 
					mul=1-power_pid_output; //range(0,60*0.007)				
					//if(mul>1){mul=1;}
					//else if(mul<0.4){mul=0.4;}//protection
				} 
				else//w<=0
				{
					w=0;
					mul=0.4;
				}
			}
			
			if(ticks_msimg%50==0)
			{	
		
				
				
				tft_clear();
				
				
			
				//tft_prints(1,2,"t_angle%d",target_angle);
				tft_prints(1,2," %d: %d",CM1Encoder.filter_rate,CM2Encoder.filter_rate);
				tft_prints(1,3,"%d: %d",CM4Encoder.filter_rate,CM3Encoder.filter_rate);
				tft_prints(1,4,"p%g i%g d%g",base_Kp,base_Ki,base_Kd); 
				tft_prints(1,5,"	yaw_pid:%f",yaw_pid_output_angle);
				tft_prints(1,6,"gyro%d",output_angle);

				tft_prints(1,7,"target%d %d",CM_target[0],CM_target[1]);
				tft_prints(1,8,"      %d %d",CM_target[3],CM_target[2]);
				//tft_prints(1,10,"t_angle%d",target_angle);
				tft_prints(1,9,"pp:%f ",power_pid_output);
				tft_prints(1,10,"W:%f ",w);
				tft_prints(1,11,"mul:%f ",mul);
			
				tft_update();
				LED_blink(LED1);
			}			
			
		}
	}	
}	

	



