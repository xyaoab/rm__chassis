
#include "stdio.h"
#include "canBusProcess.h"

static uint32_t can_count = 0;

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};//201
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};//202
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};//203
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};//204

volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};//205
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};//206
volatile Encoder GMxEncoder = {0,0,0,0,0,0,0,0,0};//207


/*
***********************************************************************************************
*Name          :GetEncoderBias
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{

            v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //�����ʼ������ֵ��Ϊƫ��  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
	
}

/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	v->velocity_from_ESC = (msg->Data[2]<<8)|msg->Data[3];
	//for detecting the speed with last sample
	if(v->diff < -7000)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7000)   //bug
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	
	
	//����õ������ı��������ֵ 
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

/*
************************************************************************************************************************
*Name        : CanReceiveMsgProcess
* Description: This function process the can message representing the encoder data received from the CAN2 bus.
* Arguments  : msg     is a pointer to the can message.
* Returns    : void
* Note(s)    : none
************************************************************************************************************************
*/

void CanReceiveMsgProcess(CanRxMsg * msg)
{      
        //GMYawEncoder.ecd_bias = yaw_ecd_bias;
        can_count++;
		switch(msg->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM1Encoder ,msg):EncoderProcess(&CM1Encoder ,msg);       //��ȡ���������ĳ�ʼƫ��ֵ            
                    
				}break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM2Encoder ,msg):EncoderProcess(&CM2Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM3Encoder ,msg):EncoderProcess(&CM3Encoder ,msg);   
				}break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM4Encoder ,msg):EncoderProcess(&CM4Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
				{
					 (can_count<=50) ? GetEncoderBias(&GMYawEncoder ,msg):EncoderProcess(&GMYawEncoder ,msg);    
					
				}break;
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{
						EncoderProcess(&GMPitchEncoder ,msg);
					
				}break;				
				
				case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
				{
						(can_count<=50) ? GetEncoderBias(&GMxEncoder ,msg):EncoderProcess(&GMxEncoder ,msg);
					
				}break;				
				
		}
 
}

/********************************************************************************
   �����̵���巢��ָ�ID��Ϊ0x200�������̷���IDΪ0x201-0x204
*********************************************************************************/
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
	
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
	
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
	
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
	
    CAN_Transmit(CANx,&tx_message);
}

/********************************************************************************
   ������巢��ָ�ID��Ϊ0x1FF��ֻ����������壬���ݻش�IDΪ0x205��0x206
	 cyq:����Ϊ�������������ָ�-5000~+5000
*********************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t gimbal_x_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (uint8_t)gimbal_yaw_iq;
    tx_message.Data[2] = (uint8_t)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (uint8_t)gimbal_pitch_iq;
    tx_message.Data[4] = (uint8_t)(gimbal_x_iq >> 8);//maybe can maybe cannot. i dont know// i only know it can fieed back by endcoder 
    tx_message.Data[5] = (uint8_t)gimbal_x_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}


