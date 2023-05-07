#include "Auto.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "tim.h"
/*

����
L�� 6.6m
H�� 1.4m
��1.8
�Ӿ�
ʶ������ok
̧ͷ����Ҫ�ˣ���ؼӾ���
Ԥ����

���
������
ƫ���Ƿ�����
ȷʵ����

����ֹ����
���ڲ�����ȥ���߸����ͺ�̫��
�������Ȳ�һ�¸���λ�����ܷ��õ�

��С��
���ȫ����õ������Ҳ�򲻵�
Ԥ�������й۲���ʾ����
��ؼ�����ƫ�ã��Ҳ������õ��������ָ����򲻵�
����Ԥ��㣬����Ԥ����Ը߶ȣ�

�ı��˵�λ���㣺�Ӿ����20�ȱ�Ϊ1.47
pit��ƫ��-50 ����0.0009
yaw��ƫִ1.5 ����0.0052

�����
�Ӿ�Ԥ����Щ���⣺���ĵط����ˣ���ĵط�����

���Ӿ��������ݻ����ʶ��Ŀ�굫���͸���ص�p��y����Ϊ0

�����ͻ���������δʶ��Ŀ��ʱ������I����� I��������add ��������


����С���Կ�������������ʱ��������Ƭ�����³�ʼ��ʧ�ܣ������ǳ�ʼ��ʧ�ܣ�����2��ʼ��ʧ��
����������ǳ�ʼ��������ȼ�������1��ʼ��������ȼ�

�����ʹ�����ز����ʹ�ýǶȲ�

��KI kp��Щû�ã�ת����λҪ������

��ʹ�þ��ԽǶ�

��ʹ���Զ����

�������Զ���Ч������

*/



extern float Kp;  
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

#define VISION_TX_BUF_LEN LEN_VISION_TX_PACKET
uint8_t vision_dma_txbuf[VISION_TX_BUF_LEN];

Vision_Cmd_Id_t AUTO_CMD_MODE = CMD_AIM_OFF;
Vision_Cmd_Id_t Cmd_Last;

void TIM4_VISION_INIT(uint16_t ms);

char Get_Color(void);
uint8_t Get_Speed_Limit(void);



void Vision_Get(void)
{
	vision_sensor.info->TxPacket.TxData.yaw        = YAW;
	vision_sensor.info->TxPacket.TxData.pitch      = PITCH;
	vision_sensor.info->TxPacket.TxData.fric_speed = Get_Speed_Limit();//SHOOT_SPEED_LIMIT;
	vision_sensor.info->TxPacket.TxData.my_color   = Get_Color();//0��ɫ 1��ɫ 
	
	vision_dma_txbuf[sof] = VISION_FRAME_HEADER;
	
	vision_dma_txbuf[Cmd_ID] = AUTO_CMD_MODE;	
	
}



void Vision_TX(void)
{

	Cmd_Last = vision_sensor.info->TxPacket.FrameHeader.cmd_id;
	
	vision_sensor.info->TxPacket.FrameHeader.sof    = vision_dma_txbuf[sof];
	vision_sensor.info->TxPacket.FrameHeader.cmd_id = (Vision_Cmd_Id_t)vision_dma_txbuf[Cmd_ID];
	vision_sensor.info->TxPacket.FrameHeader.crc8   = vision_dma_txbuf[Crc8];
	
	Append_CRC8_Check_Sum(vision_dma_txbuf, LEN_FRAME_HEADER);
	
	memcpy(&vision_dma_txbuf[Data], &vision_sensor.info->TxPacket.TxData, LEN_TX_DATA);
	
	Append_CRC16_Check_Sum(vision_dma_txbuf, LEN_VISION_TX_PACKET);
	
	if(!USART_TEST){
		
		if((CARR_MODE == 0 && POSITION == 1)){
		
			HAL_UART_Transmit_DMA(&huart5, vision_dma_txbuf, LEN_VISION_TX_PACKET);
			
		}
		
		if((CARR_MODE == 4 && POSITION == 1)){
		
			HAL_UART_Transmit_DMA(&huart3, vision_dma_txbuf, LEN_VISION_TX_PACKET);
			
		}		
		
		else{
			
			HAL_UART_Transmit_DMA(&huart1, vision_dma_txbuf, LEN_VISION_TX_PACKET);
			
		}
  }
}



/*�����Զ���״̬*/
state_t DF_STATE(void)
{
	Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
	
	if((Pack->FrameHeader.cmd_id == CMD_AIM_SMALL_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_BIG_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_ANDF))
		return ING;
	else
		return NO;
}

state_t ZM_STATE(void)
{
	Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
	
	if((Pack->FrameHeader.cmd_id == CMD_AIM_AUTO || Pack->FrameHeader.cmd_id == CMD_AIM_ANTOP))
		return ING;
	else
		return NO;
}

auto_ing_t AUTO_ING(void)
{
	if(State.AUTO_STATE == ING)
	{
		if	   (DF_STATE() == ING)return DF;
		else if(ZM_STATE() == ING)return ZM;
		else                      return NO_AUTO;
	}
	else 
		return NO_AUTO;
}

/*�Ƿ��ҵ�Ŀ��*/
bool Find_Tar(void)
{
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;	
	
	if(Pack->is_find_Dafu || Pack->is_find_Target)
		return 1;
	else
		return 0;
}

/*�Ƿ񵽴�λ��*/
bool ARRIVE(void)
{
  if(State.AUTO_STATE == ING
		&& Find_Tar()
		&& abs(Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.err) < 20
  	&& abs(Gimb_Motor[GIMB_P].pid->Imu_Out_Pid.err) < 20)
		return 1;
	else
		return 0;
}



/*
 *�Ӿ����Բ���
 */

extern int16_t	Gimb_Current[4];
extern int16_t Gimb_Stop_Cmd[4];

void VISION_TEST(void)
{

	/*-�Ƚ��г�ʼ��-*/
	if(State.Gimbal_init == NO){
		
		GIMBAL_INIT_CTRL();
	
	}
	else{
		
		if(State.Func.state.AUTO_SHOOT != OK){
			
			State.func_mode(&State.Func, AUTO_SHOOT);
			
		}
		else{
			
			GIMBAL_AUTO_Ctrl();
			
		}
	}

	//�������
	if(TEST_MODE == 2 || State.VISION_TEST){
		
		switch(SW1){
			
			case 1:
				AUTO_CMD_MODE = CMD_AIM_AUTO;
				break;
			case 3:
				AUTO_CMD_MODE = CMD_AIM_SMALL_BUFF;
				break;
			case 2:
				AUTO_CMD_MODE = CMD_AIM_BIG_BUFF;
				break;				
		}
	}

	
	if(SW2_MID)State.SW2_LOCK = 1;
	
	if(State.SW2_LOCK == 1)
	{
			if(SW2_UP)State.func_mode(&State.Func, SHOT_ONCE);
			if(SW2_MID)State.func_mode(&State.Func, FUN_RESET);
			if(SW2_DOWN)State.func_mode(&State.Func, FRICTION);		
	}
	
	if(RC_ONLINE)
		Send_Current(GIMBA_CAN, GIMBA_ID, Gimb_Current);
	else
		Send_Current(GIMBA_CAN, GIMBA_ID, Gimb_Stop_Cmd);
	
}


/*--------------------------------------------------------*/

extern TIM_HandleTypeDef htim4;
//�Զ�ģʽ��ʼ��
void SHOT_AUTO_INIT(void)
{
	if(TEST_MODE == 2)
		GIMBAL_INIT();
	else
	  GIMBAL_INIT_NOW();
	
	TIM4_VISION_INIT(600);
		
	State.AUTO_SHOT_READ = 1;
	
	State.AUTO_DF_FLG = 0;
}


bool SHOT_AUTO_JUDGE(void)
{
	bool res = 0;
	
  if((MOUSE_LEFT || KEY_R) && State.AUTO_STATE == DF)State.AUTO_SHOT_READ = 0;
  
	else{
		State.AUTO_SHOT_READ = 1;
		State.AUTO_DF_FLG = 0;
	}

//	if(AUTO_CMD_MODE == CMD_AIM_ANTOP)State.AUTO_SHOT_READ = 1;
		
	if(RC_ONLINE && VISION_ONLINE
	&& State.AUTO_STATE == ING
	&& State.AUTO_SHOT_READ)
	{
		if(Find_Tar()){
			
			res = 1;
			
		}
		else{
		
			res = 0;
			
		}
		
	}
	else{
		
		res = 0;
		
	}
	
	return res;
	
}

void DF_AUTO_SHOOT(void)
{
	
	if(State.AUTO_DF_FLG && MOUSE_RIGH){
		
		if(State.Func.state.SHOT_ONCE == NO
		&& State.Func.state.SHOT_STUCK == NO){
			
			State.func_mode(&State.Func, SHOT_ONCE);
			
			State.AUTO_DF_FLG = 0;
			
		}
		
	}

}

bool ZM_AUTO_SHOOT(void)
{
	char res = 0;
	
	Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;	
	
	if(MOUSE_LEFT && Pack->RxData.zm_shoot_enable){
		
		if(State.Func.state.SHOT_ONCE == NO)State.func_mode(&State.Func, SHOT_STAY);	
		
		res = 1;			
	}
	
	return res;
}


extern vision_sensor_t	vision_sensor;
bool AUTO_SHOOT_CTRL(void)
{
	bool res = 0;

	if(SHOT_AUTO_JUDGE()){	
		
		if(State.AUTO_ING == DF){
			
			res = State.AUTO_DF_FLG;
		}
		else if(State.AUTO_ING == ZM){
		
      res = ZM_AUTO_SHOOT();
		}
	}

	return res;
	
}


void Vision_IRQ(void)
{
	
	if(State.AUTO_SHOT_READ && State.AUTO_ING == DF)State.AUTO_DF_FLG = 1;
	
	if(State.AUTO_ING == DF)DF_AUTO_SHOOT();
	
}



/*--------------------------------------------------------*/
//345 red 0   103 104 105 blue 1

char id_record = 3;
char Get_Color(void)
{
	char res = 3;	
		
  char id = judge_info.game_robot_status.robot_id;
	
	if(JUDGE_OFFLINE)
	{
		if(State.Enemy == RED)res = 1;
		
		else if(State.Enemy == BLUE)res = 0;
		
		else {
			
			if(id_record != 3)res = id_record;
			
		}
		
	}
	
	else{
		
		if(id == 3 || id == 4 || id == 5)res = 0;//hong
		
		else if(id == 103 || id == 104 || id == 105)res = 1;//lan  
		
		if(res == 1 || res == 0)
		{
			if(id_record == 3)id_record = res;
			if(id_record != res)id_record = res;
		}
		else
		{
			if(id_record != 3)res = id_record;
			else
			{
				if(State.Enemy == RED)res = 1;
				else if(State.Enemy == BLUE)res = 0;				
			}
		}
	}
	return res;
}


uint8_t Get_Speed_Limit(void)
{
	uint8_t limit;
	
	if(JUDGE_ONLINE)
	{
		limit = SHOOT_SPEED_LIMIT;
		
	}
	else
	{
		switch(NOW_SPEED_TARGET)
		{
			case Fri_15:
			limit = 15;	
			break;
		
			case Fri_18:
			limit = 18;	
			break;

			case Fri_20:
			limit = 20;	
			break;

			case Fri_22:
			limit = 22;	
			break;		

			case Fri_30:
			limit = 30;	
			break;				
		}
	}
	return limit;
}






















//6000 600ms
void TIM4_VISION_INIT(uint16_t ms)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = ms*10 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_TIM_Base_Start_IT(&htim4);
	
	htim4.Instance->CNT = 0;

}


//2.5ms�ж�



