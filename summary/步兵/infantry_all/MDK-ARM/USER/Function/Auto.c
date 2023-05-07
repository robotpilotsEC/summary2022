#include "Auto.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "tim.h"
/*

环境
L： 6.6m
H： 1.4m
·1.8
视觉
识别正常ok
抬头量不要了，电控加就行
预测量

电控
积分项
偏置是否有用
确实有用

·静止测试
存在不跟过去或者跟随滞后太大？
接下来先测一下各个位置上能否打得到

·小符
左侧全部打得到，都右侧打不到
预测点电脑中观测显示正常
电控加入左偏置，右侧个别点打得到，左侧出现个别点打不到
更改预测点，提升预测相对高度，

改变了单位换算：视觉电控20比变为1.47
pit：偏置-50 积分0.0009
yaw：偏执1.5 积分0.0052

·大符
视觉预测有些问题：慢的地方快了，快的地方慢了

·视觉传输数据会出现识别到目标但发送给电控的p与y数据为0

·解决突发恶疾情况：未识别到目标时不进行I项计算 I项计算带有add 会计算错误


·在小电脑开启并发送数据时，开启单片机导致初始化失败：陀螺仪初始化失败，串口2初始化失败
解决：陀螺仪初始化最高优先级，串口1初始化最低优先级

·打符使用像素差，自瞄使用角度差

·KI kp这些没用，转换单位要对首先

·使用绝对角度

·使用自动打符

·自瞄自动打弹效果不好

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
	vision_sensor.info->TxPacket.TxData.my_color   = Get_Color();//0红色 1蓝色 
	
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



/*处于自动的状态*/
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

/*是否找到目标*/
bool Find_Tar(void)
{
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;	
	
	if(Pack->is_find_Dafu || Pack->is_find_Target)
		return 1;
	else
		return 0;
}

/*是否到达位置*/
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
 *视觉调试测试
 */

extern int16_t	Gimb_Current[4];
extern int16_t Gimb_Stop_Cmd[4];

void VISION_TEST(void)
{

	/*-先进行初始化-*/
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

	//自瞄测试
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
//自动模式初始化
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


//2.5ms中断



