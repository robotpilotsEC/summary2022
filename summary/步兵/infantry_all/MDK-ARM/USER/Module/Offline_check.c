/*
*	   失联检测以及处理
*/


#include "Offline_check.h"




int16_t Stop_Cmd[4] = {0,0,0,0};

//记录睡眠时间
sleep_t RC_Sleep = 
{
	.time = 0,
};

void Sleep_Mode(void)
{
	Chassis_Motor_Stop();
	Shot_Stop();
}

//遥控器失联下开启的睡眠模式
//1.5秒内让所有电机停转 1.5秒后卸力
void RC_Offline_Sleep(void)
{
	if(RC_OFFLINE)
	{
		if(HAL_GetTick() - RC_Sleep.time < 1000)Sleep_Mode();
		
		else if(HAL_GetTick() - RC_Sleep.time < 4500)
		{
			CAN1_SendData(GM_F_ID, Stop_Cmd);
			CAN1_SendData(RM_F_ID, Stop_Cmd);
			
			CAN2_SendData(RM_F_ID, Stop_Cmd);			
			CAN2_SendData(GM_F_ID, Stop_Cmd);			
		}
			
	}
	else if(RC_ONLINE)
	{
		RC_Sleep.time = HAL_GetTick();
	}
}

//遥控失联复位，并将状态初始化
void rc_offline_handle(void)
{
	RC_ResetData(&rc);
}

void imu_offline_handle(void)
{
	
}

void motor_offline_handle(void)
{
	
}

//遥控失联时，不断发送电机数据0占用can,导致失联
void OFFLINE_CHECK_TASK(void)
{
	
	RC_Offline_Sleep();
	
	if(rc_offline_check())rc_offline_handle();
	
	imu_offline_check();
	
	motor_offline_check();
	
	CAP_RP_2022.heart_beat();
	
	judge_sensor.heart_beat(&judge_sensor);
	
	vision_sensor.heart_beat(&vision_sensor);
	
	slave_heart_beat();
	
}






