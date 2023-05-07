#include "Chassis_task.h"
#include "cmsis_os.h"
#include "main.h"


/*
	底盘控制任务
	获取电容状态信息
*/
int16_t chas[4] = {0,0,0,0};
float current_tar = 0;
float sum= 0;
void Start_chassis_task(void const * argument)
{
	for(;;)
	{	
//		if(CHASSIS_MD)
//		CHASSIS_CTRL();
		Chas_Motor[POWER_0].pid->angle.target = current_tar;
		
		Chas_Motor[POWER_0].pid->out = 
		PID_CAL(&Chas_Motor[POWER_0].pid->angle,  NULL, 
						 Chas_Motor[POWER_0].info->angle_sum, NULL, 0);
		
		
		chas[0] = Chas_Motor[POWER_0].pid->out;
		
		
		sum = Chas_Motor[POWER_0].info->angle_sum;
		
		
		Send_Current(POWER_CAN, POWER_ID, chas);
		
		
		
		
		
		osDelay(2);
	}
}


/*
	电容控制任务
*/
uint16_t cishu;
void Start_Super_Task(void const * argument)
{	
	for(;;)
	{
	
		if(CAP_MD)
		{
			CAP_RP_2022.ctrl();
		}
		
		
		osDelay(4);
	}
}



