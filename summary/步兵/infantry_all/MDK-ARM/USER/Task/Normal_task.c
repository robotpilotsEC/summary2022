#include "Normal_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "State.h"

/*
	状态观测器初始化
	
	状态中心运行
	失联监测

*/

extern judge_sensor_t judge_sensor;

 
void Start_normal_task(void const * argument)
{
	STATE_INIT();
	for(;;)
	{	
	
		
#if POSITION == 0		
		
		imu_offline_check();
		
		judge_sensor.heart_beat(&judge_sensor);

		motor_offline_check();
		
		Down_Send(&judge_sensor);
		
		CAP_RP_2022.heart_beat();
	
		slave_heart_beat();
		
		osDelay(1);
		
#endif
		
#if POSITION == 1		

	#if CARR_MODE == 0 || CARR_MODE == 2 || CARR_MODE == 3 || CARR_MODE == 4
	
		Up_Send();
		
	#endif
		
		if(STATE_MD)
		{
			STATE_CENTER();	
		
			OFFLINE_CHECK_TASK();		
		}			
		
		osDelay(1);
		
#endif
	}
}


