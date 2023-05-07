#include "Imu_task.h"
#include "cmsis_os.h"
#include "main.h"



void Start_imu_task(void const * argument)
{
	
	for(;;)
	{
		Imu_Data();
		osDelay(1);
	}
	
}


