#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "State.h"

void Start_gimbal_Task(void const * argument)
{
	GIMBAL_INIT();
	
	for(;;)
	{	
		if(GIMBAL_MD)
    GIMBAL_CTRL();
		
		osDelay(2);
	}
}


