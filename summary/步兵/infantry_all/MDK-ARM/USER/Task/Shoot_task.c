#include "Shoot_task.h"
#include "cmsis_os.h"
#include "main.h"


void Start_shoot_task(void const * argument)
{
	for(;;)
	{	
		if(SHOOT_MD)
    SHOOT_CTRL();
		
		osDelay(2);
	}
}



