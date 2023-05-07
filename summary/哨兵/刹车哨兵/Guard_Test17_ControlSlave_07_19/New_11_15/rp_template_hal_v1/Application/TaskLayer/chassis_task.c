/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"
#include "chassis.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"



/* Exported functions --------------------------------------------------------*/
void StartChassisTask(void const * argument)
{
	chassis.init();
	for(;;)
	{
		if((sys.state == SYS_STATE_NORMAL) && (motor[CHASSIS].work_state == DEV_ONLINE)) 
		{
			chassis.ctrl();
//			chassis.self_protect();

		} 
		else 
		{
			chassis.self_protect();
		}
		osDelay(2);
	}

}

