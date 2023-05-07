/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"
#include "device.h"
#include "cmsis_os.h"
#include "gimbal.h"










/* Exported functions --------------------------------------------------------*/
void StartGimbalTask(void const * argument)
{
	gimbal.init();
	for(;;)
	{  		
		if((sys.state == SYS_STATE_NORMAL) && (motor[GIMB_PITCH].work_state == DEV_ONLINE) && (motor[GIMB_YAW].work_state == DEV_ONLINE)) 
		{
			gimbal.ctrl();
//			gimbal.self_protect();

		} 
		else 
		{
			gimbal.self_protect();
		}
		
		osDelay(2);
	}
}
