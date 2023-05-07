/* Includes ------------------------------------------------------------------*/
#include "fire_task.h"
#include "device.h"
#include "cmsis_os.h"
#include "dial.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/







/* Exported functions --------------------------------------------------------*/
void StartFireTask(void const * argument)
{
	shoot.init();
	for(;;)
	{
		if((sys.state == SYS_STATE_NORMAL) && (motor[DIAL].work_state == DEV_ONLINE) \
		&& (motor[FRICTION_L].work_state == DEV_ONLINE) && (motor[FRICTION_R].work_state == DEV_ONLINE))
		{
			shoot.ctrl();
//			shoot.self_protect();

		}
		else
		{
			shoot.self_protect();
		}
		
		osDelay(2);
	}
}

