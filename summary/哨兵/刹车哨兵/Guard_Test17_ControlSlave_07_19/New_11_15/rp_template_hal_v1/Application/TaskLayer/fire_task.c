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
	dial.init();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL)
		{
			dial.ctrl();
//			dial.self_protect();  //上下发射机构卸力

		}
		else
		{
			dial.self_protect();
//			dial.ctrl();

		}
		
		osDelay(2);
	}
}

