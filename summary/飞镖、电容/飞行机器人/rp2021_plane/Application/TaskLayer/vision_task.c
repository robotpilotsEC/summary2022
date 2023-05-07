/**
 * @file        vision_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        18-May-2021
 * @brief       Vision Center
 */

/* Includes ------------------------------------------------------------------*/
#include "vision_task.h"

#include "vision.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	数据消息处理任务
 */
extern osThreadId VisionTaskHandle;
void StartVisionTask(void const * argument)
{
    vision.init();
	for(;;)
	{
        if(sys.state == SYS_STATE_NORMAL)
		{
            //vision.test();
            vision.ctrl();
		}
		else
		{
			vision.self_protect();
		}
//        vision.test();
		osDelay(100);
	}
}
