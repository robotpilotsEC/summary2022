/**
 * @file        shoot_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        4-November-2020
 * @brief       Shoot Task.
 */

/* Includes ------------------------------------------------------------------*/
#include "shoot_task.h"

#include "turnplate.h"
#include "fricwheel.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	射击任务
 */
 
uint8_t flg = 0;
void StartShootTask(void const * argument)
{
	fric.init();
	turnplate.init();
	for(;;)
	{

            if(sys.state == SYS_STATE_NORMAL)
            {
                fric.ctrl();
                turnplate.ctrl();
            }
            else
            {
                fric.self_protect();
                turnplate.self_protect();
            }
            
            osDelay(2); // 跟云台任务延时时间相同时会影响到云台的运动
	}
}
