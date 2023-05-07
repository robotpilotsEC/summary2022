/**
 * @file        gimbal_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        4-November-2020
 * @brief       Shoot Task.
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"

#include "gimbal.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	云台任务
 */
//extern osThreadId GimbalTaskHandle;
//uint32_t GimbalTaskStackRemain = 0;	//分配256，剩余117 - 20/11/24 
//uint32_t GimbalTaskPeriodTime = 0;
void StartGimbalTask(void const * argument)
{
    //uint32_t GimbalTaskPrevWakeTime;
    
	gimbal.init();
	for(;;)
	{
        //imu_sensor.update(&imu_sensor);
		if(sys.state == SYS_STATE_NORMAL)
		{
			gimbal.ctrl();
		}
		else
		{
			gimbal.self_protect();
		}
        //GimbalTaskStackRemain = uxTaskGetStackHighWaterMark(GimbalTaskHandle);
        
        //GimbalTaskPrevWakeTime = xTaskGetTickCount();
        osDelay(1);
        //GimbalTaskPeriodTime = xTaskGetTickCount() - GimbalTaskPrevWakeTime;
	}
}

void GimbalTask(void)
{
    static bool init = false;
    
    if( !init ) {
        gimbal.init();
        init = true;
    }
    
    if(sys.state == SYS_STATE_NORMAL)
    {
        gimbal.ctrl();
    }
    else
    {
        gimbal.self_protect();
    }
    
}
