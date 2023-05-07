/**
 * @file        control_task.c
 * @author      RobotPilots
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "driver.h"
#include "dial.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	上下云台重置
 */
static void Rc_Gimbal_Reset(void)
{
	rc_master_info_t *rc_master_info = rc_master_sensor.info;

	rc_master_info->RC_Master_RxPacket.rc_master_ch0 = 0; //清空上云台通道值
	rc_master_info->RC_Master_RxPacket.rc_master_ch1 = 0;
	
}


/**
 *	@brief	正常控制
 */
static void Control_Normal()
{
	LED_RED_OFF();     //红灯灭    
  LED_GREEN_ON();   //绿灯亮
	LASER_ON();

}

///**
// *	@brief	leader控制
// */
//static void Control_Leader()
//{
//	
//}

/**
 *	@brief	停止控制（系统异常）
 */
static void Control_Stop()
{
	Rc_Gimbal_Reset();  //清空通道值
  LED_RED_ON();        //红灯亮
  LED_GREEN_OFF();    //绿灯灭
//  LASER_OFF();
	/*---调自瞄激光保持一直开启---*/
	LASER_ON();

}

/**
 *	@brief	控制任务
 */
void StartControlTask(void const * argument)
{
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL) {
            Control_Normal();
		} else {
			Control_Stop();
		}	

		osDelay(2);
	}
}
