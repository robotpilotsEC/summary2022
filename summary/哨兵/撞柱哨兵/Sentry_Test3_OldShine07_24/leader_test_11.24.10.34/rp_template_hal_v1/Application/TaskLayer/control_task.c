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
 *	@brief	������̨����
 */
static void Rc_Gimbal_Reset(void)
{
	rc_master_info_t *rc_master_info = rc_master_sensor.info;

	rc_master_info->RC_Master_RxPacket.rc_master_ch0 = 0; //�������̨ͨ��ֵ
	rc_master_info->RC_Master_RxPacket.rc_master_ch1 = 0;
	
}


/**
 *	@brief	��������
 */
static void Control_Normal()
{
	LED_RED_OFF();     //�����    
  LED_GREEN_ON();   //�̵���
	LASER_ON();

}

///**
// *	@brief	leader����
// */
//static void Control_Leader()
//{
//	
//}

/**
 *	@brief	ֹͣ���ƣ�ϵͳ�쳣��
 */
static void Control_Stop()
{
	Rc_Gimbal_Reset();  //���ͨ��ֵ
  LED_RED_ON();        //�����
  LED_GREEN_OFF();    //�̵���
//  LASER_OFF();
	/*---�����鼤�Ᵽ��һֱ����---*/
	LASER_ON();

}

/**
 *	@brief	��������
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
