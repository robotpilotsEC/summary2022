/**
 * @file        system_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Decision Center.
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "rc_sensor.h"
#include "dial.h"
#include "motor.h"
#include "iwdg.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
flag_t flag = {
	.gimbal = {
		.reset_start = false,
		.reset_ok = false,
	},
};

system_t sys = {
	.remote_mode = RC,
	.state = SYS_STATE_RCLOST,
	.mode = SYS_MODE_NORMAL,
	.switch_auto = false,
	.switch_inspection = false,
	.switch_rc = false,
};

bool access_switch_auto = false;
bool access_switch_inspection = false;
bool access_switch_rc = false;

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	����ң����������Ϣ(������״̬������ң����Ϣ)
 */
static void rc_update_info(void)
{
	if(sys.state != SYS_STATE_NORMAL) {
			
	}
	else {
		
	}
}

/**
 *	@brief	����ң�����л����Ʒ�ʽ
 */
static void system_ctrl_mode_switch(void)
{
	switch(RC_SW2_VALUE)
	{
		case RC_SW_UP: 
		{
			sys.remote_mode = AUTO;
			
			if(access_switch_auto)
			{
				sys.switch_auto = true;     //�Զ�ģʽ�л���ʹ�ú���Ҫ����
				access_switch_auto = false;
			}
			access_switch_inspection = true;
			access_switch_rc = true;
		}break;
		case RC_SW_MID: 
		{
			sys.remote_mode = RC;
			
			if(access_switch_rc)
			{
				sys.switch_rc = true;     //�Զ�ģʽ�л���ʹ�ú���Ҫ����
				access_switch_rc = false;
			}
			access_switch_inspection = true;
			access_switch_auto = true;
		}break;
		case RC_SW_DOWN: 
		{
			sys.remote_mode = INSPECTION;
			
			if(access_switch_inspection)
			{
				sys.switch_inspection = true;     //�Զ�ģʽ�л���ʹ�ú���Ҫ����
				access_switch_inspection = false;
			}
			access_switch_rc = true;
			access_switch_auto = true;
		}break;
	}
		
	
		
}

/**
 *	@brief	����ң�����л�ϵͳ��Ϊ
 */
static void system_mode_act_switch(void)
{
	
}

static void system_state_machine(void)
{
	// ���Ʒ�ʽ�л�(��̨��λ��ɲ������л�)
//	if(flag.gimbal.reset_ok == true)      
//	{
		system_ctrl_mode_switch();		
//	}
	// ϵͳģʽ�л�(����ģʽ�²������л�)
	if(sys.remote_mode == KEY)
		system_mode_act_switch();
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	ϵͳ��������
 */
void StartSystemTask(void const * argument)
{
	for(;;)
	{
		portENTER_CRITICAL();
		
		// ����ң����Ϣ
		rc_update_info();
		
		/* ң������ */
		if(rc_sensor.work_state == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
		} 
		/* ң������ */
		else if(rc_sensor.work_state == DEV_ONLINE)
		{
			/* ң������ */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* ʧ���ָ� */
				if(sys.state == SYS_STATE_RCLOST) 
				{
					// ���ڴ˴�ͬ����̨��λ��־λ					
					// ϵͳ������λ
					sys.remote_mode = RC;
					sys.state = SYS_STATE_NORMAL;
					sys.mode = SYS_MODE_NORMAL;
					flag.gimbal.reset_start = true;
					flag.gimbal.reset_ok = false;					
				}
				
				// ���ڴ˴��ȴ���̨��λ��������л�״̬
				system_state_machine();
			}
			/* ң�ش��� */
			else if(rc_sensor.errno == DEV_DATA_ERR) {
				sys.state = SYS_STATE_RCERR;
				//reset CPU
				__set_FAULTMASK(1);
				NVIC_SystemReset();
			} else {
				sys.state = SYS_STATE_WRONG;
				//reset CPU
				__set_FAULTMASK(1);
				NVIC_SystemReset();
			}
		}
		//ι��
		HAL_IWDG_Refresh(&hiwdg);
		
		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
