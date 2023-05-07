/**
 * @file        system_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Decision Center.
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"
#include "cmsis_os.h"
#include "rc_sensor.h"
#include "dial.h"
#include "motor.h"
#include "rp_math.h"
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
};

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
	if(master_info.RxPacket.ctrl_mode.mode == RC)
	{
		sys.remote_mode = RC;
	}		
	else if(master_info.RxPacket.ctrl_mode.mode == KEY)
	{
		sys.remote_mode = KEY;
	}
	else if(master_info.RxPacket.ctrl_mode.mode == AUTO)
	{
		sys.remote_mode = AUTO;
	}
	else if(master_info.RxPacket.ctrl_mode.mode == INSPECTION)
	{
		sys.remote_mode = INSPECTION;
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
		
		/* ң���������� */
		if(master_info.RxPacket.ctrl_mode.online == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
		} 		
		/* ң���������� */
		else if(master_info.RxPacket.ctrl_mode.online == DEV_ONLINE)
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
