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
 *	@brief	更新遥控器数据信息(非正常状态下重置遥控信息)
 */
static void rc_update_info(void)
{
	if(sys.state != SYS_STATE_NORMAL) {
			
	}
	else {
		
	}
}

/**
 *	@brief	根据遥控器切换控制方式
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
 *	@brief	根据遥控器切换系统行为
 */
static void system_mode_act_switch(void)
{
	
}

static void system_state_machine(void)
{
	// 控制方式切换(云台复位完成才允许切换)
//	if(flag.gimbal.reset_ok == true)      
//	{
		system_ctrl_mode_switch();		
//	}
	// 系统模式切换(键盘模式下才允许切换)
	if(sys.remote_mode == KEY)
		system_mode_act_switch();
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统决策任务
 */
void StartSystemTask(void const * argument)
{
	for(;;)
	{
		portENTER_CRITICAL();
		
		// 更新遥控信息
		rc_update_info();
		
		/* 遥控连接离线 */
		if(master_info.RxPacket.ctrl_mode.online == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
		} 		
		/* 遥控连接在线 */
		else if(master_info.RxPacket.ctrl_mode.online == DEV_ONLINE)
		{		
			/* 遥控正常 */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* 失联恢复 */
				if(sys.state == SYS_STATE_RCLOST) 
				{
					// 可在此处同步云台复位标志位					
					// 系统参数复位
					sys.remote_mode = RC;
					sys.state = SYS_STATE_NORMAL;
					sys.mode = SYS_MODE_NORMAL;
					flag.gimbal.reset_start = true;
					flag.gimbal.reset_ok = false;					
				}
				
				// 可在此处等待云台复位后才允许切换状态
				system_state_machine();
			}
			/* 遥控错误 */
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
		//喂狗
		HAL_IWDG_Refresh(&hiwdg);

		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
