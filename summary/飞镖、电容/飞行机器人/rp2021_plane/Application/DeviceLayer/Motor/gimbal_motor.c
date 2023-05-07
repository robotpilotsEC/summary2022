/**
 * @file        gimbal_motor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Gimbal Motor(GM6020).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "gimbal_motor.h"

#include "can_potocol.h"
#include "rp_math.h"

extern void gimbal_motor_update(gimbal_motor_t *motor, uint8_t *rxBuf);
extern void gimbal_motor_init(gimbal_motor_t *motor);

/* Private macro -------------------------------------------------------------*/
#define GIM_YAW_OFFLINE_MAX_CNT 10
#define GIM_PIT_OFFLINE_MAX_CNT 10

/* Private function prototypes -----------------------------------------------*/
static void gimbal_motor_check(gimbal_motor_t *motor);
static void gimbal_motor_heart_beat(gimbal_motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
// 云台电机驱动
drv_can_t		gimbal_motor_driver[GIMBAL_MOTOR_CNT] = {
	[YAW] = {
		.type = DRV_CAN1,
		.can_id = GIMBAL_CAN_ID_YAW,
//		.std_id = GM6020_GetStdId,
//		.drv_id = GM6020_GetDrvId,
		.add_tx_msg = CAN_AddTxMessage,
        .start_tx = CAN_StartTx
	},
	[PIT] = {
		.type = DRV_CAN1,
		.can_id = GIMBAL_CAN_ID_PIT,
//		.std_id = GM6020_GetStdId,
//		.drv_id = GM6020_GetDrvId,
		.add_tx_msg = CAN_AddTxMessage,
        .start_tx = CAN_StartTx
	},
};

// 云台电机信息
gimbal_motor_info_t 	gimbal_motor_info[GIMBAL_MOTOR_CNT] = {
	[YAW] = {
		.offline_max_cnt = GIM_YAW_OFFLINE_MAX_CNT,
	},
	[PIT] = {
		.offline_max_cnt = GIM_PIT_OFFLINE_MAX_CNT,
	},
};

// 云台电机传感器
gimbal_motor_t		gimbal_motor[GIMBAL_MOTOR_CNT] = {
	[YAW] = {
		.info = &gimbal_motor_info[YAW],
		.driver = &gimbal_motor_driver[YAW],
		.init = gimbal_motor_init,
		.update = gimbal_motor_update,
		.check = gimbal_motor_check,
		.heart_beat = gimbal_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_YAW,
	},
	[PIT] = {
		.info = &gimbal_motor_info[PIT],
		.driver = &gimbal_motor_driver[PIT],
		.init = gimbal_motor_init,
		.update = gimbal_motor_update,
		.check = gimbal_motor_check,
		.heart_beat = gimbal_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_PIT,		
	},
};

/* Private functions ---------------------------------------------------------*/
static void gimbal_motor_check(gimbal_motor_t *motor)
{
	int16_t err;
	gimbal_motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8191 + err;
		/* 8191↑ -> 0 */
		else
			motor_info->angle_sum += 8191 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->angle_sum += err;
	}
	
	motor_info->angle_prev = motor_info->angle;		
}

uint8_t js_yaw_offline_cnt, js_pit_offline_cnt;
static void gimbal_motor_heart_beat(gimbal_motor_t *motor)
{
	gimbal_motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
    
    js_yaw_offline_cnt = gimbal_motor[YAW].info->offline_cnt;
    js_pit_offline_cnt = gimbal_motor[PIT].info->offline_cnt;
}

/* Exported functions --------------------------------------------------------*/

