/**
 * @file        chassis_motor.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       Chassis Motor(RM3508).
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(24-October-2021)
 *                  1.将init与update函数定义在本文件中
 *              v1.1.1(8-November-2021)
 *                  1.更新驱动函数
 */
 
/* Includes ------------------------------------------------------------------*/
#include "motor.h"

#include "can_protocol.h"
#include "rm_protocol.h"
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void motor_init(motor_t *motor);
static void dial_motor_update(motor_t *motor, uint8_t *rxBuf);
static void gimbal_motor_update(motor_t *motor, uint8_t *rxBuf);
static void chassis_motor_update(motor_t *motor, uint8_t *rxBuf);
static void motor_check(motor_t *motor);
static void motor_heart_beat(motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 电机驱动
drv_can_t		motor_driver[] = {
	[DIAL] = {
		.id = DRV_CAN2,
		.rx_id = DIAL_CAN_ID,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	[GIMB_PITCH] = {
		.id = DRV_CAN2,
		.rx_id = GIMBAL_CAN_ID_PITCH,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	[GIMB_YAW] = {
		.id = DRV_CAN1,
		.rx_id = GIMBAL_CAN_ID_YAW,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	[CHASSIS] = {
		.id = DRV_CAN1,
		.rx_id = CHASSIS_CAN_ID,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	
};

// 底盘电机信息
motor_info_t 	motor_info[] = {
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
};

// 电机传感器
motor_t	motor[] = {
	[DIAL] = {
		.info = &motor_info[DIAL],
		.driver = &motor_driver[DIAL],
		.init = motor_init,
		.update = dial_motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_DIAL,
	},
	[GIMB_PITCH] = {
		.info = &motor_info[GIMB_PITCH],
		.driver = &motor_driver[GIMB_PITCH],
		.init = motor_init,
		.update = gimbal_motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_PITCH,
	},
	[GIMB_YAW] = {
		.info = &motor_info[GIMB_YAW],
		.driver = &motor_driver[GIMB_YAW],
		.init = motor_init,
		.update = gimbal_motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_YAW,
	},
	[CHASSIS] = {
		.info = &motor_info[CHASSIS],
		.driver = &motor_driver[CHASSIS],
		.init = motor_init,
		.update = chassis_motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS,
	},
	
};

/* Private functions ---------------------------------------------------------*/
static void motor_init(motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_DIAL) {
		drv_can->data_idx = RM2006_GetDataId(drv_can);
		drv_can->tx_id = RM2006_GetTxId(drv_can);
	}
	else if(motor->id == DEV_ID_GIMBAL_PITCH) {
		drv_can->data_idx = GM6020_GetDataId(drv_can);
		drv_can->tx_id = GM6020_GetTxId(drv_can);
	}
	else if(motor->id == DEV_ID_GIMBAL_YAW) {
		drv_can->data_idx = GM6020_GetDataId(drv_can);
		drv_can->tx_id = GM6020_GetTxId(drv_can);	
	}
	else if(motor->id == DEV_ID_CHASSIS) {
		drv_can->data_idx = RM3508_GetDataId(drv_can);
		drv_can->tx_id = RM3508_GetTxId(drv_can);		
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}

static void dial_motor_update(motor_t *dial_motor, uint8_t *rxBuf)
{
	motor_info_t *motor_info = dial_motor->info;
	
	motor_info->angle = RM2006_GetMotorAngle(rxBuf);
	motor_info->speed = RM2006_GetMotorSpeed(rxBuf);
	motor_info->torque = RM2006_GetMotorTorque(rxBuf);
   
	motor_info->offline_cnt = 0;
}


static void gimbal_motor_update(motor_t *gimbal_motor, uint8_t *rxBuf)
{
	motor_info_t *motor_info = gimbal_motor->info;
	
	motor_info->angle = GM6020_GetMotorAngle(rxBuf);
	motor_info->speed = GM6020_GetMotorSpeed(rxBuf);
	motor_info->current = GM6020_GetMotorCurrent(rxBuf);
	motor_info->temperature = GM6020_GetMotorTemperature(rxBuf);
	
	motor_info->offline_cnt = 0;
}

static void chassis_motor_update(motor_t *chassis_motor, uint8_t *rxBuf)
{
	motor_info_t *motor_info = chassis_motor->info;
	
	motor_info->angle = RM3508_GetMotorAngle(rxBuf);
	motor_info->speed = RM3508_GetMotorSpeed(rxBuf);
	motor_info->current = RM3508_GetMotorCurrent(rxBuf);
	motor_info->temperature = RM3508_GetMotorTemperature(rxBuf);
   
	motor_info->offline_cnt = 0;
}


static void motor_check(motor_t *motor)
{
	int16_t err;
	motor_info_t *motor_info = motor->info;
	
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

static void motor_heart_beat(motor_t *motor)
{
	motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/

