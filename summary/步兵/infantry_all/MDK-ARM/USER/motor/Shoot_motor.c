#include "Shoot_motor.h"
#include "can_drv.h"
#include "motor.h"



/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_info_t 	SHOT_MOTOR_IF[] = {
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
};


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
drv_can_t		SHOT_MOTOR_DRV[] = {
	[FRIC_L] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_FRIC_L,
		.tx_data = CAN_SendSingleData,
	},
	[FRIC_R] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_FRIC_R,
		.tx_data = CAN_SendSingleData,
	},
	[BOX] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_BOX,
		.tx_data = CAN_SendSingleData,
	},	
	[BARREL] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_BARREL,
		.tx_data = CAN_SendSingleData,
	},	
};


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_pid_t 	SHOT_MOTOR_PID[SHOT_MOTOR_CNT] = {

	[FRIC_L] = {	
		.speed.kp = 8.f,
		.speed.ki = 0.5f,
		.speed.kd = 0.f,
		.speed.integral_max = 1000.f,
		.speed.out_max = 16500.f,
		.speed.blind_err = 0.f,
	},
	[FRIC_R] = {	
		.speed.kp = 8.f,//10
		.speed.ki = 0.5f,//0.8
		.speed.kd = 0.f,
		.speed.integral_max = 1000.f,
		.speed.out_max = 16500.f,
		.speed.blind_err = 0.f,
	},	
	
	[BOX] = {	
		.speed.kp = 3.f,
		.speed.ki = 0.5f,
		.speed.kd = 0.f,
		.speed.integral_max = 5000.f,
		.speed.out_max = 10000.f,
		.speed.blind_err = 0.f,
		
		.angle.kp = 0.28f,
		.angle.ki = 0.f,
		.angle.kd = 0.f,
		.angle.integral_max = 1000.f,
		.angle.out_max = 12500.f,
		.angle.blind_err = 0.f,		
	},
	
	[BARREL] = {	
		.speed.kp = 3.f,
		.speed.ki = 0.5f,
		.speed.kd = 0.f,
		.speed.integral_max = 5000.f,
		.speed.out_max = 10000.f,
		.speed.blind_err = 0.f,
		
		.angle.kp = 0.28f,
		.angle.ki = 0.f,
		.angle.kd = 0.f,
		.angle.integral_max = 1000.f,
		.angle.out_max = 12500.f,
		.angle.blind_err = 0.f,		
	},
};

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_t		Shot_Motor[] = {
	[FRIC_L] = {
		.info       = &SHOT_MOTOR_IF[FRIC_L],
		.driver     = &SHOT_MOTOR_DRV[FRIC_L],
		.pid        = &SHOT_MOTOR_PID[FRIC_L],
		.id         = DEV_ID_FRIC_L,
		.type       = SHOOT_FRIC,
		.work_state = DEV_OFFLINE,
	},
	[FRIC_R] = {
		.info       = &SHOT_MOTOR_IF[FRIC_R],
		.driver     = &SHOT_MOTOR_DRV[FRIC_R],
		.pid        = &SHOT_MOTOR_PID[FRIC_R],
		.id         = DEV_ID_FRIC_R,
		.type       = SHOOT_FRIC,
		.work_state = DEV_OFFLINE,
	},
	[BOX] = {
		.info       = &SHOT_MOTOR_IF[BOX],
		.driver     = &SHOT_MOTOR_DRV[BOX],
		.pid        = &SHOT_MOTOR_PID[BOX],
		.id         = DEV_ID_BOX,
		.type       = SHOOT_BOX,
		.work_state = DEV_OFFLINE,
	},
	[BARREL] = {
		.info       = &SHOT_MOTOR_IF[BARREL],
		.driver     = &SHOT_MOTOR_DRV[BARREL],
		.pid        = &SHOT_MOTOR_PID[BARREL],
		.id         = DEV_ID_BARREL,
		.type       = SHOOT_BARREL,
		.work_state = DEV_OFFLINE,
	},
};



