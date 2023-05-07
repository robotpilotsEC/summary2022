#include "Chassis_motor.h"
#include "can_drv.h"
#include "motor.h"




/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_info_t 	CHAS_MOTOR_IF[] = {
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
	
	
	
	{
		.offline_max_cnt = 50,
		.MID = MEC_MID_TURN_0,
	},
	{
		.offline_max_cnt = 50,
		.MID = MEC_MID_TURN_1,
	},
	{
		.offline_max_cnt = 50,
		.MID = MEC_MID_TURN_2,
	},
	{
		.offline_max_cnt = 50,
		.MID = MEC_MID_TURN_3,
	},
};


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
drv_can_t		CHAS_MOTOR_DRV[] = {
	[POWER_0] = {
		.type    = DRV_CAN1,
		.can_id  = CAN_ID_POWER_0,
		.tx_data = CAN_SendSingleData,
	},
	[POWER_1] = {
		.type    = DRV_CAN1,
		.can_id  = CAN_ID_POWER_1,
		.tx_data = CAN_SendSingleData,
	},
	[POWER_2] = {
		.type    = DRV_CAN1,
		.can_id  = CAN_ID_POWER_2,
		.tx_data = CAN_SendSingleData,
	},
	[POWER_3] = {
		.type    = DRV_CAN1,
		.can_id  = CAN_ID_POWER_3,
		.tx_data = CAN_SendSingleData,
	},

/*-------------------------------------------------------------*/
	[TURN_0] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_TURN_0,
		.tx_data = CAN_SendSingleData,
	},
	[TURN_1] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_TURN_1,
		.tx_data = CAN_SendSingleData,
	},
	[TURN_2] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_TURN_2,
		.tx_data = CAN_SendSingleData,
	},
	[TURN_3] = {
		.type    = DRV_CAN2,
		.can_id  = CAN_ID_TURN_3,
		.tx_data = CAN_SendSingleData,
	},
};


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_pid_t 	CHAS_MOTOR_PID[CHAS_MOTOR_CNT] = {
	[POWER_0] = {	
		.speed.kp           = POWER_PID_P,
		.speed.ki           = POWER_PID_I,
		.speed.kd           = POWER_PID_D,
		.speed.integral_max = POWER_PID_I_MAX,
		.speed.out_max      = POWER_PID_O_MAX,
		.speed.blind_err    = POWER_PID_B_ERR,
	},
	[POWER_1] = {	
		.speed.kp           = POWER_PID_P,
		.speed.ki           = POWER_PID_I,
		.speed.kd           = POWER_PID_D,
		.speed.integral_max = POWER_PID_I_MAX,
		.speed.out_max      = POWER_PID_O_MAX,
		.speed.blind_err    = POWER_PID_B_ERR,
	},
	[POWER_2] = {	
		.speed.kp           = POWER_PID_P,
		.speed.ki           = POWER_PID_I,
		.speed.kd           = POWER_PID_D,
		.speed.integral_max = POWER_PID_I_MAX,
		.speed.out_max      = POWER_PID_O_MAX,
		.speed.blind_err    = POWER_PID_B_ERR,
	},
	[POWER_3] = {	
		.speed.kp           = POWER_PID_P,
		.speed.ki           = POWER_PID_I,
		.speed.kd           = POWER_PID_D,
		.speed.integral_max = POWER_PID_I_MAX,
		.speed.out_max      = POWER_PID_O_MAX,
		.speed.blind_err    = POWER_PID_B_ERR,
	},
/*-------------------------------------------------------------*/
	[TURN_0] = {	
		.Mec_Out_Pid.kp           = TURN_OUT_PID_P,
		.Mec_Out_Pid.ki           = TURN_OUT_PID_I,
		.Mec_Out_Pid.kd           = TURN_OUT_PID_D,
		.Mec_Out_Pid.integral_max = TURN_OUT_PID_I_MAX,
		.Mec_Out_Pid.out_max      = TURN_OUT_PID_O_MAX,
		.Mec_Out_Pid.blind_err    = TURN_OUT_PID_B_ERR,
		
		.Mec_Inn_Pid.kp           = TURN_IN_PID_P,
		.Mec_Inn_Pid.ki           = TURN_IN_PID_I,
		.Mec_Inn_Pid.kd           = TURN_IN_PID_D,
		.Mec_Inn_Pid.integral_max = TURN_IN_PID_I_MAX,
		.Mec_Inn_Pid.out_max      = TURN_IN_PID_O_MAX,
		.Mec_Inn_Pid.blind_err    = TURN_IN_PID_B_ERR,
	},
	[TURN_1] = {	
		.Mec_Out_Pid.kp           = TURN_OUT_PID_P,
		.Mec_Out_Pid.ki           = TURN_OUT_PID_I,
		.Mec_Out_Pid.kd           = TURN_OUT_PID_D,
		.Mec_Out_Pid.integral_max = TURN_OUT_PID_I_MAX,
		.Mec_Out_Pid.out_max      = TURN_OUT_PID_O_MAX,
		.Mec_Out_Pid.blind_err    = TURN_OUT_PID_B_ERR,
		
		.Mec_Inn_Pid.kp           = TURN_IN_PID_P,
		.Mec_Inn_Pid.ki           = TURN_IN_PID_I,
		.Mec_Inn_Pid.kd           = TURN_IN_PID_D,
		.Mec_Inn_Pid.integral_max = TURN_IN_PID_I_MAX,
		.Mec_Inn_Pid.out_max      = TURN_IN_PID_O_MAX,
		.Mec_Inn_Pid.blind_err    = TURN_IN_PID_B_ERR,
	},
	[TURN_2] = {	
		.Mec_Out_Pid.kp           = TURN_OUT_PID_P,
		.Mec_Out_Pid.ki           = TURN_OUT_PID_I,
		.Mec_Out_Pid.kd           = TURN_OUT_PID_D,
		.Mec_Out_Pid.integral_max = TURN_OUT_PID_I_MAX,
		.Mec_Out_Pid.out_max      = TURN_OUT_PID_O_MAX,
		.Mec_Out_Pid.blind_err    = TURN_OUT_PID_B_ERR,
		
		.Mec_Inn_Pid.kp           = TURN_IN_PID_P,
		.Mec_Inn_Pid.ki           = TURN_IN_PID_I,
		.Mec_Inn_Pid.kd           = TURN_IN_PID_D,
		.Mec_Inn_Pid.integral_max = TURN_IN_PID_I_MAX,
		.Mec_Inn_Pid.out_max      = TURN_IN_PID_O_MAX,
		.Mec_Inn_Pid.blind_err    = TURN_IN_PID_B_ERR,
	},
	[TURN_3] = {	
		.Mec_Out_Pid.kp           = TURN_OUT_PID_P,
		.Mec_Out_Pid.ki           = TURN_OUT_PID_I,
		.Mec_Out_Pid.kd           = TURN_OUT_PID_D,
		.Mec_Out_Pid.integral_max = TURN_OUT_PID_I_MAX,
		.Mec_Out_Pid.out_max      = TURN_OUT_PID_O_MAX,
		.Mec_Out_Pid.blind_err    = TURN_OUT_PID_B_ERR,
		
		.Mec_Inn_Pid.kp           = TURN_IN_PID_P,
		.Mec_Inn_Pid.ki           = TURN_IN_PID_I,
		.Mec_Inn_Pid.kd           = TURN_IN_PID_D,
		.Mec_Inn_Pid.integral_max = TURN_IN_PID_I_MAX,
		.Mec_Inn_Pid.out_max      = TURN_IN_PID_O_MAX,
		.Mec_Inn_Pid.blind_err    = TURN_IN_PID_B_ERR,
	},
};

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_t		Chas_Motor[] = {
	[POWER_0] = {
		.info       = &CHAS_MOTOR_IF[POWER_0],
		.driver     = &CHAS_MOTOR_DRV[POWER_0],
		.pid        = &CHAS_MOTOR_PID[POWER_0],
		.id         = DEV_ID_POWER_0,
		.type       = CHASSIS_POWER,
		.work_state = DEV_OFFLINE,
	},
	[POWER_1] = {
		.info       = &CHAS_MOTOR_IF[POWER_1],
		.driver     = &CHAS_MOTOR_DRV[POWER_1],
		.pid        = &CHAS_MOTOR_PID[POWER_1],
		.id         = DEV_ID_POWER_1,
		.type       = CHASSIS_POWER,
		.work_state = DEV_OFFLINE,
	},
	[POWER_2] = {
		.info       = &CHAS_MOTOR_IF[POWER_2],
		.driver     = &CHAS_MOTOR_DRV[POWER_2],
		.pid        = &CHAS_MOTOR_PID[POWER_2],
		.id         = DEV_ID_POWER_2,
		.type       = CHASSIS_POWER,
		.work_state = DEV_OFFLINE,
	},
	[POWER_3] = {
		.info       = &CHAS_MOTOR_IF[POWER_3],
		.driver     = &CHAS_MOTOR_DRV[POWER_3],
		.pid        = &CHAS_MOTOR_PID[POWER_3],
		.id         = DEV_ID_POWER_3,
		.type       = CHASSIS_POWER,
		.work_state = DEV_OFFLINE,
	},
	
/*-------------------------------------------------------------*/
	[TURN_0] = {
		.info       = &CHAS_MOTOR_IF[TURN_0],
		.driver     = &CHAS_MOTOR_DRV[TURN_0],
		.pid        = &CHAS_MOTOR_PID[TURN_0],
		.id         = DEV_ID_TURN_0,
		.type       = CHASSIS_TURN,
		.work_state = DEV_OFFLINE,
	},
	[TURN_1] = {
		.info       = &CHAS_MOTOR_IF[TURN_1],
		.driver     = &CHAS_MOTOR_DRV[TURN_1],
		.pid        = &CHAS_MOTOR_PID[TURN_1],
		.id         = DEV_ID_TURN_1,
		.type       = CHASSIS_TURN,
		.work_state = DEV_OFFLINE,
	},
	[TURN_2] = {
		.info       = &CHAS_MOTOR_IF[TURN_2],
		.driver     = &CHAS_MOTOR_DRV[TURN_2],
		.pid        = &CHAS_MOTOR_PID[TURN_2],
		.id         = DEV_ID_TURN_2,
		.type       = CHASSIS_TURN,
		.work_state = DEV_OFFLINE,
	},
	[TURN_3] = {
		.info       = &CHAS_MOTOR_IF[TURN_3],
		.driver     = &CHAS_MOTOR_DRV[TURN_3],
		.pid        = &CHAS_MOTOR_PID[TURN_3],
		.id         = DEV_ID_TURN_3,
		.type       = CHASSIS_TURN,
		.work_state = DEV_OFFLINE,
	},
};




