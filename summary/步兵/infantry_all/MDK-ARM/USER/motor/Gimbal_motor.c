#include "Gimbal_motor.h"
#include "can_drv.h"
#include "motor.h"



/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_info_t 	GIMB_MOTOR_IF[GIMB_MOTOR_CNT] = {
	{
		.offline_max_cnt = 50,
		.MID = MEC_MID_Y_F,
	},
	{
		.offline_max_cnt = 50,
	},
};


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
drv_can_t		GIMB_MOTOR_DRV[] = {
	[GIMB_Y] = {
		.type    = DRV_CAN1,
		.can_id  = CAN_ID_GIMB_Y,
		.tx_data = CAN_SendSingleData,
	},
	[GIMB_P] = {
		.type    = DRV_CAN1,
		.can_id  = CAN_ID_GIMB_P,
		.tx_data = CAN_SendSingleData,
	},
};


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_pid_t 	GIMB_MOTOR_PID[GIMB_MOTOR_CNT] = {
	[GIMB_Y] = {	
		
#if CARR_MODE == 0
		/*-内环陀螺仪差分速度-*/
		.Mec_Out_Pid.kp = 2.f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 1000.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 200.f,
		.Mec_Inn_Pid.ki = 2.5f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 10000.f,
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪差分速度-*/
		.Imu_Out_Pid.kp = YAW_IMU_KP,//1.f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 1000.f,
		.Imu_Out_Pid.out_max = 12500.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 150.f,
		.Imu_Inn_Pid.ki = 2.5f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 10000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,

		.Turn_pid.kp = YAW_TURN_KP,
		.Turn_pid.ki = 0.f,
		.Turn_pid.kd = 0.f,
		.Turn_pid.integral_max = 0.f,
		.Turn_pid.out_max = 25000.f,
		.Turn_pid.blind_err = 10.f,//50
		
		.speed.kp = 0.6f,
		.speed.ki = 0.0005f,
		.speed.kd = 0.f,
		.speed.integral_max = 500.f,
		.speed.out_max = 5000.f,
		.speed.blind_err = 0.f,		
#endif

#if CARR_MODE == 1
		/*-内环陀螺仪差分速度-*/
		.Mec_Out_Pid.kp = 1.6f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 1000.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 200.f,
		.Mec_Inn_Pid.ki = 2.5f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 10000.f,
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪差分速度-*/
		.Imu_Out_Pid.kp = YAW_IMU_KP,//1.f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 1000.f,
		.Imu_Out_Pid.out_max = 12500.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 180.f,
		.Imu_Inn_Pid.ki = 2.5f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 10000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,

		.Turn_pid.kp = YAW_TURN_KP,
		.Turn_pid.ki = 0.f,
		.Turn_pid.kd = 0.f,
		.Turn_pid.integral_max = 0.f,
		.Turn_pid.out_max = 25000.f,
		.Turn_pid.blind_err = 10.f,
		
		.speed.kp = 0.6f,
		.speed.ki = 0.0005f,
		.speed.kd = 0.f,
		.speed.integral_max = 500.f,
		.speed.out_max = 5000.f,
		.speed.blind_err = 0.f,		
#endif


#if CARR_MODE == 2
		/*-内环陀螺仪差分速度-*/
		.Mec_Out_Pid.kp = 1.6f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 1000.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 200.f,
		.Mec_Inn_Pid.ki = 2.5f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 10000.f,
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪差分速度-*/
		.Imu_Out_Pid.kp = YAW_IMU_KP,//1.f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 1000.f,
		.Imu_Out_Pid.out_max = 12500.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 180.f,
		.Imu_Inn_Pid.ki = 2.5f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 10000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,

		.Turn_pid.kp = YAW_TURN_KP,
		.Turn_pid.ki = 0.f,
		.Turn_pid.kd = 0.f,
		.Turn_pid.integral_max = 0.f,
		.Turn_pid.out_max = 25000.f,
		.Turn_pid.blind_err = 10.f,
		
		.speed.kp = 0.6f,
		.speed.ki = 0.0005f,
		.speed.kd = 0.f,
		.speed.integral_max = 500.f,
		.speed.out_max = 5000.f,
		.speed.blind_err = 0.f,		
#endif
#if CARR_MODE == 3
		/*-内环陀螺仪差分速度-*/
		.Mec_Out_Pid.kp = 1.6f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 1000.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 200.f,
		.Mec_Inn_Pid.ki = 2.5f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 10000.f,
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪差分速度-*/
		.Imu_Out_Pid.kp = YAW_IMU_KP,//1.f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 1000.f,
		.Imu_Out_Pid.out_max = 12500.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 180.f,
		.Imu_Inn_Pid.ki = 2.5f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 10000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,

		.Turn_pid.kp = YAW_TURN_KP,
		.Turn_pid.ki = 0.f,
		.Turn_pid.kd = 0.f,
		.Turn_pid.integral_max = 0.f,
		.Turn_pid.out_max = 25000.f,
		.Turn_pid.blind_err = 10.f,
		
		.speed.kp = 0.6f,
		.speed.ki = 0.0005f,
		.speed.kd = 0.f,
		.speed.integral_max = 500.f,
		.speed.out_max = 5000.f,
		.speed.blind_err = 0.f,
#endif

#if CARR_MODE == 4
		/*-内环陀螺仪差分速度-*/
		.Mec_Out_Pid.kp = 1.6f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 1000.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 200.f,
		.Mec_Inn_Pid.ki = 2.5f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 10000.f,
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪差分速度-*/
		.Imu_Out_Pid.kp = YAW_IMU_KP,//1.f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 1000.f,
		.Imu_Out_Pid.out_max = 12500.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 180.f,
		.Imu_Inn_Pid.ki = 2.5f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 10000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,

		.Turn_pid.kp = YAW_TURN_KP,
		.Turn_pid.ki = 0.f,
		.Turn_pid.kd = 0.f,
		.Turn_pid.integral_max = 0.f,
		.Turn_pid.out_max = 25000.f,
		.Turn_pid.blind_err = 10.f,
		
		.speed.kp = 0.6f,
		.speed.ki = 0.0005f,
		.speed.kd = 0.f,
		.speed.integral_max = 500.f,
		.speed.out_max = 5000.f,
		.speed.blind_err = 0.f,		
#endif

	},
	[GIMB_P] = {	
		
#if CARR_MODE == 0
		/*-内环陀螺仪速度-*/
		.Mec_Out_Pid.kp = 10.f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 500.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 15.f,//50
		.Mec_Inn_Pid.ki = 1.55f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 20000.f,//8000
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		
		/*-内环陀螺仪速度-*/
		.Imu_Out_Pid.kp = PIT_IMU_KP,//2.2f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 500.f,
		.Imu_Out_Pid.out_max = 25000.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 15.f,
		.Imu_Inn_Pid.ki = 1.55f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 20000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,
		
#endif		

#if CARR_MODE == 1

		/*-内环陀螺仪速度-*/
		.Mec_Out_Pid.kp = 10.f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 500.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 15.f,//50
		.Mec_Inn_Pid.ki = 1.55f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 20000.f,//8000
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		
		/*-内环陀螺仪速度-*/
		.Imu_Out_Pid.kp = PIT_IMU_KP,//2.2f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 500.f,
		.Imu_Out_Pid.out_max = 25000.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 15.f,
		.Imu_Inn_Pid.ki = 1.55f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 20000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,
		
#endif			
		
#if CARR_MODE == 2
		/*-内环陀螺仪速度-*/
		.Mec_Out_Pid.kp = 10.f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 500.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 24.f,//50
		.Mec_Inn_Pid.ki = 1.55f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 20000.f,//8000
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪速度-*/
		.Imu_Out_Pid.kp = PIT_IMU_KP,//2.2f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 500.f,
		.Imu_Out_Pid.out_max = 25000.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 24.f,
		.Imu_Inn_Pid.ki = 1.55f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 25000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,
		
#endif	

#if CARR_MODE == 3
		/*-内环陀螺仪速度-*/
		.Mec_Out_Pid.kp = 10.f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 500.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 15.f,//50
		.Mec_Inn_Pid.ki = 1.55f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 20000.f,//8000
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪速度-*/
		.Imu_Out_Pid.kp = PIT_IMU_KP,//2.2f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 500.f,
		.Imu_Out_Pid.out_max = 25000.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 15.f,
		.Imu_Inn_Pid.ki = 1.55f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 25000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,
		
#endif	

#if CARR_MODE == 4
		/*-内环陀螺仪速度-*/
		.Mec_Out_Pid.kp = 7.f,
		.Mec_Out_Pid.ki = 0.f,
		.Mec_Out_Pid.kd = 0.f,
		.Mec_Out_Pid.integral_max = 500.f,
		.Mec_Out_Pid.out_max = 25000.f,
		.Mec_Out_Pid.blind_err = 0.f,
		
		.Mec_Inn_Pid.kp = 24.f,//50
		.Mec_Inn_Pid.ki = 1.55f,
		.Mec_Inn_Pid.kd = 0.f,
		.Mec_Inn_Pid.integral_max = 20000.f,//8000
		.Mec_Inn_Pid.out_max = 25000.f,
		.Mec_Inn_Pid.blind_err = 0.f,
		
		/*-内环陀螺仪速度-*/
		.Imu_Out_Pid.kp = PIT_IMU_KP,//2.2f,
		.Imu_Out_Pid.ki = 0.f,
		.Imu_Out_Pid.kd = 0.f,
		.Imu_Out_Pid.integral_max = 500.f,
		.Imu_Out_Pid.out_max = 25000.f,
		.Imu_Out_Pid.blind_err = 0.f,
		
		.Imu_Inn_Pid.kp = 24.f,
		.Imu_Inn_Pid.ki = 1.55f,
		.Imu_Inn_Pid.kd = 0.f,
		.Imu_Inn_Pid.integral_max = 25000.f,
		.Imu_Inn_Pid.out_max = 25000.f,
		.Imu_Inn_Pid.blind_err = 0.f,
		
#endif	

	},
};

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
motor_t		Gimb_Motor[] = {
	[GIMB_Y] = {
		.info       = &GIMB_MOTOR_IF[GIMB_Y],
		.driver     = &GIMB_MOTOR_DRV[GIMB_Y],
		.pid        = &GIMB_MOTOR_PID[GIMB_Y],
		.id         = DEV_ID_GIMB_Y,
		.type       = GIMBAL,
		.work_state = DEV_OFFLINE,
	},
	[GIMB_P] = {
		.info       = &GIMB_MOTOR_IF[GIMB_P],
		.driver     = &GIMB_MOTOR_DRV[GIMB_P],
		.pid        = &GIMB_MOTOR_PID[GIMB_P],
		.id         = DEV_ID_GIMB_P,
		.type       = GIMBAL,
		.work_state = DEV_OFFLINE,
	},
};

