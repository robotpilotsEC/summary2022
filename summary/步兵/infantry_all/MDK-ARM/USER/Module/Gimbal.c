/*

	云台相关功能函数
	can1统一发送

*/

#include "Gimbal.h"
#include "State.h"
#include "Auto.h"
/*
	视觉偏置
*/
//打符部分
#define DF_YAW_ADD 0//20
#define DF_PIT_ADD 0//-30
#define DF_KP      (1024.f / 700.f)
//自瞄部分
#define AIM_YAW_ADD 0
#define AIM_PIT_ADD 0
#define AIM_KP      (360.f / 8191.f)


//打符模式下的偏置与积分项
float Yaw_add;
float Pit_add;
float VISION_KP;


extern float Kp;//陀螺仪bmi的kp值
int16_t	Gimb_Current[4] = {0,0,0,0};
int16_t Gimb_Stop_Cmd[4] = {0,0,0,0};

void Gimbal_Motor_Current(motor_t *gimbal,int16_t *data)
{	
	if(RC_OFFLINE)
		data[gimbal->driver->drv_id] = 0;
	else 
		data[gimbal->driver->drv_id] = gimbal->pid->out;
	
}


/*--------------------------------------*/
/*
 *P Y 输入部分
 */

float Gimb_Yaw_Input(void)
{
	float tar = 0;	
	
	if(State.Ctrl.state.RC == OK)	
		 tar = CH0_VALUE;
	if(State.Ctrl.state.KEY == OK)
	 	 tar = CH0_VALUE_K;
	
	if(tar == NULL)tar = 0;
	
	return tar;
}

float Gimb_Pit_Input(void)
{
	float tar = 0;
	
	if(State.Ctrl.state.RC == OK)
		 tar = CH1_VALUE;	
	if(State.Ctrl.state.KEY == OK)
		 tar = CH1_VALUE_K;
	
	if(State.Func.state.MAGAZINE == OK)tar = 0;
		
	if(tar == NULL)tar = 0;
	
	return tar;
}



 /*--------------------------------------------*/
/*
			P Y 输出部分
*/

/*陀螺仪模式*/

void Gimbal_Yaw_IMU_Output(motor_t *gimbal ,float tar)
{
	motor_pid_t *pid  = gimbal->pid;	
	
	if(tar && (!Q_FLAG && !E_FLAG && !C_FLAG))
	{		
		//目标值限制
		pid->Imu_Out_Pid.kp = YAW_IMU_KP;
		pid->Imu_Out_Pid.target = Limit_Target(YAW - tar);
	}

	pid->out = PID_CAL(&pid->Imu_Out_Pid,
	                   &pid->Imu_Inn_Pid,
	                   YAW, YAW_V, 1);
	
	Gimbal_Motor_Current(gimbal, Gimb_Current);	//输出
	
}

void Gimbal_Pitch_IMU_Output(motor_t *gimbal ,float tar)
{
	motor_pid_t *pid   = gimbal->pid;
	
	if(tar)
	{		//目标值限制
		pid->Imu_Out_Pid.target = Limit_Target(PITCH - tar);
	}
	
	pid->Imu_Out_Pid.target = 	
	constrain(pid->Imu_Out_Pid.target, IMU_HIG_P, IMU_LOW_P);//anti_constrain

	pid->out = PID_CAL(&pid->Imu_Out_Pid,
	                   &pid->Imu_Inn_Pid,
	                   PITCH, PITCH_V, 1);

	Gimbal_Motor_Current(gimbal, Gimb_Current);      //输出
}

/*机械模式*/
void Gimbal_Yaw_MEC_Output(motor_t *gimbal ,float tar)
{
	motor_pid_t  *pid  = gimbal->pid;	
	motor_info_t *info = gimbal->info;

	pid->out = PID_CAL(&pid->Mec_Out_Pid,
	          				 &pid->Mec_Inn_Pid,
	          				 info->angle, YAW_V, 1);
	
	Gimbal_Motor_Current(gimbal, Gimb_Current);	//输出
}



void Gimbal_Pitch_MEC_Output(motor_t *gimbal ,float tar)
{
	motor_pid_t  *pid  = gimbal->pid;
	motor_info_t *info = gimbal->info;
	
	if(tar)                                  //目标值限制
		pid->Mec_Out_Pid.target = Limit_Target(info->angle - tar);
	
	
#if	CARR_MODE == 0
	
	pid->Mec_Out_Pid.target = 
	constrain(pid->Mec_Out_Pid.target,\
						MEC_MID_P - IMU_MID_P + IMU_HIG_P,\
						MEC_MID_P + IMU_LOW_P - IMU_MID_P);
	
#endif
	
	
#if CARR_MODE == 1

	pid->Mec_Out_Pid.target = 
	constrain(pid->Mec_Out_Pid.target,6187,7316);
#endif

#if CARR_MODE == 2

	pid->Mec_Out_Pid.target = 
	constrain(pid->Mec_Out_Pid.target,6783,7744);
	
#endif

#if CARR_MODE == 3

	pid->Mec_Out_Pid.target = 
	constrain(pid->Mec_Out_Pid.target,5850,7248);
	
#endif

#if CARR_MODE == 4

	pid->Mec_Out_Pid.target = 
	constrain(pid->Mec_Out_Pid.target,5880,7280);
	
#endif


	pid->Mec_Out_Pid.target = 
	constrain(pid->Mec_Out_Pid.target,0,8191);	

	pid->out = PID_CAL(&pid->Mec_Out_Pid,
	          				 &pid->Mec_Inn_Pid,
	          				 info->angle, PITCH_V, 1);

	Gimbal_Motor_Current(gimbal , Gimb_Current);      //输出
}






 /*--------------------------------------*/
/*--------------------------------------*/


void YAW_IMU_CTRL(void)
{
	float tar = 0;
	
	tar = Gimb_Yaw_Input();
	Gimbal_Yaw_IMU_Output(&Gimb_Motor[GIMB_Y], tar);
}

void YAW_MEC_CTRL(void)
{
	float tar = 0;
	if(TEST_MODE == 3)tar = Gimb_Yaw_Input();
	Gimbal_Yaw_MEC_Output(&Gimb_Motor[GIMB_Y], tar);
}



void PIT_IMU_CTRL(void)
{
	float tar = 0;
	
	tar = Gimb_Pit_Input();
	Gimbal_Pitch_IMU_Output(&Gimb_Motor[GIMB_P], tar);
}

void PIT_MEC_CTRL(void)
{
	float tar = 0;
		
	tar = Gimb_Pit_Input();
	Gimbal_Pitch_MEC_Output(&Gimb_Motor[GIMB_P], tar);
}


 /*--------------------------------------*/
/*--------------------------------------*/


void GIMBAL_GIMB_FRI_CTRL(void)
{
	YAW_IMU_CTRL();
	PIT_IMU_CTRL();
}

void GIMBAL_CHAS_FRI_CTRL(void)
{
	YAW_MEC_CTRL();
	PIT_MEC_CTRL();
}













 /*--------------------------------------*/
/*--------------------------------------*/


/*******AUTO输入输出*******/

extern Vision_Cmd_Id_t AUTO_CMD_MODE;

float Gimb_Yaw_AUTO_Input(void)
{
	float tar = 0;	
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;
  
	/*--赋值--*/
//	tar = Pack->yaw_angle / VISION_KP;
	tar = Pack->yaw_angle;
	return tar;
}

float Gimb_Pit_AUTO_Input(void)
{
	float tar = 0;
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;	

	/*--赋值--*/
//	tar = Pack->pitch_angle / VISION_KP;
	tar = Pack->pitch_angle;
	return tar;
}


void Gimbal_Yaw_Auto_Output(motor_t *gimbal ,float tar)
{
	motor_pid_t *pid  = gimbal->pid;	
	
	/*--赋予目标值--*/
	pid->Imu_Out_Pid.target = tar + Yaw_add;

	/*--进行PID计算--*/
	pid->out = PID_CAL(&pid->Imu_Out_Pid,
	                   &pid->Imu_Inn_Pid,
	                   YAW, YAW_V, 1);
  /*--赋予电流--*/
	Gimbal_Motor_Current(gimbal, Gimb_Current);	
}

//未识别到目标不进行I计算
void Gimbal_Pitch_Auto_Output(motor_t *gimbal ,float tar)
{
	motor_pid_t *pid = gimbal->pid;
	
	/*--赋予目标值--*/
	pid->Imu_Out_Pid.target = tar + Pit_add;
	
	/*--进行PID计算--*/
	pid->Imu_Out_Pid.target = 
	constrain(pid->Imu_Out_Pid.target, IMU_HIG_P, IMU_LOW_P);

	pid->out = PID_CAL(&pid->Imu_Out_Pid,
	                   &pid->Imu_Inn_Pid,
	                   PITCH, PITCH_V, 1);
	/*--赋予电流--*/
	Gimbal_Motor_Current(gimbal, Gimb_Current);      //输出
}



void YAW_AUTO_CTRL(void)
{
	float tar = 0;

	tar = Gimb_Yaw_AUTO_Input();
	
	Gimbal_Yaw_Auto_Output(&Gimb_Motor[GIMB_Y], tar);
}

void PIT_AUTO_CTRL(void)
{
	float tar = 0;
	
	tar = Gimb_Pit_AUTO_Input();
	
	Gimbal_Pitch_Auto_Output(&Gimb_Motor[GIMB_P], tar);
}	
	


/*************
 *自动控制逻辑
 *电脑失联 cmd为0切换为原来模式
 *************/
void GIMBAL_AUTO_FRI_CTRL(void)
{	
  YAW_AUTO_CTRL();	
	PIT_AUTO_CTRL();
}




extern Vision_Cmd_Id_t Cmd_Last;
void GIMBAL_AUTO_Ctrl(void)
{
	Vision_Cmd_Id_t Cmd = vision_sensor.info->TxPacket.FrameHeader.cmd_id;
	
	if(VISION_ONLINE && Cmd && Find_Tar())
	{	
		
		/*--自瞄--*/
		if(ZM_STATE())
		{
			Yaw_add = AIM_YAW_ADD;
			Pit_add = AIM_PIT_ADD;	
		}
		
		/*--打符--*/
		if(DF_STATE())
		{
			Yaw_add = DF_YAW_ADD;
			Pit_add = DF_PIT_ADD;	
		}
		
		/*-自动状态-*/
		State.AUTO_STATE = ING;
		
		/*--控制--*/
		GIMBAL_AUTO_FRI_CTRL();		
	}
	else
	{	
		if(State.AUTO_STATE == ING)
		{
			GIMBAL_INIT_NOW();
		}
		
//		/*-保证机械模式也能先归位，避免甩头-*/
//		if(State.Move.mode == CHAS_FIRST)
//		{
//			State.Move.Init = ING;
//		}
		/*-自动状态-*/
		State.AUTO_STATE = NO;
		/*--控制--*/
		GIMBAL_GIMB_FRI_CTRL();		
	}
}






 /*--------------------------------------*/
/*---------------初始化-------------------*/

/*云台初始化*/
void GIMBAL_YAW_INIT(void)
{
	Judge_Dir(&Gimb_Motor[GIMB_Y]);
	
	Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = YAW;	
}
void GIMBAL_PIT_INIT(void)
{
  Gimb_Motor[GIMB_P].pid->Imu_Out_Pid.target = IMU_MID_P;

	Gimb_Motor[GIMB_P].pid->Mec_Out_Pid.target = MEC_MID_P;
}

/*-切换模式后，会目标切换到此时方向-*/
void GIMBAL_PIT_NOW_SET(void)
{
  Gimb_Motor[GIMB_P].pid->Imu_Out_Pid.target = PITCH;

	Gimb_Motor[GIMB_P].pid->Mec_Out_Pid.target = Gimb_Motor[GIMB_P].info->angle;
}

/*--获取转向pid误差--*/
float Yaw_Turn_Err(void)
{
	float err;
	err = Half_Turn(Gimb_Motor[GIMB_Y].pid->Turn_pid.target \
								- Gimb_Motor[GIMB_Y].info->angle, 8191);
	return err;
}
/*--获取pit机械模式误差--*/
float Pit_Mec_Err(void)
{
	float err;
	err = Half_Turn(Gimb_Motor[GIMB_P].pid->Mec_Out_Pid.target \
								- Gimb_Motor[GIMB_P].info->angle, 8191);
	return err;
}
/*--获取yaw机械模式误差--*/
float Yaw_Mec_Err(void)
{
	float err;
	err = Half_Turn(Gimb_Motor[GIMB_Y].pid->Mec_Out_Pid.target \
								- Gimb_Motor[GIMB_Y].info->angle, 8191);
	return err;
}

/**
 *开机初始化
 *TEST_MODE == 2
 *PID_SET
 *Start_gimbal_Task 使用
 **/
void GIMBAL_INIT(void)
{
	GIMBAL_YAW_INIT();
	GIMBAL_PIT_INIT();
}

/*--中途切换初始化,以此时角度做初始化--*/
void GIMBAL_INIT_NOW(void)
{
	GIMBAL_YAW_INIT();
	GIMBAL_PIT_NOW_SET();
}


































/*-初始化控制-*/
uint32_t GIMB_INIT_TIME, GIMB_INIT_TIME2;

void GIMBAL_INIT_CTRL(void)
{
	/*-获取开始初始化时间-*/
	if(RC_OFFLINE)
	{
		GIMBAL_INIT();
		GIMB_INIT_TIME  = HAL_GetTick();
		GIMB_INIT_TIME2 = HAL_GetTick();
	}
		
	/*-遥控在线云台初始化-*/	
	else
	{	
		/*-到位后目标值切换-*/
		if(abs(Yaw_Mec_Err()) < 50 && abs(Pit_Mec_Err()) < 50)
		{
			GIMBAL_INIT_NOW();
			State.Gimbal_init = OK;
		}
		/*-否则继续初始化-*/
		else
		{
			GIMBAL_INIT();	
			GIMBAL_CHAS_FRI_CTRL();	
		}
		
		/*-长时间未到位强制初始化成功-*/
		GIMB_INIT_TIME2 = HAL_GetTick();
		if(GIMB_INIT_TIME2 - GIMB_INIT_TIME > 1500)
		{
			GIMBAL_INIT_NOW();
			State.Gimbal_init = OK;
		}
	}
}


/******************************************/
/*
 *·未云台初始化底盘不动
 *·电脑失联下为GIMBFIR控制
 *·遥控失联卸力
 */
void GIMBAL_CTRL(void)
{

	/*-获取自动状态-*/
	State.AUTO_ING = AUTO_ING();

	/*-切换陀螺仪Kp-*/
	if(HAL_GetTick() > 4000)Kp = IMU_AUTO_KP;	
	else 										Kp = IMU_NORM_KP;	
	
#if TEST_MODE == 2	
	
	/*-获取开始初始化时间-*/
	if(RC_OFFLINE)
	{
		GIMBAL_INIT();
		GIMB_INIT_TIME  = HAL_GetTick();
		GIMB_INIT_TIME2 = HAL_GetTick();
	}
	
#endif
	
#if TEST_MODE == 0	

		if(!State.VISION_TEST)
			Gimb_Motor[GIMB_P].pid->Imu_Out_Pid.kp = PIT_IMU_KP;
		else 
			Gimb_Motor[GIMB_P].pid->Imu_Out_Pid.kp = PIT_IMU_KP * 0.6f;
		
		if(State.Gimbal_init == NO)
      GIMBAL_INIT_CTRL();
		
		else if(State.Func.state.AUTO_SHOOT == OK)	
			GIMBAL_AUTO_Ctrl();
		
		else
		{
			State.AUTO_STATE = NO;
			
			if(State.Move.state.CHAS_FIRST == OK)
				GIMBAL_CHAS_FRI_CTRL();
		
			else if( State.Move.state.TOP        == OK
						|| State.Move.state.GIMB_FIRST == OK
						||(State.Move.mode == CHAS_FIRST && State.Move.Init == ING))
				GIMBAL_GIMB_FRI_CTRL();
		} 
		

		if(RC_ONLINE)
			Send_Current(GIMBA_CAN, GIMBA_ID, Gimb_Current);
		else
			Send_Current(GIMBA_CAN, GIMBA_ID, Gimb_Stop_Cmd);
		
#endif
}



/*
 ·PID调试
 ·统一向上IMU 向下MEC
 */
void SET_GIMB_PID(void)
{
	if(State.Gimbal_init != OK)
	{
		GIMBAL_INIT();
		State.Gimbal_init = OK;
	}
	
	if(SW1_UP)  YAW_IMU_CTRL();
	if(SW1_DOWN)YAW_MEC_CTRL();
	
	if(SW2_UP)  PIT_IMU_CTRL();
	if(SW2_DOWN)PIT_MEC_CTRL();

	if(SW1_MID && SW2_MID)
		Kp = IMU_NORM_KP;
	else
		Kp = IMU_AUTO_KP;
	
	//非处于调试状态关闭电流
	if(SW1_MID)Gimb_Current[0] = 0;
	if(SW2_MID)Gimb_Current[1] = 0;	
	
	Send_Current(GIMBA_CAN, GIMBA_ID, Gimb_Current);
}








