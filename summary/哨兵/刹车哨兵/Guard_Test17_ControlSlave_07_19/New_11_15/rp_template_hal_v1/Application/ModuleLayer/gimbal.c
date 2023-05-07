/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"
#include "vision_task.h"
#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"
#include "judge_sensor.h"
#include "chassis.h"

float real_yaw_rate;

float pitch_real_angle_target;  //增加为往下
float yaw_real_angle_target;   //增加为逆时针
float pitch_init_angle_target;
float yaw_init_angle_target;

extern Queue_t *pitch_history_queue;
extern Queue_t *yaw_history_queue;
/* Private macro -------------------------------------------------------------*/
//云台自动模式下每次转动的角度
#define PITCH_ROTATE_UNIT 2
#define YAW_ROTATE_UNIT 5

/* Private function prototypes -----------------------------------------------*/
void GIMBAL_Init(void);
void GIMBAL_Ctrl(void);
void GIMBAL_Test(void);
void GIMBAL_SelfProtect(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 云台电机本地驱动
drv_can_t				*gimb_drv[GIMBAL_MOTOR_CNT];
motor_t			*gimb_motor[GIMBAL_MOTOR_CNT];
motor_info_t	*gimb_motor_info[GIMBAL_MOTOR_CNT];

/* Exported variables --------------------------------------------------------*/
// 云台电机PID控制器
gimbal_motor_pid_t 	gimb_motor_pid[GIMBAL_MOTOR_CNT] = {
	[GIMB_PITCH_CLASS] = {
		.speed.kp = 16.f, //6 //12       12    20   16
		.speed.ki = 0.7f, //0.2  //0.1   0.7     1  0.7
		.speed.kd = 0.f,
		.speed.integral_max = 26000.f,
		.speed.out_max = 28000.f,
		.angle.kp = 20.f, //20  //20
		.angle.ki = 0.f,
		.angle.kd = 0.f,
		.angle.integral_max =8000.f,
		.angle.out_max = 16000.f,
	},
	[GIMB_YAW_CLASS] = {
		.speed.kp = 16.f,  //12    16
		.speed.ki = 0.8f,  //0.5    0.8
		.speed.kd = 0.f,
		.speed.integral_max = 20000.f,
		.speed.out_max = 28000.f,
		.angle.kp = 15.f,  //15
		.angle.ki = 0.f,
		.angle.kd = 0.f,
		.angle.integral_max = 8000.f,
		.angle.out_max = 16000.f,
	},
};

// 云台模块控制器
gimbal_ctrl_t		gimb_ctrl = {
	.motor = &gimb_motor_pid,
};

// 云台模块传感器
gimbal_dev_t		gimb_dev = {
	.gimb_motor[GIMB_PITCH_CLASS] = &motor[GIMB_PITCH],
	.gimb_motor[GIMB_YAW_CLASS] = &motor[GIMB_YAW],	
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 云台模块信息
gimbal_info_t 	gimb_info = {
	.remote_mode = RC,
	.local_mode = GIMBAL_MODE_NORMAL,
};

gimbal_t gimbal = {
	.controller = &gimb_ctrl,
	.dev = &gimb_dev,
	.info = &gimb_info,
	.init = GIMBAL_Init,
	.test = GIMBAL_Test,
	.ctrl = GIMBAL_Ctrl,
	.self_protect = GIMBAL_SelfProtect,
};
//识别和丢失到目标之间切换的标志位
bool access_identify = false;

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
/**
 *	@brief	云台电机PID参数初始化
 */
void GIMBAL_PidParamsInit(gimbal_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}
}

/**
 *	@brief	云台电机卸力
 */
static void GIMBAL_Stop(gimbal_motor_pid_t *pid)
{
	for(uint8_t i=0; i < GIMBAL_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		gimb_drv[i]->add_halfword(gimb_drv[i], (int16_t)pid[i].out);
	}
}

/**
 *	@brief	云台电机PID输出
 */
static void GIMBAL_PidOut(gimbal_motor_pid_t *pid)
{
	for(uint8_t i=0; i < GIMBAL_MOTOR_CNT; i++) {
		if(gimb_motor[i]->work_state == DEV_ONLINE) {
            gimb_drv[i]->add_halfword(gimb_drv[i], (int16_t)pid[i].out);
		} 
		else {
            gimb_drv[i]->add_halfword(gimb_drv[i], 0);
		}
	}    
}

///**
// *	@brief	云台电机速度环
// */
//static void GIMBAL_Speed_PidCalc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx)
//{
//	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
//	single_pid_ctrl(&pid[MOTORx].speed);
//	pid[MOTORx].out = pid[MOTORx].speed.out;     
//}

/**
 *	@brief	云台电机位置环内环
 */
static void GIMBAL_Angle_PidCalc_In(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;  
}

/**
 *	@brief	云台电机位置环
 */
static void GIMBAL_Angle_PidCalc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	single_pid_ctrl(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;      //内环的目标等于外环的输出
	GIMBAL_Angle_PidCalc_In(pid,MOTORx);
}

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
/**
 *	@brief	云台获取系统信息
 */
void GIMBAL_GetSysInfo(void)
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		gimb_info.remote_mode = RC;
	}
	else if(sys.remote_mode == KEY) {
		gimb_info.remote_mode = KEY;
	}
	else if(sys.remote_mode == AUTO) {
		gimb_info.remote_mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) {
		gimb_info.remote_mode = INSPECTION;
	}
	/*----本地模式修改----*/
}



/**
* @brief 云台手控制下云台向后扫描
* @note Ctrl+S 哨兵下云台向后扫描
*/
void GIMBAL_GetAerialInfo(void)
{
	if(judge_sensor.info->GameRobotStatus.robot_id == 7)//红色id
	{
		if((judge_sensor.info->AerialData.send_id == 6) && (judge_sensor.info->AerialData.receive_id == 7))  //无人机发送给哨兵
		{
			if(judge_sensor.info->AerialData.cmd == back_scan)
			{
				gimbal.back_scan = true;
			}
			else
			{
				gimbal.back_scan = false;				
			}
		}
	}
	else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//蓝色id
	{
		if((judge_sensor.info->AerialData.send_id == 106) && (judge_sensor.info->AerialData.receive_id == 107))  //无人机发送给哨兵
		{
			if(judge_sensor.info->AerialData.cmd == back_scan)
			{
				gimbal.back_scan = true;
			}
			else
			{
				gimbal.back_scan = false;
			}
		}
	}
	
}


void GIMBAL_UpdateController(void)
{
	//角度总和
	gimb_motor_pid[GIMB_PITCH_CLASS].angle.measure = -gimb_motor_info[GIMB_PITCH_CLASS]->angle_sum;
	gimb_motor_pid[GIMB_YAW_CLASS].angle.measure = gimb_motor_info[GIMB_YAW_CLASS]->angle_sum;	
	
	//陀螺仪速度
	real_yaw_rate = Gimbal_Real_YawRate();
	gimb_motor_pid[GIMB_PITCH_CLASS].speed.measure = -imu_sensor.info->rate_yaw;
	gimb_motor_pid[GIMB_YAW_CLASS].speed.measure = real_yaw_rate;
	//卡尔曼
	gimb_motor_pid[GIMB_PITCH_CLASS].speed.measure = KalmanFilter(&pitch_measure_rateKF ,-imu_sensor.info->rate_yaw);
	gimb_motor_pid[GIMB_YAW_CLASS].speed.measure = KalmanFilter(&yaw_measure_rateKF, real_yaw_rate);
	
  //电机速度
//	gimb_motor_pid[GIMB_PITCH_CLASS].speed.measure = gimb_motor_info[GIMB_PITCH_CLASS]->speed;
//	gimb_motor_pid[GIMB_YAW_CLASS].speed.measure = gimb_motor_info[GIMB_YAW_CLASS]->speed;
	
}

/* 应用层 --------------------------------------------------------------------*/
/* 任务层 --------------------------------------------------------------------*/
void GIMBAL_Init(void)
{
	gimb_drv[GIMB_PITCH_CLASS] = gimb_dev.gimb_motor[GIMB_PITCH_CLASS]->driver;
	gimb_drv[GIMB_YAW_CLASS] = gimb_dev.gimb_motor[GIMB_YAW_CLASS]->driver;

	
	gimb_motor[GIMB_PITCH_CLASS] = gimb_dev.gimb_motor[GIMB_PITCH_CLASS];
	gimb_motor[GIMB_YAW_CLASS] = gimb_dev.gimb_motor[GIMB_YAW_CLASS];
	
	gimb_motor_info[GIMB_PITCH_CLASS] = gimb_dev.gimb_motor[GIMB_PITCH_CLASS]->info;
	gimb_motor_info[GIMB_YAW_CLASS] = gimb_dev.gimb_motor[GIMB_YAW_CLASS]->info;	
	
	gimbal.back_scan = false;
 
	gimbal.rc_ch0 = 0;
	gimbal.rc_ch1 = 0;

}

void GIMBAL_Reset(void)
{
	float yaw_res1;
	float yaw_res2;
	float yaw_res3;
	float pitch_AngleDiff;
	float yaw_AngleDiff;
	if(flag.gimbal.reset_start == true)
	{		
		pitch_AngleDiff = -(GIMBAL_ANGLE_MID_PITCH - gimb_motor_info[GIMB_PITCH_CLASS]->angle);
		yaw_res1 = GIMBAL_ANGLE_MID_YAW - gimb_motor_info[GIMB_YAW_CLASS]->angle;                 //先计算到达复位位置所需的角路程
		yaw_res2 = GIMBAL_ANGLE_MID_YAW - gimb_motor_info[GIMB_YAW_CLASS]->angle + 8192;
		yaw_res3 = GIMBAL_ANGLE_MID_YAW - gimb_motor_info[GIMB_YAW_CLASS]->angle - 8192;

		if((abs(yaw_res1) > abs(yaw_res2)) && (abs(yaw_res3) > abs(yaw_res2)))         //取最小的复位路程，即就近复位
		{
			yaw_AngleDiff = yaw_res2;   
		}
		else if((abs(yaw_res1) < abs(yaw_res2)) && (abs(yaw_res1)< abs(yaw_res3)))
		{
			yaw_AngleDiff = yaw_res1;
		}
		else if((abs(yaw_res3) < abs(yaw_res1)) && (abs(yaw_res3)< abs(yaw_res2)))
		{
			yaw_AngleDiff = yaw_res3;
		}

		gimb_motor_info[GIMB_PITCH_CLASS]->angle_sum = 0; //复位前先清理行程
		gimb_motor_info[GIMB_YAW_CLASS]->angle_sum = 0;
		
		pitch_init_angle_target = pitch_AngleDiff;
		yaw_init_angle_target = yaw_AngleDiff;
		
		pitch_real_angle_target = 0;
		yaw_real_angle_target = 0;
		flag.gimbal.reset_ok = true;   //复位完成
		flag.gimbal.reset_start = false;  //复位完成，下次不进行二次复位
	}
	
}


void GIMBAL_GetInfo(void)
{
	GIMBAL_GetSysInfo();
	GIMBAL_GetAerialInfo();
	GIMBAL_UpdateController();
}

void GIMBAL_SelfProtect(void)
{
	GIMBAL_Stop(gimb_motor_pid);
	GIMBAL_PidParamsInit(gimb_motor_pid, GIMBAL_MOTOR_CNT);
	GIMBAL_GetInfo();
}

void GIMBAL_PidCtrl(void)
{
//	// 云台电机速度环
//	GIMBAL_Speed_PidCalc(gimb_motor_pid, GIMB_PITCH_CLASS);
//	GIMBAL_Speed_PidCalc(gimb_motor_pid, GIMB_YAW_CLASS);
	
	// 云台电机位置环
	GIMBAL_Angle_PidCalc(gimb_motor_pid, GIMB_PITCH_CLASS);
	GIMBAL_Angle_PidCalc(gimb_motor_pid, GIMB_YAW_CLASS);
	
	gimb_motor_pid[GIMB_PITCH_CLASS].out = -gimb_motor_pid[GIMB_PITCH_CLASS].out;
	GIMBAL_PidOut(gimb_motor_pid);
}

/**
 *	@brief	云台打弹补偿角计算
 *  根据视觉发来的距离来拟合补偿曲线
 */
void GIMBAL_OffsetCal(void)
{
	float npw_distance;
	npw_distance = vision.distance_filtered;
	/*---yaw轴补偿---*/
	if(npw_distance >= 400) 
	{
		vision.yaw_angle_offset = -10 + 0.01f * npw_distance - 7.7136f;
	}
	else if(npw_distance < 400)
	{
		vision.yaw_angle_offset = -10;		
	}
	/*---pitch轴补偿---*/
	if(npw_distance < 400)
	{
		vision.pitch_angle_offset = 0.1541f * npw_distance - 53.285f;		
	}
	else if(npw_distance >= 400)
	{
		vision.pitch_angle_offset = 17 + 0.01f * npw_distance - 17.714f;
	}
	
	if(npw_distance <= 150)  //关闭补偿
	{
		vision.pitch_angle_offset = 0;
		vision.yaw_angle_offset = 0;
	}
	
}


/**
* @brief 检测背后伤害
* @param void
* @return void
*检测到后作用一段时间，再恢复.
*/
static bool  Hurt_back(void)
{
	
	static int32_t safe_cnt;
	static bool result = false;

	if(judge_sensor.info->hurt_data_update)//伤害数据更新
	{
		chassis.hurt_data_update = true;      //底盘伤害更新
		judge_sensor.info->hurt_data_update = false;  //底盘运动策略无重置该标志位
		if((judge_sensor.info->RobotHurt.armor_id == 1)&&(judge_sensor.info->RobotHurt.hurt_type == 0))
		{
			safe_cnt = 0;
			result = true;
		}
	}
	else
	{
		safe_cnt++;
		if (safe_cnt>3000)//6s
		{
			safe_cnt = 0;
			result = false;
		}		
	}

  return result;
}


/**
 *	@brief	云台自动控制
 */
void Pitch_Auto(void)
{
	static uint8_t direction = Auto_Up;  //初始巡航方向
	static uint8_t cnt = 0;        //改变目标值的时间间隔
	cnt++;
	if(pitch_real_angle_target >= GIMBAL_ANGLE_AUTO_PITCH_MAX)
	{
		direction = Auto_Down;
	}
	else if(pitch_real_angle_target <= GIMBAL_ANGLE_AUTO_PITCH_MIN)
	{
		direction = Auto_Up;
	}
	if(cnt > 0)
	{
		switch(direction)
		{
			case Auto_Up: pitch_real_angle_target += PITCH_ROTATE_UNIT; break;  //2
			case Auto_Down: pitch_real_angle_target -= PITCH_ROTATE_UNIT; break;
		}
//				//限幅
//		pitch_real_angle_target = constrain(pitch_real_angle_target,GIMBAL_ANGLE_PITCH_MIN,GIMBAL_ANGLE_PITCH_MAX);
		cnt = 0;       //计数清空为下一次做准备
	}
}

void Yaw_Auto(void)
{
	static uint8_t direction = Auto_Right;  //初始巡航方向
	static uint8_t cnt = 0;        //改变目标值的时间间隔
	static bool mode_flag = true;
		
	cnt++;
	
	if((Hurt_back() == true) || (gimbal.back_scan))       //若背后有伤害源,向后扫描;或者云台手接管控制
	{
		if(mode_flag)
		{
			mode_flag = false;
			yaw_real_angle_target -= 4096;
		}
		
		if(((int)yaw_real_angle_target % 8192) >= (GIMBAL_ANGLE_YAW_MAX - 4096))
		{
			direction = Auto_Right;
		}
		else if(((int)yaw_real_angle_target % 8192) <= (GIMBAL_ANGLE_YAW_MIN - 4096))
		{
			direction = Auto_Left;
		}		
	}
	else        //背后无伤害源
	{
		if(!mode_flag)
		{
			mode_flag = true;
			yaw_real_angle_target += 4096;
		}
		if(((int)yaw_real_angle_target % 8192) >= GIMBAL_ANGLE_YAW_MAX)
		{
			direction = Auto_Right;
		}
		else if(((int)yaw_real_angle_target % 8192) <= GIMBAL_ANGLE_YAW_MIN)
		{
			direction = Auto_Left;
		}			
	}
	if(cnt > 0)
	{
		switch(direction)
		{
			case Auto_Left: yaw_real_angle_target += YAW_ROTATE_UNIT; break;  //5
			case Auto_Right: yaw_real_angle_target -= YAW_ROTATE_UNIT; break;
		}
		
		cnt = 0;       //计数清空为下一次做准备
	}
}

/**
 *	@brief	云台遥控器控制
 *  @note   关闭自瞄
 */
void GIMBAL_RcCtrl(void)
{
	/* 遥控器控制注释 */
	pitch_real_angle_target += KalmanFilter(&rc_gimbal_ch1,gimbal.rc_ch1 / 660.f * 10);
	yaw_real_angle_target += KalmanFilter(&rc_gimbal_ch0,-gimbal.rc_ch0/660.f *10);
	
	//限幅
	pitch_real_angle_target = constrain(pitch_real_angle_target,GIMBAL_ANGLE_PITCH_MIN,GIMBAL_ANGLE_PITCH_MAX);
	//给定目标到PIDtarget
	gimb_motor_pid[GIMB_PITCH_CLASS].angle.target = pitch_init_angle_target + pitch_real_angle_target;   
	gimb_motor_pid[GIMB_YAW_CLASS].angle.target = yaw_init_angle_target + yaw_real_angle_target;

}

void GIMBAL_KeyCtrl(void)
{

}


/**
 *	@brief	云台自动控制
 */
void GIMBAL_AutoCtrl(void)
{
	static float pitch_now_passed_angle = 0;   //当前云台走过的角度(基于复位零点)
  static float yaw_now_passed_angle = 0;

	if(vision.identify_ok)   //如果识别到了目标
	{
		if(vision.target_update)
		{
			vision.target_update = false;			
			//计算打弹补偿角
//			GIMBAL_OffsetCal();
			

		}
		//计算云台当前转过的角度
		pitch_now_passed_angle = gimb_motor_pid[GIMB_PITCH_CLASS].angle.measure - pitch_init_angle_target;
		yaw_now_passed_angle = gimb_motor_pid[GIMB_YAW_CLASS].angle.measure - yaw_init_angle_target;		

//		//最终期望（还未给定到PID）
//		pitch_real_angle_target = pitch_now_passed_angle + vision.pitch_angle_raw ;
//		yaw_real_angle_target = yaw_now_passed_angle + vision.yaw_angle_raw ;
//		//最终期望（还未给定到PID)(未矫正)
//		pitch_real_angle_target = -(vision.pitch_angle_err) + pitch_now_passed_angle;
//		yaw_real_angle_target = (vision.yaw_angle_err) + yaw_now_passed_angle;
//		//卡尔曼
//		pitch_real_angle_target = KalmanFilter(&pitch_real_target_KF,(-(vision.pitch_angle_err) - pitch_init_angle_target));
//		yaw_real_angle_target = KalmanFilter(&yaw_real_target_KF,((vision.yaw_angle_err) - yaw_init_angle_target));
		

//		//最终期望（还未给定到PID)(已矫正)
//		pitch_real_angle_target = vision.pitch_angle_corrected - gimb_motor_pid[GIMB_PITCH_CLASS].angle.measure - pitch_init_angle_target;
//		yaw_real_angle_target = vision.yaw_angle_corrected - gimb_motor_pid[GIMB_YAW_CLASS].angle.measure - yaw_init_angle_target;
		//误差矫正
		Vision_Correct();
		pitch_real_angle_target = -vision.pitch_angle_err_corrected + pitch_now_passed_angle;
		yaw_real_angle_target = vision.yaw_angle_err_corrected + yaw_now_passed_angle;
		
	}
	else   //没有识别到目标，云台继续扫描
	{
		Pitch_Auto();
		Yaw_Auto();
	}

	//限幅
	pitch_real_angle_target = constrain(pitch_real_angle_target,GIMBAL_ANGLE_PITCH_MIN,GIMBAL_ANGLE_PITCH_MAX);
	//给定目标到PIDtarget
	gimb_motor_pid[GIMB_PITCH_CLASS].angle.target = pitch_init_angle_target + pitch_real_angle_target;   
	gimb_motor_pid[GIMB_YAW_CLASS].angle.target = yaw_init_angle_target + yaw_real_angle_target;
	
}

/**
 *	@brief	云台自检控制
 *  @note   开启自瞄
 */
void GIMBAL_InspectionCtrl(void)
{
//	/* 调视觉数据时，遥控器控制注释 */
//	pitch_real_angle_target += KalmanFilter(&rc_gimbal_ch1,rc_sensor.info->ch1 / 660.f * 10);
//	yaw_real_angle_target += KalmanFilter(&rc_gimbal_ch0,-rc_sensor.info->ch0/660.f *10);
	
	/********/
	static float pitch_now_passed_angle = 0;   //当前云台走过的角度(基于复位零点)
  static float yaw_now_passed_angle = 0;


	if(vision.identify_ok)   //如果识别到了目标
	{
		if(vision.target_update)
		{
			vision.target_update = false;			
					
			//计算打弹补偿角
//			GIMBAL_OffsetCal();
			
		}
	
		//计算云台当前转过的角度
		pitch_now_passed_angle = gimb_motor_pid[GIMB_PITCH_CLASS].angle.measure - pitch_init_angle_target;
		yaw_now_passed_angle = gimb_motor_pid[GIMB_YAW_CLASS].angle.measure - yaw_init_angle_target;		

//		//最终期望（还未给定到PID）
//		pitch_real_angle_target = pitch_now_passed_angle + vision.pitch_angle_raw;
//		yaw_real_angle_target = yaw_now_passed_angle + vision.yaw_angle_raw;	
		//最终期望（还未给定到PID)(已矫正)
//		pitch_real_angle_target = pitch_now_passed_angle + vision.pitch_angle_corrected ;
//		yaw_real_angle_target = yaw_now_passed_angle + vision.yaw_angle_corrected ;
		
//		pitch_real_angle_target = -0.7f *(vision.pitch_angle_err) + pitch_now_passed_angle;
//		yaw_real_angle_target = 0.7f *(vision.yaw_angle_err) + yaw_now_passed_angle;
		//误差矫正
		Vision_Correct();
		pitch_real_angle_target = -vision.pitch_angle_err_corrected + pitch_now_passed_angle;
		yaw_real_angle_target = vision.yaw_angle_err_corrected + yaw_now_passed_angle;

	}
	else   //没有识别到目标，云台继续扫描
	{
//		Pitch_Auto();
//		Yaw_Auto();
	}

	//限幅
	pitch_real_angle_target = constrain(pitch_real_angle_target,GIMBAL_ANGLE_PITCH_MIN,GIMBAL_ANGLE_PITCH_MAX);
	//给定目标到PIDtarget
	gimb_motor_pid[GIMB_PITCH_CLASS].angle.target = pitch_init_angle_target + pitch_real_angle_target;   
	gimb_motor_pid[GIMB_YAW_CLASS].angle.target = yaw_init_angle_target + yaw_real_angle_target;

}


void GIMBAL_Ctrl(void)
{
	/*----信息读入----*/
	GIMBAL_GetInfo();
	/*----云台复位----*/	
	GIMBAL_Reset();
	/*----期望修改----*/ 
	if(gimb_info.remote_mode == RC) {
		GIMBAL_RcCtrl();
	}
	else if(gimb_info.remote_mode == KEY) {
		GIMBAL_KeyCtrl();
	}
	else if(gimb_info.remote_mode == AUTO) {
		GIMBAL_AutoCtrl();
	}
	else if(gimb_info.remote_mode == INSPECTION) {
		GIMBAL_InspectionCtrl();
	}
	
	/*----最终输出----*/
	GIMBAL_PidCtrl();
	GIMBAL_Test();
}

//float pitch_angle_target;
float pitch_angle_measure;
//float pitch_angle_out;
//float pitch_speed_target;
//float pitch_speed_measure;
//float pitch_speed_out;

//float yaw_angle_target;
float yaw_angle_measure;
//float yaw_angle_out;
//float yaw_speed_target;
//float yaw_speed_measure;
//float yaw_speed_out;

//float yaw_angle_sum;
void GIMBAL_Test(void)
{
//	pitch_angle_target = gimb_motor_pid[GIMB_PITCH_CLASS].angle.target;
//	pitch_angle_measure = gimb_motor_pid[GIMB_PITCH_CLASS].angle.measure;
//	pitch_speed_target = gimb_motor_pid[GIMB_PITCH_CLASS].speed.target;
//	pitch_speed_measure = gimb_motor_pid[GIMB_PITCH_CLASS].speed.measure;
//   
//	yaw_angle_target = gimb_motor_pid[GIMB_YAW_CLASS].angle.target;
	yaw_angle_measure = motor[GIMB_YAW].info->angle;
	pitch_angle_measure = motor[GIMB_PITCH].info->angle;

//	yaw_angle_out = gimb_motor_pid[GIMB_YAW_CLASS].angle.out;
//	yaw_speed_target = gimb_motor_pid[GIMB_YAW_CLASS].speed.target;
//	yaw_speed_measure = gimb_motor_pid[GIMB_YAW_CLASS].speed.measure;
//	yaw_speed_out = gimb_motor_pid[GIMB_YAW_CLASS].speed.out;
	
}
