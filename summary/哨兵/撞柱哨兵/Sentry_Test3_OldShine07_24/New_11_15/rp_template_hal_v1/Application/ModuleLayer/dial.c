/* Includes ------------------------------------------------------------------*/
#include "dial.h"
#include "drv_io.h"
#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"
#include "driver.h"
#include "control_task.h"
#include "judge_sensor.h"
#include  "vision_sensor.h"
/* Private macro -------------------------------------------------------------*/
/*开火阈值*/
#define PITCH_THRESHOLD 3
#define YAW_THRESHOLD 5
/*卡弹相关*/
#define STUCK_THRESHOLD 200
#define STUCK_HANDLE_TIME 200
#define STUCK_MOTOR_SPEED 10

/* Private function prototypes -----------------------------------------------*/
void DIAL_Init(void);
void DIAL_Ctrl(void);
void DIAL_Test(void);
void DIAL_SelfProtect(void);

void DIAL_NormoalShot(void);     
void DIAL_TripleShot(void);
void DIAL_RepeatingShot(void);
void DIAL_StuckHandle(void);   

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 拨盘电机本地驱动
drv_can_t				*dia_drv[DIAL_MOTOR_CNT];
motor_t			*dia_motor[DIAL_MOTOR_CNT];
motor_info_t	*dia_motor_info[DIAL_MOTOR_CNT];

Friction_t friction = {.Target = 0};
//int8_t dial_direction = 1; //拨盘方向
uint16_t fric_cnt = 0;    //摩擦轮开启时间计时，用于摩擦轮转速稳定后才发射弹丸的控制

//射频
float shoot_freq_val = SHOOT_FREQ_TEN;  //初始10射频
//射速
float firing_rate = FIRING_RATE_HIGH;

// 拨盘电机PID控制器
dial_motor_pid_t 	dia_motor_pid[DIAL_MOTOR_CNT] = {
	[DIAL_CLASS] = {
		.speed.kp = 6,       //6
		.speed.ki = 0.02f,   //0.02
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 8000,
		.angle.kp = 0.15,    //0.15
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	},
};

// 拨盘模块控制器
dial_ctrl_t		dia_ctrl = {
	.motor = &dia_motor_pid,
};

// 拨盘模块传感器
dial_dev_t		dia_dev = {
	.dia_motor[DIAL_CLASS] = &motor[DIAL],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 拨盘模块信息
dial_info_t 	dia_info = {
	.remote_mode = RC,
	.local_mode = DIAL_MODE_NORMAL,
	.normal_init_flag = false,
	.triple_init_flag = false,
	.repeating_init_flag = false,
	.stuck_init_flag = false,
};

dial_t dial = {
	.controller = &dia_ctrl,
	.dev = &dia_dev,
	.info = &dia_info,
	.init = DIAL_Init,
	.test = DIAL_Test,
	.ctrl = DIAL_Ctrl,
	.self_protect = DIAL_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/

/**
 *	@brief	卡弹检查
 */
void StuckCheck(void)
{
	static uint16_t stuck_cnt = 0;

	if(!dial.stuck)	
	{
		if(dia_info.local_mode == DIAL_MODE_REPEATING)
		{
			if(dial.fire_open)
			{
				if(abs(dia_motor_pid[DIAL_CLASS].speed.measure) < STUCK_MOTOR_SPEED)
				{
					stuck_cnt++;
				}
				else
				{
					stuck_cnt = 0;
				}
				
				if(stuck_cnt > STUCK_THRESHOLD)
				{
					dial.stuck = true;   //发生卡弹
					stuck_cnt = 0;    //计数清零
				}
			}
		}
	}
	else if(!dial.fire_open)      //无开火
	{
		stuck_cnt = 0;        //计数清零
	}
	
}

/**
 *	@brief	超射速处理
 */
void RateLimit_Process(void)
{	
	static float last_speed = 0;
	static float now_speed = 0;
	
	static uint16_t low_speed_cnt = 0;
	
	if(judge_sensor_info.ShootData.shooter_id == 1)
	{
		last_speed = now_speed;      //记录上一次射速
		now_speed = judge_sensor_info.ShootData.bullet_speed;  //获得当前射速
		
		if(judge_sensor_info.shoot_update)
		{
			judge_sensor_info.shoot_update = false;      //清空枪管数据更新标志
			
			if((judge_sensor_info.ShootData.bullet_speed > 27) && (now_speed != last_speed))  //超射速降速
			{
				firing_rate -= 2;       //调整单位,可以自行调整调整单位
				if(firing_rate < 0)
				{
					firing_rate = 0;
				}
			}
			else if((judge_sensor_info.ShootData.bullet_speed < 26) && (now_speed != last_speed))  //射速补偿，避免射速过低
			{
				low_speed_cnt++;
				if(low_speed_cnt > 0)
				{
					low_speed_cnt = 0;
					firing_rate += 2;    //调整单位,可以自行调整调整单位
					if(firing_rate > 800)
					{
						firing_rate = 800;
					}
				}
			}
		}
	}
}

/**
 *	@brief	摩擦轮控制
 */
void Fric_Control(void)
{
	if(dial.fric_unlock)   //摩擦轮已解锁
	{
		RateLimit_Process();  //射速限制，防止一直超射速
		
		if(dial.fric_open)       //开启摩擦轮
		{
			friction.Target = firing_rate;    
		}
		else                 //关闭摩擦轮
		{
			friction.Target = 0;
		}
		NormalPwm[FRICTION_L] = friction.Target;
		NormalPwm[FRICTION_R] = friction.Target;	
		
		FRICTION_PwmOut(NormalPwm[FRICTION_L],NormalPwm[FRICTION_R]);
	}
}
/**
 *	@brief	拨盘电机PID参数初始化
 */
void DIAL_PidParamsInit(dial_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}
}

/**
 *	@brief	拨盘和摩擦轮电机卸力
 */
static void DIAL_Stop(dial_motor_pid_t *pid)
{
	for(uint8_t i=0; i < DIAL_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		dia_drv[i]->add_halfword(dia_drv[i], (int16_t)pid[i].out);
	}
	//若摩擦轮解锁过则让其停止
	if(dial.fric_unlock)
	{
		FRICTION_PwmOut(0, 0);
	}
}

/**
 *	@brief	拨盘电机PID输出
 */
static void DIAL_PidOut(dial_motor_pid_t *pid)
{
	for(uint8_t i=0; i < DIAL_MOTOR_CNT; i++) {
		if(dia_motor[i]->work_state == DEV_ONLINE) {
            dia_drv[i]->add_halfword(dia_drv[i], (int16_t)pid[i].out);
		} 
		else {
            dia_drv[i]->add_halfword(dia_drv[i], 0);
		}
	}    
}

/**
 *	@brief	拨盘电机速度环
 */
static void DIAL_Speed_PidCalc(dial_motor_pid_t *pid, dial_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	拨盘电机位置环
 */
static void DIAL_Angle_PidCalc(dial_motor_pid_t *pid, dial_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	single_pid_ctrl(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	DIAL_Speed_PidCalc(pid,MOTORx);
}


/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
/**
 *	@brief	拨盘获取系统信息
 */
void DIAL_GetSysInfo(void)
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		dia_info.remote_mode = RC;
	}
//	else if(sys.remote_mode == KEY) {
//		dia_info.remote_mode = KEY;
//	}
	else if(sys.remote_mode == AUTO)
	{
		dia_info.remote_mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) 
	{
		dia_info.remote_mode = INSPECTION;
	}
	
	/*----本地模式修改----*/
}

/**
 *	@brief	云台手信息获取
 */
void DIAL_GetAerialInfo(void)
{
	if(judge_sensor.info->GameRobotStatus.robot_id == 7)//红色id
	{
		if((judge_sensor.info->AerialData.send_id == 6) && (judge_sensor.info->AerialData.receive_id == 7))  //无人机发送给哨兵
		{
			if(judge_sensor.info->AerialData.cmd == stop_fire)
			{
				dial.stop_fire = true;
				leader_info.TxPacket.ctrl_mode.stop_fire = true;
			}
			else
			{
				dial.stop_fire = false;
				leader_info.TxPacket.ctrl_mode.stop_fire = false;
			}
		}
	}
	else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//蓝色id
	{
		if((judge_sensor.info->AerialData.send_id == 106) && (judge_sensor.info->AerialData.receive_id == 107))  //无人机发送给哨兵
		{
			if(judge_sensor.info->AerialData.cmd == stop_fire)
			{
				dial.stop_fire = true;
				leader_info.TxPacket.ctrl_mode.stop_fire = true;
			}
			else
			{
				dial.stop_fire = false;
				leader_info.TxPacket.ctrl_mode.stop_fire = false;
			}
		}
	}
	
}

/**
* @brief 云台手控制开火状态
* @note Ctrl+W 哨兵上下云台停止开火
*/
void Aerial_Ctrl(void)            
{
	if(dial.stop_fire)
	{
		dial.fire_open = false;     //下云台拨盘锁定
		leader_info.TxPacket.ctrl_mode.fire_open = false;   //上云台拨盘锁定			
	}
}

void DIAL_UpdateController(void)
{
	dia_motor_pid[DIAL_CLASS].speed.measure = dia_motor_info[DIAL_CLASS]->speed;
	dia_motor_pid[DIAL_CLASS].angle.measure = dia_motor_info[DIAL_CLASS]->angle_sum;
}

/* 应用层 --------------------------------------------------------------------*/
/* 任务层 --------------------------------------------------------------------*/
void DIAL_Init(void)
{
	dia_drv[DIAL_CLASS] = dia_dev.dia_motor[DIAL_CLASS]->driver;

	dia_motor[DIAL_CLASS] = dia_dev.dia_motor[DIAL_CLASS];
	
	dia_motor_info[DIAL_CLASS] = dia_dev.dia_motor[DIAL_CLASS]->info;
	
	dial.fire_open = false;
	
	dial.fric_open = false;
	
	dial.stop_fire = false;
	
	dial.lock = true;
	
	dial.stuck = false;
	
	dial.fric_unlock = false;
}

void DIAL_GetInfo(void)
{
	DIAL_GetSysInfo();
	DIAL_GetAerialInfo();
	DIAL_UpdateController();
}

void DIAL_SelfProtect(void)
{
	DIAL_Stop(dia_motor_pid);
	DIAL_PidParamsInit(dia_motor_pid, DIAL_MOTOR_CNT);
	DIAL_GetInfo();
}

void DIAL_ModeSelect(void)        //拨盘本地模式选择(可在此设置判断是否强制停火)
{
	if(!dial.stuck)      //若无卡弹
	{
		switch(dia_info.local_mode)
		{
			case DIAL_MODE_NORMAL:      DIAL_NormoalShot();      break;
			case DIAL_MODE_TRIPLE_SHOT: DIAL_TripleShot();       break;
			case DIAL_MODE_REPEATING:   DIAL_RepeatingShot();    break;
		}	
	}
	else    //若发生卡弹
	{
		DIAL_StuckHandle(); //卡弹处理
	}
}

void DIAL_PidCtrl(void)
{
	if(!dial.stuck)
	{
		if(dia_info.local_mode == DIAL_MODE_REPEATING)
		{
			// 拨盘电机速度环
			DIAL_Speed_PidCalc(dia_motor_pid, DIAL_CLASS);
		}
		else
		{
			// 拨盘电机位置环
			DIAL_Angle_PidCalc(dia_motor_pid, DIAL_CLASS);
		}
	}
	else
	{
		// 拨盘电机位置环
		DIAL_Angle_PidCalc(dia_motor_pid, DIAL_CLASS);
	}
	// 拨盘电机输出响应
	DIAL_PidOut(dia_motor_pid);
}


/**
* @brief 拨盘打弹策略
*/
static void Shoot_Strategy(void)
{
	if(vision_sensor_info.RxPacket.RxData.identify_target)   //目标在视野内
	{
		if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) < PITCH_THRESHOLD) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) <  YAW_THRESHOLD))
		{
			dial.fire_open = true;    //开火
			shoot_freq_val = SHOOT_FREQ_TWELVE;
		}
		else if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) <  2) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) < 3))
		{
			dial.fire_open = true;    //开火
			shoot_freq_val = SHOOT_FREQ_HIGH;
		}
		else if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) <  1) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) < 1))
		{
			dial.fire_open = true;    //开火
			shoot_freq_val = SHOOT_FREQ_HEATLIMIT;
		}
		else    //目标没在开火阈值内
		{
			dial.fire_open = false;    //不开火
		}
	}
	else                //目标不在视野内
	{
		dial.fire_open = false;    //不开火
	}
//	if(gimbal.predict_open)  //预测开启
//	{
//		if(abs(vision.pitch_angle_corrected + vision.pitch_angle_offset) < SHOOT_THRESHOLD)  //在阈值内
//		{
//			dial.fire_open = true;	
//		}
//		else
//		{
//			dial.fire_open = false;
//		}
//	}
//	else if(!gimbal.predict_open)   //预测关闭
//	{
//		if(abs(vision.pitch_angle_corrected) < SHOOT_THRESHOLD)  //在阈值内
//		{
//			dial.fire_open = true;	
//		}
//		else
//		{
//			dial.fire_open = false;
//		}
//	}
}

uint8_t ACCESS = 0;
void DIAL_RcCtrl(void)
{
	dia_info.local_mode = DIAL_MODE_REPEATING;    //默认连发(通过改变射频来调整连发快慢，即进攻程度)
	
	dial.lock = false;        //拨盘解锁
	//以下配合遥控器测试专用 
	if(!dial.fric_unlock)  //下云台摩擦轮未解锁
	{
		dial.fric_open = false;  //关闭摩擦轮
	}
	else 
	{
		if(RC_THUMB_WHEEL_VALUE >= 600)
		{
			dial.fire_open = true;
			leader_info.TxPacket.ctrl_mode.fire_open = true;
		}
		else
		{
			dial.fire_open = false;
			leader_info.TxPacket.ctrl_mode.fire_open = false;
		}
		//遥控器下开关摩擦轮
		if(RC_SW1_VALUE == RC_SW_UP)
		{
			dial.fric_open = true;
		}
		else if(RC_SW1_VALUE == RC_SW_DOWN)
		{
			dial.fric_open = false;
		}
	}
}

void DIAL_KeyCtrl(void)
{

}

/**
* @brief 拨盘自动模式
* @param void
* @return void
*/
void DIAL_AutoCtrl(void)
{
	dia_info.local_mode = DIAL_MODE_REPEATING;    //默认连发(通过改变射频来调整连发快慢，即进攻程度)
	
	if(judge_sensor_info.GameStatus.game_progress == 4)      //比赛开始
	{
		dial.fric_open = true;        //开启摩擦轮
		fric_cnt++;          //计时加加
		if(fric_cnt < 2000)       
		{
			dial.fire_open = false;       //关闭拨盘
			dial.lock = true;          //拨盘锁定
		}
		else     //4s
		{
			dial.lock = false;        //拨盘解锁
			fric_cnt = 2001;
			//打弹策略
			Shoot_Strategy();
		}
	}
	else              //比赛还没开始
	{
		dial.lock = true;          //拨盘锁定
		dial.fric_open = false;        //关闭摩擦轮
		dial.fire_open = false;       //关闭拨盘
		fric_cnt = 0;              //计数清零
	}
	
}

/**
* @brief 拨盘自检模式
* @param void
* @return void
*/
void DIAL_InspectionCtrl(void)
{
	static bool access_one = false;
	static uint16_t unlock_time = 0;
	static bool unlock_flg = false;
	
	dia_info.local_mode = DIAL_MODE_REPEATING;    //默认连发(通过改变射频来调整连发快慢，即进攻程度)

	/*---测试用---*/
	dial.lock = false;        //拨盘解锁	

	if(!dial.fric_unlock)  //下云台摩擦轮未解锁
	{		
		if((RC_THUMB_WHEEL_VALUE <= -600) && access_one)
		{
			standard_flg = 1;
			access_one = false;
		}
		else if((RC_THUMB_WHEEL_VALUE >= 600) && access_one)
		{
			FRICTION_PwmOut(0, 0);  //电调解锁
			unlock_flg = true;   //解锁标志
			access_one = false;
		}
		else
		{
			access_one = true;			
		}
		
		if(unlock_flg)
		{
			unlock_time++;
			if(unlock_time > 500)  //1s后解锁摩擦轮
			{
				unlock_time = 501;
				dial.fric_unlock = true;  //解锁
			}
		}
	}
	else if(dial.fric_unlock)   //已解锁，可以进行打弹
	{
		//以下配合遥控器测试专用 
		if(RC_THUMB_WHEEL_VALUE >= 600)
		{
			dial.fire_open = true;
			leader_info.TxPacket.ctrl_mode.fire_open = true;
		}
		else
		{
			dial.fire_open = false;
			leader_info.TxPacket.ctrl_mode.fire_open = false;
		}		
	}
	/*---比赛用---*/
//	dial.fric_open = false;
//	dial.lock = true;          //拨盘锁定
//	dial.fire_open = false;       //关闭拨盘
}

/**
* @brief 枪管热量控制
* @param void
* @return void
* 用热量上限减去当前热量值得到剩余可用热量，
* 判断可用热量是否大于发射一发单位所需的热量来决定是否允许开枪
*/
void SHOOTER_HeatLimit(void)            
{
	static float Real_Heat_Error;
	static float Shooter_RealHeat;
	static float Shooter_LimitHeat;

	if(dia_info.local_mode == DIAL_MODE_REPEATING)       //如果连发热量差增加，防止扣血
	{
		Real_Heat_Error = One_Shot_17mm + 80;
	}
	else
	{
		Real_Heat_Error = One_Shot_17mm;
	}
	
	Shooter_RealHeat = judge_sensor_info.PowerHeatData.shooter_id1_17mm_cooling_heat;
	Shooter_LimitHeat = COOLING_LIMIT;
	
	if(Shooter_LimitHeat - Shooter_RealHeat < Real_Heat_Error)   //可用热量小于热量差，不允许发射
	{
		dial.fire_open = false;
	}
}

void DIAL_Ctrl(void)
{
	/*----信息读入----*/
	DIAL_GetInfo();
	/*----期望修改----*/ 
	if(dia_info.remote_mode == RC) {
		DIAL_RcCtrl();
	}
	else if(dia_info.remote_mode == AUTO) {
		DIAL_AutoCtrl();
	}
	else if(dia_info.remote_mode == INSPECTION) {
		DIAL_InspectionCtrl();
	}
	/*---云台手控制开火状态---*/
	Aerial_Ctrl();
	/*---枪管热量限制---*/
	SHOOTER_HeatLimit();	
	/*---拨盘卡弹检查---*/	
	StuckCheck();
	/*---射击模式选择---*/
	DIAL_ModeSelect();
	/*----最终输出----*/
	DIAL_PidCtrl();	
	Fric_Control();
}

void DIAL_Test(void)
{
    
}
/**
 *	@brief	拨盘正常模式（单发）
 */
void DIAL_NormoalShot(void)      
{
	if(!dia_info.normal_init_flag)
	{ //切换模式首先初始化一次
		dia_info.normal_init_flag = true;		
		dia_info.triple_init_flag = false;
		dia_info.repeating_init_flag = false;
		
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;				
		motor[DIAL].info->angle_sum = 0;

		dial.fire_open = false;
	}
	if(dial.fire_open)
	{
		dial.fire_open = false;
		dia_motor_pid[DIAL_CLASS].angle.target += SHOOT_SINGLE_ANGLE;
	}
}

/**
 *	@brief	拨盘三连发模式
 */
void DIAL_TripleShot(void)
{
	if(!dia_info.triple_init_flag)
	{ //切换模式首先初始化一次
		dia_info.triple_init_flag = true;
		dia_info.repeating_init_flag = false;
		dia_info.normal_init_flag = false;
		
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;						
		motor[DIAL].info->angle_sum = 0;

		dial.fire_open = false;
	}
	if(dial.fire_open)
	{
		dial.fire_open = false;
		dia_motor_pid[DIAL_CLASS].angle.target += SHOOT_TRIPLE_ANGLE;
	}
}

/**
 *	@brief	拨盘连发模式
 */
void DIAL_RepeatingShot(void)
{
	if(!dia_info.repeating_init_flag)
	{ //切换模式首先初始化一次
		dia_info.repeating_init_flag = true;
		dia_info.normal_init_flag = false;
		dia_info.triple_init_flag = false;
		dia_info.stuck_init_flag = false;
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;						
		motor[DIAL].info->angle_sum = 0;

//		dial.fire_open = false;
	}
	if(dial.fire_open)
	{
		dia_motor_pid[DIAL_CLASS].speed.target = shoot_freq_val;  //
	}
	else
	{
		dia_motor_pid[DIAL_CLASS].speed.target = 0;
	}
}

/**
 *	@brief	拨盘卡弹处理
 */
void DIAL_StuckHandle(void)      
{
	static uint16_t stuck_finish_cnt = 0;
	if(!dia_info.stuck_init_flag)
	{ //切换模式首先初始化一次
		dia_info.stuck_init_flag = true;		
		dia_info.triple_init_flag = false;
		dia_info.normal_init_flag = false;
		dia_info.repeating_init_flag = false;	
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;				
		motor[DIAL].info->angle_sum = 0;
	}
	if(dial.stuck && (stuck_finish_cnt == 0))
	{
		dia_motor_pid[DIAL_CLASS].angle.target -= SHOOT_SINGLE_ANGLE;
		
	}

	stuck_finish_cnt++;
	
	if(stuck_finish_cnt > STUCK_HANDLE_TIME)
	{
		stuck_finish_cnt = 0;
		dial.stuck = false;
	}
}

