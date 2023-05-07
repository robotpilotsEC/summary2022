/**
 * @file        chassis.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       Chassis Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"

#include "chassis.h"
#include "path_sensor.h"
#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"
#include "judge_sensor.h"

/* Private macro -------------------------------------------------------------*/
/*底盘位置环Kp*/
#define GRID_KP_LARGE 0.5
#define GRID_KP_NORMAL 0.5
#define GRID_KP_LOW 0.4
/*跑轨额外给定目标*/
#define NORMAL_GIVEN_MIN -4000
#define NORMAL_GIVEN_MAX 4000
#define GRID_GIVEN_MIN -4000
#define GRID_GIVEN_MAX 4000


/* Private function prototypes -----------------------------------------------*/
void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);
void CHASSIS_PowerLimit(void);
float AcceleratedSpeed_Cal(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 底盘电机本地驱动
drv_can_t				*chas_drv[CHASSIS_MOTOR_CNT];
motor_t			*chas_motor[CHASSIS_MOTOR_CNT];
motor_info_t	*chas_motor_info[CHASSIS_MOTOR_CNT];

bool flag_grid = true;   //坐标初始化标志
bool flag_normal = true;    //正常模式初始化
bool normal_init = true;    //第一次进入正常模式
bool access_encode = false;  //第一次进入测试编码器移动模式
float chassis_power_factor = 1;
//用来判断目标是否发生改变以刹车
//float angle_target_pre;
//float angle_target_now;
int32_t random_value = 0;


//底盘加速度
float chassis_accelerate = 0;

/* Exported variables --------------------------------------------------------*/
// 底盘电机PID控制器
chassis_motor_pid_t 	chas_motor_pid[CHASSIS_MOTOR_CNT] = {
	[CHASSIS_CLASS] = {
		.speed.kp = 11.75,    //11.75
		.speed.ki = 0.472,    //0.472
		.speed.kd = 0, 
		.speed.integral_max = 10000,
		.speed.out_max = 18000,
		.angle.kp = 0.45,   //0.45   0.6  0.7  0,8
		.angle.ki = 0,
		.angle.kd = 0.008,   //0.008
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	},
	
};

// 底盘模块控制器
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
};

// 底盘模块传感器
chassis_dev_t		chas_dev = {
	.chas_motor[CHASSIS_CLASS] = &motor[CHASSIS],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 底盘模块信息
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.local_mode = RUN_NORMAL,
};

chassis_t chassis = {
	.controller = &chas_ctrl,
	.dev = &chas_dev,
	.info = &chas_info,
	.init = CHASSIS_Init,
	.test = CHASSIS_Test,
	.ctrl = CHASSIS_Ctrl,
	.self_protect = CHASSIS_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
/**
 *	@brief	底盘电机PID参数初始化
 */
void CHASSIS_PidParamsInit(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid_val_init(&pid[i].angle);
		pid[i].out = 0;
	}
}

/**
 *	@brief	底盘全部电机卸力
 */
static void CHASSIS_Stop(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<CHASSIS_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
	}
}




/**
 *	@brief	底盘电机PID输出
 */
static void CHASSIS_PidOut(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<CHASSIS_MOTOR_CNT; i++) {
		if(chas_motor[i]->work_state == DEV_ONLINE) {
            chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
		} 
		else {
            chas_drv[i]->add_halfword(chas_drv[i], 0);
		}
	}    
}

/**
 *	@brief	底盘电机速度环
 */
static void CHASSIS_Speed_PidCalc(chassis_motor_pid_t *pid, chassis_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	底盘电机位置环
 */
static void CHASSIS_Angle_PidCalc(chassis_motor_pid_t *pid, chassis_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	single_pid_ctrl(&pid[MOTORx].angle);	
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	CHASSIS_Speed_PidCalc(pid,MOTORx);
}


/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
/**
 *	@brief	底盘获取系统信息
 */
void CHASSIS_GetSysInfo(void)
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		chas_info.remote_mode = RC;
	}
//	else if(sys.remote_mode == KEY) {
//		chas_info.remote_mode = KEY;
//	}      //后面再定夺要不要使用键盘控制底盘
	else if(sys.remote_mode == AUTO)
	{
		chas_info.remote_mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) 
	{
		chas_info.remote_mode = INSPECTION;
	}
	/*----本地模式修改----*/
}

/**
 *	@brief	云台手信息获取
 */
void CHASSIS_GetAerialInfo(void)
{
	if(judge_sensor.info->GameRobotStatus.robot_id == 7)//红色id
	{
		if((judge_sensor.info->AerialData.send_id == 6) && (judge_sensor.info->AerialData.receive_id == 7))  //无人机发送给哨兵
		{
			if(judge_sensor.info->AerialData.cmd == escape)
			{
				chassis.escape = true;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == check_road)
			{
				chassis.escape = false;
				chassis.check_road = true;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == control_aerial)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = true;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == test_sentry)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = true;

			}
			else
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			chassis.control_dir = judge_sensor.info->AerialData.control_dir;
		}
	}
	else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//蓝色id
	{
		if((judge_sensor.info->AerialData.send_id == 106) && (judge_sensor.info->AerialData.receive_id == 107))  //无人机发送给哨兵
		{
			if(judge_sensor.info->AerialData.cmd == escape)
			{
				chassis.escape = true;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == check_road)
			{
				chassis.escape = false;
				chassis.check_road = true;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == control_aerial)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = true;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == test_sentry)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = true;
			}
			else
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			chassis.control_dir = judge_sensor.info->AerialData.control_dir;
		}
	}
}

void CHASSIS_UpdateController(void)
{
	static uint8_t dir_cnt_l = 0;
	static uint8_t dir_cnt_r = 0;
	
	chas_motor_pid[CHASSIS_CLASS].speed.measure = chas_motor_info[CHASSIS_CLASS]->speed;
	chas_motor_pid[CHASSIS_CLASS].angle.measure = path_sensor.info->mileage_total;	 //用编码器的值作为外环
	//更新运动方向
	if(chas_motor_pid[CHASSIS_CLASS].speed.measure < 0)
	{
		dir_cnt_r = 0;
		dir_cnt_l++;
		if(dir_cnt_l == 10)
		{
			chassis.run_dir = LEFT;
			dir_cnt_l = 0;
		}
	}
	else if(chas_motor_pid[CHASSIS_CLASS].speed.measure > 0)
	{
		dir_cnt_l = 0;
		dir_cnt_r++;
		if(dir_cnt_r == 10)
		{
			chassis.run_dir = RIGHT;
			dir_cnt_r = 0;
		}	
	}
	else
	{
		dir_cnt_l = 0;
		dir_cnt_r = 0;
	}
	
//	chassis_accelerate = AcceleratedSpeed_Cal();  //计算加速度
	
}

/* 应用层 --------------------------------------------------------------------*/
/* 任务层 --------------------------------------------------------------------*/
void CHASSIS_Init(void)
{
	chas_drv[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS]->driver;

	chas_motor[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS];
	
	chas_motor_info[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS]->info;
	
	chassis.init_mileage = true;
	chassis.rail_mileage = 0;
	chassis.direction = LEFT;
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;         //速度环模式
	/*---云台手控制---*/
	chassis.check_road = false;
	chassis.control_dir = 0;
	chassis.ctrl_sentry = false;
	chassis.escape = false;
	chassis.test_sentry = false;
	/*---伤害更新---*/
	chassis.hurt_data_update = false;
	/*---底盘卸力状态---*/
	chassis.unloading_force = false;
}

void CHASSIS_GetInfo(void)
{
	CHASSIS_GetSysInfo();
	CHASSIS_GetAerialInfo();
	CHASSIS_UpdateController();
}

void CHASSIS_SelfProtect(void)
{
//	chassis_accelerate = AcceleratedSpeed_Cal();  //计算加速度
	CHASSIS_Stop(chas_motor_pid);
	CHASSIS_PidParamsInit(chas_motor_pid, CHASSIS_MOTOR_CNT);
	CHASSIS_GetInfo();
}

void CHASSIS_PidCtrl(void)
{

	switch(chas_motor_pid[CHASSIS_CLASS].mode)
	{
			// 底盘电机速度环
		case SPEED_PID:	CHASSIS_Speed_PidCalc(chas_motor_pid, CHASSIS_CLASS); break;
			// 底盘电机位置环	
		case POSITION_PID: CHASSIS_Angle_PidCalc(chas_motor_pid, CHASSIS_CLASS);break;
	}	


	//底盘电机卸力情况
	if(chassis.unloading_force == true)
	{
		chas_motor_pid[CHASSIS_CLASS].out = 0;
	}	
	// 底盘电机输出响应
	CHASSIS_PidOut(chas_motor_pid);
}

/**
* @brief 底盘加速度差计算
*/
float AcceleratedSpeed_Cal(void)
{
	static int16_t pre_chassis_speed;
	static int16_t now_chassis_speed;
	static float accelerated_speed;
	static uint8_t time_interval = 0;  //计算时间间隔 /ms
	
	if(time_interval == 0)   
	{
		now_chassis_speed = motor[CHASSIS].info->speed;
		pre_chassis_speed = now_chassis_speed;
	}
	else if(time_interval == 5)  //10ms
	{
		time_interval = 0;
		now_chassis_speed = motor[CHASSIS].info->speed;		
		accelerated_speed = (now_chassis_speed - pre_chassis_speed) *1.0 / 10;
	}
	
	time_interval++;
//	if(time_interval >= 5)
//	{
//		time_interval = 0;
//		now_chassis_speed = path_info->mileage_total;
//		
//	}
	return accelerated_speed;
}


/*---底盘运动策略---*/
/**
* @brief 初始巡航
* @param void
* @return void
* 用于第一遍跑轨时对轨道长度的校定，跑轨速度较慢，一般在3分钟准备中完成
*/
static void Cruise_First()
{
		pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;
    static uint8_t step = 0;	/*初始化步骤*/
    if(chassis.init_mileage)		/*未对里程完成初始化*/
    {
			chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //切换到速度环模式
      switch(step)
      {
        case 0:			/*初始启动先向左边靠*/
        {
          if(path_sensor.info->left_touch)		/*左边的点触开关*/
          {
            Speed->target = 0;
            step = 1;
          }
          else	/*未识别到 -- 向左运动*/
          {
						chassis.direction = LEFT;
            Speed->target = INITIAL_SPEED * chassis.direction;	/*向左运动*/
          }
          break;
        }
        case 1:
        {
					if(path_sensor.info->left_touch)
          {
						chassis.direction = RIGHT;
            Speed->target = 100 * chassis.direction;	/*向右微调，调节到刚好不触发点触开关的位置*/
          }
          else			/*微调完成后先停住，进入下一个调节阶段*/
          {
            Speed->target = 0;
            path_sensor.info->mileage_total = 0;	/*清除里程数 -- 开始记录*/
            step = 2;
          }
          break;
        }
        case 2:
        {
          if(path_sensor.info->right_touch)	/*已经识别到右轨道*/
          {
            Speed->target = 0;
            step = 3;
          }
          else 		/*未识别到*/
          {
						chassis.direction = RIGHT;
            Speed->target = INITIAL_SPEED * chassis.direction;			/*向右运动*/
          }
          break;
        }
        case 3:
        {
          if(path_sensor.info->right_touch)
          {
						chassis.direction = LEFT;	
						Speed->target = 100 * chassis.direction;		/*已经识别到右点触开关，现在向左微调到刚好未识别到的状态*/
          }
          else			/*微调完成*/
          {
						Speed->target = 0;															/*停止运动*/
            chassis.rail_mileage = path_sensor.info->mileage_total;	/*记录总里程数*/
            step = 0;																							/*复位step*/
            chassis.init_mileage = false;											/*更新标志位*/
            chassis.direction = LEFT;                        //向左运动标志位
//                Chassis_process.Spot_taget = 0;//位置环
//                Chassis_process.getchange_flag = true;
          }
          break;
        }
      }
    }
}

/**
* @brief 底盘跑轨模式-静止
* @param void
* @return void
*/
void CHASSIS_Static(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //切换到位置环模式
  
	Grid->target = chassis.rail_mileage *3.0/ 4;    //停在右端
	

	if(!flag_grid)
	{
		flag_grid = true;  //更新坐标模式初始化标志
	}

	if(!flag_normal)
	{
		flag_normal = true;  //更新正常模式初始化标志
	}
	
  //调整位置环Kp
	Grid->kp = GRID_KP_NORMAL;  //Kp = 0.45
}

/**
* @brief 底盘跑轨模式-查看公路
* @param void
* @return void
*/
void CHASSIS_CheckRoad(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //切换到位置环模式

	Grid->target = chassis.rail_mileage / 4;    //停在左端
	


	if(!flag_grid)
	{
		flag_grid = true;  //更新坐标模式初始化标志
	}

	if(!flag_normal)
	{
		flag_normal = true;  //更新正常模式初始化标志
	}
	
  //调整位置环Kp
	Grid->kp = GRID_KP_NORMAL;  //Kp = 0.45

}

/**
* @brief 底盘跑轨模式-云台手操控
* @param void
* @return void
*/
void CHASSIS_ControlDir(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //切换到速度环模式
	
	if(chassis.control_dir == LEFT)
	{
		Speed->target -= 5;
	}
	else if(chassis.control_dir == RIGHT)
	{
		Speed->target += 5;
	}
		//限幅
	Speed->target = constrain(Speed->target, -8000, 8000);



	if(!flag_grid)
	{
		flag_grid = true;  //更新坐标模式初始化标志
		Speed->target = 0;

	}

	if(!flag_normal)
	{
		flag_normal = true;  //更新正常模式初始化标志
		Speed->target = 0;
	}
	
  //调整位置环Kp
	Grid->kp = GRID_KP_NORMAL;  //Kp = 0.45

}

/**
* @brief 底盘坐标随机值生成
*/
void RandomValue_Generate(void)
{
//  path_info_t *path_info = path_sensor.info;
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	static int32_t target_buffer;

//	target_buffer = rand() % (chassis.rail_mileage);   //更新目标(随机)  0 ~ chassis.rail_mileage

	target_buffer = GRID_GIVEN_MIN + rand() % (chassis.rail_mileage + (GRID_GIVEN_MAX * 2));   //更新目标(随机)  -3000 ~ chassis.rail_mileage + 3000

	//来回方向目标生成
//	if(path_info->mileage_dif > 0)
//	{
//		if(((target_buffer - Grid->target) < -(chassis.rail_mileage / 3)) )    //判断生成的随机数有没有大于阈值
//		{
//			random_value = target_buffer;     //记录下一次目标值
//		}	
//		
//	}
//	else if(path_info->mileage_dif < 0)
//	{
//		if(((target_buffer - Grid->target) > (chassis.rail_mileage / 3)))    //判断生成的随机数有没有大于阈值
//		{
//			random_value = target_buffer;     //记录下一次目标值
//		}		
//	}
//	else
//	{
//		if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 3)))    //判断生成的随机数有没有大于阈值
//		{
//			random_value = target_buffer;     //记录下一次目标值
//		}	
//	}
  //全方向随机
	if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 3)))    //判断生成的随机数有没有大于阈值
	{
		random_value = target_buffer;     //记录下一次目标值
	}	

}



/**
* @brief 底盘安全状态判断
* @param void
* @return void
*/
void IS_Safety_Status(void)
{
	static uint16_t Cur_HP;          //当前血量
	static uint16_t Pre_HP = 600;     //初始血量为最大血量，600     
	static uint16_t Deducted_HP = 0;    //已扣血量
	static uint16_t HP_Cnt = 0;	    //血量检测计时
	static uint16_t security_delay = 0;      //恢复到安全状态的延迟
	static uint16_t Pre_HP1 = 600;   //初始血量为最大血量，600 
	
	Cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;  //更新当前血量
	
	if((chassis.hurt_data_update)&&(judge_sensor.info->RobotHurt.hurt_type == 0))   //受到伤害
	{
		chassis.hurt_data_update = false;  //重置标志位
		if(Pre_HP1 - Cur_HP >= 100)     //被打弹丸击中
		{
			chassis.safety_status = HUGE_DANGER;
		}
		else
		{
			chassis.safety_status = DANGER;
		}
		security_delay = 0;
	}
	else          //没受到伤害,3s过后若没受到伤害则恢复安全状态
	{
		if(chassis.safety_status != SAFETY)
		{
			security_delay++;
			if(security_delay == 3000)       //6s
			{
				security_delay = 0;        //计时清零
				chassis.safety_status = SAFETY;
			}
		}
	}
	
	Pre_HP1 = Cur_HP;
	
	HP_Cnt++;
	if(HP_Cnt == 1500)     //每3s检查一次
	{
		Deducted_HP = Pre_HP - Cur_HP;  //计算已扣血量
		if(Deducted_HP >= 100)        //3s内扣了100血以上
		{
			chassis.safety_status = HUGE_DANGER;	
		}
		else if(Deducted_HP >= 50)   //3s内扣了50血
		{
			chassis.safety_status = DANGER;			
		}
		HP_Cnt = 0;
	}
	Pre_HP = Cur_HP;
}

/**
* @brief 底盘跑轨模式-正常
* @param void
* @return void
*/
void CHASSIS_Normal(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //切换到位置环模式

	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

//	if((abs(Grid->target - Grid->measure)) <= 5000)
//	{
//		if(abs((chassis.rail_mileage + 3000) - Grid->measure) > abs((-3000 - Grid->measure)))	 //若底盘方向是右
//		{
//			Grid->target = chassis.rail_mileage + 3000;
//		}
//		else if(abs((chassis.rail_mileage + 3000) - Grid->measure) <= abs((-3000 - Grid->measure)))	 //若底盘方向是左
//		{
//			Grid->target = -3000;
//		}
//	}
	
	if(chassis.direction == LEFT)
	{
		Grid->target = NORMAL_GIVEN_MIN;
	}
	else if(chassis.direction == RIGHT)
	{
		Grid->target = chassis.rail_mileage + NORMAL_GIVEN_MAX;		
	}
	//限幅
	Grid->target = constrain(Grid->target, NORMAL_GIVEN_MIN, chassis.rail_mileage + NORMAL_GIVEN_MAX);


	if(!flag_grid)
	{
		flag_grid = true;  //更新坐标初始化标志,为下一次切换坐标模式做准备
	}
	
  //不用刹车,调整位置环Kp
	Grid->kp = GRID_KP_LOW;  //Kp = 0.4

}


/**
* @brief 底盘跑轨模式-坐标(备用)
* @param void
* @return void
*/
void CHASSIS_Coordinate(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //切换到位置环模式
	path_info_t *path_info = path_sensor.info;
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
//	static int32_t target_buffer;
	static int32_t next_target;

	
	if(flag_grid)  //若未进行坐标初始化
	{
		flag_grid = false;
		Grid->target = Grid->measure;
	}
	
//	target_buffer = GRID_GIVEN_MIN + rand() % (chassis.rail_mileage + (GRID_GIVEN_MAX * 2));   //更新目标(随机)  -3000 ~ chassis.rail_mileage + 3000
	
////单方向随机值
//	if(chassis.direction == LEFT)
//	{
//		
//	}
//	else if(chassis.direction == RIGHT)
//	{
//		
//	}
//	//全方向随机值
//	if(abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 4))    //判断生成的随机数有没有大于阈值
//	{
//		next_target = target_buffer;     //记录下一次目标值
//		if(abs(Grid->measure - Grid->target) < 4000)   //进入达到目标的死区 
//		{
//			Grid->target = next_target; 
//			//限幅
//			Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);
//		}
//		else if(path_info->left_touch || path_info->right_touch)   //碰到轨道两端
//		{
//			Grid->target = next_target; 
//			//限幅
//			Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);			
//		}
//	}

	//全方向随机值
	next_target = random_value;     //记录下一次目标值
	if(abs(Grid->measure - Grid->target) < 4000)   //进入达到目标的死区 
	{
		Grid->target = next_target; 
		//限幅
		Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);
	}
	else if(path_info->left_touch || path_info->right_touch)   //碰到轨道两端
	{
		Grid->target = next_target; 
		//限幅
		Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);			
	}

  //不用刹车,调整位置环Kp
	Grid->kp = GRID_KP_LOW;  //Kp = 0.4

}


/**
* @brief 底盘跑轨模式-混合(备用)
* @note  正常跑轨加编码器跑轨
* @param void
* @return void
*/
void CHASSIS_Mixed(void)
{
	static uint16_t time_cnt = 0;
	static uint16_t switch_mode = 0;
	uint16_t tempt = 1500 + rand() % (2500 + 1);
	
	time_cnt++;
		
	if(time_cnt < tempt)
	{
		switch(switch_mode)
		{
			case 0: CHASSIS_Normal(); break;
			case 1: CHASSIS_Coordinate(); break;
		}	
	}
	else
	{
		time_cnt = 0;
		tempt = 1500 + rand() % (2500 + 1);   //生成一个1500~2500的随机数
		
		if(switch_mode)
		{
			switch_mode = 0;
		}
		else
		{
			switch_mode = 1;
		}
	}
}


/**
* @brief 底盘自动跑轨模式
* @param void
* @return void
*/
static void CHASSIS_AutoRun(void)
{	
// /*测试用*/
//	switch(RC_SW1_VALUE)
//	{
//		case RC_SW_UP: chas_info.local_mode = RUN_MIXED; break;
//		case RC_SW_MID: chas_info.local_mode = RUN_NORMAL; break;
//		case RC_SW_DOWN: chas_info.local_mode = RUN_COORDINATE; break;
//	}
	
	 /*-------测试用-------*/
	 
//	/*---期望修改---*/
//	switch(chas_info.local_mode)
//	{
//		/*---正常模式巡线,即来回撞柱跑动---*/
//		case RUN_NORMAL:
//		{
//			CHASSIS_Normal(); 
//		}break;
//		/*---坐标模式跑轨---*/
//		case RUN_COORDINATE:
//		{
//			CHASSIS_Coordinate();
//		}break;
//		/*---混合模式跑轨---*/		
//		case RUN_MIXED:
//		{
//			CHASSIS_Mixed();
//		}break;
//	}
// /**************************************/	

 /*比赛时候需注释(比赛用) */ 
	
	/*--- 根据血量情况选择对应跑轨模式(比赛用) ---*/
	IS_Safety_Status();       //安全状态判断
	
	switch(chassis.safety_status)   //根据安全状态选择对应的模式
	{
		case SAFETY: chas_info.local_mode = RUN_COORDINATE; break;    
		case DANGER: chas_info.local_mode = RUN_MIXED; break;
		case HUGE_DANGER: chas_info.local_mode = RUN_MIXED; break;	
	}
	/* 云台手控制 */
	if(chassis.check_road)
	{
		CHASSIS_CheckRoad();
	}
	else if(chassis.test_sentry)   //测试，适应性训练用
	{
		CHASSIS_Coordinate();
	}
	else if(chassis.ctrl_sentry)
	{
		CHASSIS_ControlDir();
	}	
	else if(chassis.escape)
	{
		CHASSIS_Mixed();
	}
	 /* 云台手不控制 */
	else 
	{
		if(judge_sensor.info->EventData.outpost == 1)        // 前哨战存活
		{
			CHASSIS_Static();         //停留在右端
		}
		else if((RC_THUMB_WHEEL_VALUE >= 50) && (RC_THUMB_WHEEL_VALUE < 600))      //训练时模拟用拨轮模拟前哨战状态，比赛时需要注释
		{
			CHASSIS_Static();         //停留在右端
		}
		else if(RC_THUMB_WHEEL_VALUE >= 600)      //比赛用，用于测试随机跑轨
		{
			CHASSIS_Coordinate();
		}
		else if(judge_sensor.info->EventData.outpost == 0)             //前哨战被击毁
		{
			switch(chas_info.local_mode)
			{
				/*---正常模式巡线,即来回撞柱跑动---*/
				case RUN_NORMAL:
				{
					CHASSIS_Normal(); 
				}break;
				/*---坐标模式跑轨---*/
				case RUN_COORDINATE:
				{
					CHASSIS_Coordinate();
				}break;
				/*---混合模式跑轨---*/		
				case RUN_MIXED:
				{
					CHASSIS_Mixed();
				}break;
			}
		}
	}
 /**************************************/	

}

/**
* @brief 底盘撞柱判断
* @param void
* @return void
*/
void CHASSIS_ImpactEnd(void)
{
	path_info_t *path_info = path_sensor.info;
	static uint8_t stop_access = 1;
	
	//撞柱刹车判断 
	if(path_info->left_touch && stop_access)   //到达左端点        
	{	
		stop_access = 0;
		chassis.direction = RIGHT;
		chassis.unloading_force = true;        //卸力
	}
	else if(path_info->right_touch && stop_access)   //到达右端点
	{
		stop_access = 0;
		chassis.direction = LEFT;
		chassis.unloading_force = true;        //卸力
	}
	else if((!path_info->left_touch) && (!path_info->right_touch))//没有在端点
	{
		stop_access = 1;
		chassis.unloading_force = false;        
	}
}






/**
* @brief 底盘遥控模式
* @param void
* @return void
*/
void CHASSIS_RcCtrl(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //切换到速度环模式
	chas_motor_pid[CHASSIS_CLASS].speed.target = RampFloat(rc_sensor.info->ch2 / 660.f * 9000, chas_motor_pid[CHASSIS_CLASS].speed.target, 20);


	access_encode = true;
	
	if(!flag_grid)
	{
		flag_grid = true;  //更新坐标模式初始化标志
	}

	if(!flag_normal)
	{
		flag_normal = true;  //更新正常模式初始化标志
	}
	
}

void CHASSIS_KeyCtrl(void)
{

}

/**
* @brief 底盘自动模式
* @param void
* @return void
*/
void CHASSIS_AutoCtrl(void)
{	
	pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;

	access_encode = true;
	
	if(RC_SW1_VALUE == RC_SW_DOWN)   //光电或编码器坏了就保持底盘不动
	{
		chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //切换到速度环模式
		Speed->target = 0;
		chassis.init_mileage = true;
	}
	else     //正常运动
	{
		/*---第一遍跑轨初始化---*/	
		Cruise_First();
		
		if(chassis.init_mileage == false)  //巡航初始化完毕
		{
			/*---产生随机数---*/						
			RandomValue_Generate();
			/*---自动跑轨---*/		
			CHASSIS_AutoRun();	
			/*---撞柱判断---*/			
			CHASSIS_ImpactEnd();
		}
	}
}

/**
* @brief 底盘自检模式
* @param void
* @return void
*/
void CHASSIS_InspectionCtrl(void)
{
	static bool left_access = false;
	static bool right_access = false;
	
  path_info_t *path_info = path_sensor.info;
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //切换到位置环模式
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

	if(RC_SW1_VALUE == RC_SW_DOWN)
	{
		if(access_encode)
		{
			Grid->target = Grid->measure;
			access_encode = false;
		}
		Grid->target += rc_sensor.info->ch2 / 6.f;

		 //光电开关检查
		if(path_info->left_touch)
		{
			if(left_access)
			{
				left_access = false;
				Grid->target += 3000;
			}
		}
		else if(path_info->right_touch)
		{
			if(right_access)
			{
				right_access = false;
				Grid->target += -3000;
			}
		}
		else
		{
			left_access = true;
			right_access = true;
		}
	}
	else
	{
		access_encode = true;
	}
//	brake_step = 0;
//	chassis.brake_check = false;
//	chassis.brake_finished = true;  
//	chassis.brake_start = false;

	if(!flag_grid)
	{
		flag_grid = true;  //更新坐标模式初始化标志
	}

	if(!flag_normal)
	{
		flag_normal = true;  //更新正常模式初始化标志
	}

	
}




void CHASSIS_Ctrl(void)
{
	/*----信息读入----*/
	CHASSIS_GetInfo();
	/*----期望修改----*/
	if(chas_info.remote_mode == RC) {
		CHASSIS_RcCtrl();
	}
	else if(chas_info.remote_mode == KEY) {
		CHASSIS_KeyCtrl();
	}
	else if(chas_info.remote_mode == AUTO) {
		CHASSIS_AutoCtrl();
	}
	else if(chas_info.remote_mode == INSPECTION) {
		CHASSIS_InspectionCtrl();
	}
	/*---底盘功率限制---*/
	CHASSIS_PowerLimit();
	
	/*----最终输出----*/
	CHASSIS_PidCtrl();	
}

void CHASSIS_Test(void)
{
    
}

/**
* @brief 底盘功率控制
* @param void
* @return void
* PID最大输出 = (chassis_power_factor )^2 *  Speed->out_max, 其中系数为 ：剩余缓冲能量/60J
*/
void CHASSIS_PowerLimit(void)
{
	uint16_t Real_PowerBuffer;
	pid_ctrl_t *Speed  = &chas_motor_pid[CHASSIS_CLASS].speed;
	
		 //功率算法
	switch(judge_sensor.work_state)
	{
		case DEV_ONLINE: 
		{
			if(judge_sensor.info->GameStatus.game_progress == 1)   //准备阶段
			{
				chassis_power_factor = 0.3;
			}
			else
			{
				Real_PowerBuffer = judge_sensor.info->PowerHeatData.chassis_power_buffer;
				if(Real_PowerBuffer < 60)
				{
					chassis_power_factor = Real_PowerBuffer / 60.f; //0-1
				}
				else
				{
					chassis_power_factor = 1;  //1	
				}					
			}
		}break;
	
		case DEV_OFFLINE:
		{
			chassis_power_factor = 0.5;
		}break;	
	}
	
	Speed->out_max = (chassis_power_factor) * 18000.f;		
}
