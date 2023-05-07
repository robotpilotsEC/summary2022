/**
 * @file        turnplate.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        3-November-2020
 * @brief       Turnplate Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "turnplate.h"

#include "can_potocol.h"
#include "fricwheel.h"
#include "rp_math.h"
#include "cmsis_os.h"
#include "drv_can.h"
#include "kalman.h"
#include "kalman_filter.h"

/* Private macro -------------------------------------------------------------*/
#define TURNPLATE_SPEED_RPS		2160
#define TURNPLATE_GRID			8.0f
#define TURNPLATE_BULLET_RPS	(TURNPLATE_SPEED_RPS/TURNPLATE_GRID)
#define TURNPLATE_BULLET_ANGLE	(8192*36/TURNPLATE_GRID) //24576.0f	// 8192*36/12=24576.0f
#define TURNPLATE_BULLET_RAMP 	(TURNPLATE_BULLET_ANGLE/10)

/* Private function prototypes -----------------------------------------------*/
void TURNPLATE_Init(void);
void TURNPLATE_SelfProtect(void);
void TURNPLATE_Ctrl(void);
void TURNPLATE_Test(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 拨盘电机本地驱动
static drv_can_t				*tplt_drv;
static turnplate_motor_info_t	*tplt_motor_info;

// 遥控按键信息
static switch_t         SW1;
static switch_flip_t    SW1_TRIG;
static switch_t         SW2;
static switch_flip_t    SW2_TRIG;
static button_t         MOUSE_L;
static button_flip_t    MOUSE_L_TRIG;
static uint32_t         MOUSE_L_TIME;

extKalman_t  tplt_speed_kalman;

/* Exported variables --------------------------------------------------------*/
// 拨盘电机PID控制器
turnplate_pid_t 	tplt_pid = {
	// 角度环
	.angle.kp = 0.32,   // 0.60 0.5
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.out_max = 6480,	// 18800PJ8格小拨盘速度闭环时最大转速 => 理论极限射频8.7rps	
    .angle.loop_time = 0.002, // 2ms  
	// 速度环
	.speed.kp = 8.75,   // 10.5 9.6
	.speed.ki = 160,   // 90   0.16
	.speed.kd = 0,
	.speed.integral_max = 8000,
	.speed.out_max = 10000,
    .speed.loop_time = 0.002, // 2ms  
};

// 射击模块控制器
turnplate_ctrl_t		tplt_ctrl = {
	.tplt = &tplt_pid,
};

// 射击模块传感器
turnplate_dev_t			tplt_dev = {
	.turnplate_motor = &turnplate_motor,
	.rc_sensor = &rc_sensor,
};

// 射击模块信息
turnplate_info_t		tplt_info = {
	.remote_mode = RC,
	.local_mode = TURNPLATE_MODE_NORMAL,
	.pid_mode = TPLT_POSI_PID,
	.state = TPLT_STATE_OFF,
};

// 射击模块
turnplate_t	turnplate = {
	.controller = &tplt_ctrl,
	.dev = &tplt_dev,
	.info = &tplt_info,
	.test_open = false,
	.init = TURNPLATE_Init,
	.test = TURNPLATE_Test,
	.ctrl = TURNPLATE_Ctrl,
	.self_protect = TURNPLATE_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
static void TURNPLATE_PidParamsInit(turnplate_pid_t *pid)
{
	pid_val_init(&pid->angle);
	pid_val_init(&pid->speed);
	pid->out = 0;
}

static void TURNPLATE_Stop(turnplate_pid_t *pid)
{
	pid->speed.out = 0;
	pid->out = 0;

    tplt_drv->add_tx_msg(tplt_drv, pid->out);
    tplt_drv->start_tx(tplt_drv);
}

static void TURNPLATE_PidOut(turnplate_pid_t *pid)
{
    if(tplt_dev.turnplate_motor->work_state == DEV_ONLINE)
        tplt_drv->add_tx_msg(tplt_drv, pid->out);
    else
        tplt_drv->add_tx_msg(tplt_drv, 0);

    tplt_drv->start_tx(tplt_drv);
}

float js_tplt_speed_target;
float js_tplt_speed_measure;
float js_tplt_speed_integral;
float js_tplt_speed_out;
static void TURNPLATE_Speed_PidCalc(turnplate_pid_t *pid)
{
	pid->speed.err = pid->speed.target - pid->speed.measure;
	single_pid_ctrl(&pid->speed);
	pid->out = pid->speed.out;
    
    js_tplt_speed_target = pid->speed.target;
    js_tplt_speed_measure = pid->speed.measure;
    js_tplt_speed_integral = pid->speed.integral;
    js_tplt_speed_out = pid->speed.out;
}

float js_tplt_angle_target;
float js_tplt_angle_measure;
static void TURNPLATE_Angle_PidCalc(turnplate_pid_t *pid)
{
    pid->angle.err = pid->angle.target - pid->angle.measure;
	single_pid_ctrl(&pid->angle);
    
    js_tplt_angle_target = pid->angle.target;
    js_tplt_angle_measure = pid->angle.measure;
}

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
void TURNPLATE_GetSysInfo(void)
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC)
		tplt_info.remote_mode = RC;
	else if(sys.remote_mode == KEY)
		tplt_info.remote_mode = KEY;
	
	/*----本地模式修改----*/
	switch(sys.mode)
	{
		case SYS_MODE_NORMAL:
		{
			tplt_info.local_mode = TURNPLATE_MODE_NORMAL;
		}break;
		case SYS_MODE_AUTO:
		{
			tplt_info.local_mode = TURNPLATE_MODE_AUTO;
		}break;
		default:
		{
		}break;
	}
}
	
void TURNPLATE_GetJudgeInfo(void)
{
}

void TURNPLATE_GetRcInfo(void)
{
    rc_sensor_t *rc_sen = tplt_dev.rc_sensor;
    
	/* 系统正常 */
	if(sys.state == SYS_STATE_NORMAL)
	{
		if(sys.remote_mode == RC)
			MOUSE_L_TIME = 0;
	}
	/* 系统异常 */
	else
	{
		MOUSE_L_TIME = 0;
	}
    
    SW1_TRIG = rc_sen->copy_switch(&SW1, &rc_sen->info->SW1);
    SW2_TRIG = rc_sen->copy_switch(&SW2, &rc_sen->info->SW2);
    MOUSE_L_TRIG = rc_sen->copy_button(&MOUSE_L, &rc_sen->info->MOUSE_L);
    
    if(SW2.state == RC_SW_UP) {
        /* 位置环->速度环 */
		if(SW2_TRIG == SW_MID_TO_DOWN) {
			tplt_pid.speed.target = 0;
		}
        tplt_info.pid_mode = TPLT_SPEED_PID;
    }
    else if(SW2.state == RC_SW_MID) {
        /* 速度环->位置环 */
        if(SW2_TRIG == SW_DOWN_TO_MID) {
            tplt_pid.angle.target = tplt_pid.angle.measure;
			tplt_pid.angle_ramp_target = tplt_pid.angle.target;
        }
        tplt_info.pid_mode = TPLT_POSI_PID;
    }
    else if(SW2.state == RC_SW_DOWN) {
        /* 位置环->速度环 */
		if(SW2_TRIG == SW_MID_TO_DOWN) {
			tplt_pid.speed.target = 0;
		}
        tplt_info.pid_mode = TPLT_SPEED_PID;
    }
}

void TURNPLATE_UpdateController(void)
{
	tplt_pid.angle.measure = tplt_motor_info->angle_sum;
	tplt_pid.speed.measure = KalmanFilter(&tplt_speed_kalman, tplt_motor_info->speed);
}

/* 应用层 --------------------------------------------------------------------*/
#define BULLET_STUCK_PID_OUT	4000
#define BULLET_STUCK_SPEED		60
#define BULLET_TURNBACK_SPEED	(-4000)
#define BULLET_STUCK_TIME		100
#define BULLET_TURNBACK_TIME	100

void TURNPLATE_Speed_BulletStuck(void)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint16_t stuck_flag = false;
	
	if(stuck_flag == false)
	{
		if(tplt_pid.speed.out > BULLET_STUCK_PID_OUT
		   && tplt_pid.speed.measure < BULLET_STUCK_SPEED)
		{
			stuck_time++;
		} else {
			stuck_time = 0;
		}
		
		if(stuck_time > BULLET_STUCK_TIME)
		{
			stuck_flag = true;
			stuck_time = 0;
			// TODO:增加卡弹计数
			// TODO:屏蔽打弹指令
			tplt_info.shoot = 0;
			tplt_info.stuck_cnt++;
		}
	}
	else
	{
		tplt_pid.speed.target = BULLET_TURNBACK_SPEED;
		turnback_time++;
		if(turnback_time > BULLET_TURNBACK_TIME) 
		{
			stuck_flag = false;
			turnback_time = 0;
		}
	}
}

void TURNPLATE_Angle_BulletStuck(void)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint16_t stuck_flag = false;
	
	if(stuck_flag == false)
	{
		if(tplt_pid.speed.out > BULLET_STUCK_PID_OUT
		   && tplt_pid.speed.measure < BULLET_STUCK_SPEED)
		{
			stuck_time++;
		} else {
			stuck_time = 0;
		}
		
		if(stuck_time > BULLET_STUCK_TIME)
		{
			stuck_flag = true;
			stuck_time = 0;
			// TODO:增加卡弹计数
			// TODO:屏蔽打弹指令
			tplt_info.shoot = 0;
			tplt_info.stuck_cnt++;
			// 反拨至上一格点的位置
			tplt_pid.angle_ramp_target = tplt_pid.angle.target - TURNPLATE_BULLET_ANGLE;
		}
	}
	else
	{
		turnback_time++;
		if(turnback_time > BULLET_TURNBACK_TIME) 
		{
			stuck_flag = false;
			turnback_time = 0;
			tplt_pid.angle.target = tplt_pid.angle.measure;
			tplt_pid.angle_ramp_target = tplt_pid.angle.target;
		}
	}	
}

#define BULLET_HEAT		10
uint8_t TURNPLATE_HeatLimit(uint8_t shoot_num)
{
	// 无人机不限热量
	return shoot_num;
//	uint16_t heat_remain;
//	uint8_t	 allow_num;
//	
//	heat_remain = tplt_info.heat_limit - tplt_info.heat_real;
//	allow_num = heat_remain / BULLET_HEAT;
//	
//	return (allow_num >= shoot_num)?shoot_num:allow_num;
}

/* #遥控 ---------------------------------------------------------------------*/
void RC_SetTurnplateShoot(void)
{
    if(SW2.state == RC_SW_MID) {
        if(SW1_TRIG == SW_MID_TO_DOWN) {
            tplt_info.shoot = TURNPLATE_HeatLimit(1);
			tplt_info.shoot_time = osKernelSysTick();
        }
    }
    else if(SW2.state == RC_SW_DOWN) {
        if(SW1.state == RC_SW_DOWN) {
            tplt_info.state = TPLT_STATE_ON;
			tplt_info.shoot_freq = 20;
			tplt_info.shoot_time = osKernelSysTick();
        } else {
            tplt_info.state = TPLT_STATE_OFF;
			tplt_info.shoot_freq = 0;
        }
    }
}

void KEY_SetTurnplateShoot(void)
{
//    tplt_info.shoot_freq = 20;
//    tplt_info.shoot_interval = 1000 / tplt_info.shoot_freq;
    
//    if(MOUSE_L.state == PRESS) {
//        if((MOUSE_L.hold_time - MOUSE_L_TIME) >= tplt_info.shoot_interval) {
//            tplt_info.shoot += TURNPLATE_HeatLimit(1);
//            tplt_info.shoot_time = osKernelSysTick();
//            MOUSE_L_TIME = MOUSE_L.hold_time;
//        }
//    } else {
//        MOUSE_L_TIME = 0;
//    }
    
    // 点射晃动较大，改成连射
    if(MOUSE_L.state == PRESS) {
        tplt_info.state = TPLT_STATE_ON;
        tplt_info.shoot_freq = 20;
        tplt_info.shoot_time = osKernelSysTick();
    } else {
        tplt_info.state = TPLT_STATE_OFF;
        tplt_info.shoot_freq = 0;
    }
}

/* 任务层 --------------------------------------------------------------------*/
uint16_t tplt_speed_kalman_r = 3;
void TURNPLATE_Init(void)
{
	// 载入拨盘电机输出的本地驱动
	tplt_drv = tplt_dev.turnplate_motor->driver;
	// 载入拨盘电机的信息
	tplt_motor_info = tplt_dev.turnplate_motor->info;
    // 拨盘电机速度卡尔曼滤波器初始化
    KalmanCreate(&tplt_speed_kalman, 1, tplt_speed_kalman_r);
}

void TURNPLATE_GetInfo(void)
{
	TURNPLATE_GetSysInfo();
	TURNPLATE_GetJudgeInfo();
	TURNPLATE_GetRcInfo();
	TURNPLATE_UpdateController();
}

void TURNPLATE_SelfProtect(void)
{
	tplt_motor_info->angle_sum = 0;
	
	TURNPLATE_PidParamsInit(&tplt_pid);
	TURNPLATE_Stop(&tplt_pid);
	TURNPLATE_GetInfo();
}

void TURNPLATE_PidCtrl(void)
{
	/* 摩擦轮没有就绪屏蔽打弹指令 */
	if(fric.if_ready() == false)
	{
		TURNPLATE_PidParamsInit(&tplt_pid);
		// TODO:要不要卸力?正在射击时突然升级射速怎么处理?
		TURNPLATE_Stop(&tplt_pid);
		MOUSE_L_TIME = 0;
		tplt_info.shoot = 0;
		tplt_info.state = TPLT_STATE_OFF;
	}
	else
	{
		if(tplt_info.pid_mode == TPLT_SPEED_PID)
		{
			if(tplt_info.state == TPLT_STATE_ON) 
			{
//				if(TURNPLATE_HeatLimit(1) > 0)
					tplt_pid.speed.target = tplt_info.shoot_freq * TURNPLATE_BULLET_RPS;
//				else
//					tplt_pid.speed.target = 0;
			} else {
				tplt_pid.speed.target = 0;
			}
			
			TURNPLATE_Speed_BulletStuck();
			TURNPLATE_Speed_PidCalc(&tplt_pid);
		}
		else if(tplt_info.pid_mode == TPLT_POSI_PID)
		{
			TURNPLATE_Angle_BulletStuck();
			if(tplt_info.shoot > 0)
			{
				tplt_info.shoot--;
				tplt_pid.angle_ramp_target += TURNPLATE_BULLET_ANGLE;
				tplt_info.shoot_time = osKernelSysTick();
			}
			
			tplt_pid.angle.target = RampFloat(tplt_pid.angle_ramp_target, tplt_pid.angle.target, TURNPLATE_BULLET_RAMP);
			TURNPLATE_Angle_PidCalc(&tplt_pid);
			tplt_pid.speed.target = tplt_pid.angle.out;
			TURNPLATE_Speed_PidCalc(&tplt_pid);
		}
		/*----最终输出----*/
		TURNPLATE_PidOut(&tplt_pid);
	}
}

void TURNPLATE_NormalShootCtrl(void)
{
    KEY_SetTurnplateShoot();
}

void TURNPLATE_AutoShootCtrl(void)
{
    KEY_SetTurnplateShoot();
}

void TURNPLATE_BanShootCtrl(void)
{
}

void TURNPLATE_RcCtrl(void)
{
	if(fric.if_ready() == false)
		return;
	
	RC_SetTurnplateShoot();
}

void TURNPLATE_KeyCtrl(void)
{
	if(fric.if_ready() == false)
		return;
	
	switch(tplt_info.local_mode)
	{
		case TURNPLATE_MODE_NORMAL:
			TURNPLATE_NormalShootCtrl();
            break;
		case TURNPLATE_MODE_AUTO:
			TURNPLATE_AutoShootCtrl();
            break;
		case TURNPLATE_MODE_BAN:
			TURNPLATE_BanShootCtrl();
            break;
        default:
            break;
	}
}

void TURNPLATE_Ctrl(void)
{
	/*----信息读入----*/
	TURNPLATE_GetInfo();
	/*----期望修改----*/
	if(tplt_info.remote_mode == RC) {
		TURNPLATE_RcCtrl();
	}
	else if(tplt_info.remote_mode == KEY) {
		TURNPLATE_KeyCtrl();
	}
	
	/*----最终输出----*/
	TURNPLATE_PidCtrl();
}

int16_t test_shoot_freq = 12;
void TURNPLATE_Test(void)
{
	/*----信息读入----*/
	TURNPLATE_UpdateController();
	/*----期望修改----*/
	tplt_info.shoot_freq = test_shoot_freq;
	tplt_pid.speed.target = tplt_info.shoot_freq * TURNPLATE_BULLET_RPS;
	//TURNPLATE_Speed_BulletStuck();
	TURNPLATE_Speed_PidCalc(&tplt_pid);
	/*----最终输出----*/
	TURNPLATE_PidOut(&tplt_pid);
}
