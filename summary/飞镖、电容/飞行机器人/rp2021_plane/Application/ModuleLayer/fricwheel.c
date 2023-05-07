/**
 * @file        fricwheel.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        3-November-2020
 * @brief       Friction Wheel Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "fricwheel.h"

#include "rp_math.h"
#include "drv_io.h"
#include "judge.h"

/* Private macro -------------------------------------------------------------*/
#define FRIC_RAMP_STEP	2
#define FRICADAPTMAX    4

/* Private function prototypes -----------------------------------------------*/
static bool FRICWHEEL_IfOpen(void);
static bool FRICWHEEL_IfReady(void);
void FRICWHEEL_Init(void);
void FRICWHEEL_SelfProtect(void);
bool FRICWHEEL_Adjust(uint8_t *flg);
void FRICWHEEL_Adapt(void);
void FRICWHEEL_Ctrl(void);
void FRICWHEEL_Test(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 摩擦轮电机本地驱动
static drv_pwm_t	*fric_drv[FRIC_MOTOR_CNT];
// 遥控按键信息
static button_t         MOUSE_L;
static button_flip_t    MOUSE_L_TRIG;     
static switch_t         SW1;
static switch_flip_t    SW1_TRIG;
static switch_t         SW2;
static switch_flip_t    SW2_TRIG;

/* Exported variables --------------------------------------------------------*/
// 摩擦轮电机斜坡控制器
fric_ramp_t 	fric_ramp = {
	.speed.target = 0,
	.speed.measure = 0, 
};

// 摩擦轮模块控制器
fric_ctrl_t		fric_ctrl = {
	.fric = &fric_ramp,
};

// 摩擦轮模块传感器
fric_dev_t		fric_dev = {
	.fric_motor[FRIC_L] = &fric_motor[FRIC_L],
	.fric_motor[FRIC_R] = &fric_motor[FRIC_R],
	.rc_sensor = &rc_sensor,
    .judge_sensor = &judge_sensor
};

// 摩擦轮模块信息
fric_info_t		fric_info = {
	.remote_mode = RC,
	.local_mode = FRIC_MODE_NORMAL,
	.state = FRIC_STATE_OFF,
	.level = FRIC_LEVEL_HIGH,
};

// 摩擦轮pwm对照表
int16_t fric_pwm_table[] = {
	[FRIC_LEVEL_LOW] = 480,
	[FRIC_LEVEL_MID] = 525,
	[FRIC_LEVEL_HIGH] = 640,    //785 885 885
};

// 摩擦轮模块
fric_t	fric = {
	.controller = &fric_ctrl,
	.dev = &fric_dev,
	.info = &fric_info,
	.init = FRICWHEEL_Init,
	.ctrl = FRICWHEEL_Ctrl,
	.self_protect = FRICWHEEL_SelfProtect,
    .adjust = FRICWHEEL_Adjust,
    .adapt = FRICWHEEL_Adapt,
    .test = FRICWHEEL_Test,
	.if_open = FRICWHEEL_IfOpen,
	.if_ready = FRICWHEEL_IfReady,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
static void FRICWHEEL_Stop(fric_ramp_t *fric_ramp)
{
	fric_ramp->speed.target = 0;
	fric_ramp->speed.measure = RampInt(0, fric_ramp->speed.measure, FRIC_RAMP_STEP);
										   
	fric_drv[FRIC_L]->output(fric_drv[FRIC_L], 0);
	fric_drv[FRIC_R]->output(fric_drv[FRIC_R], 0);
}

#define MAX_PWM_VAL 800
static bool FRICWHEEL_Adjust(uint8_t *flg)
{
	static uint16_t standard_cnt = 0;   //校准计时变量
	static uint16_t cnt = 0;    //最大输出计时变量
	
	if(*flg)
	{

		if(cnt < 4000)  //先给最大输出4s
		{
            fric_drv[FRIC_L]->output(fric_drv[FRIC_L], MAX_PWM_VAL);
            fric_drv[FRIC_R]->output(fric_drv[FRIC_R], MAX_PWM_VAL);
		}
		else           //每1ms依次降低，直至最小输出
		{
			cnt = 4001;
			
            fric_drv[FRIC_L]->output(fric_drv[FRIC_L], MAX_PWM_VAL - standard_cnt);
            fric_drv[FRIC_R]->output(fric_drv[FRIC_R], MAX_PWM_VAL - standard_cnt);
			
			standard_cnt++;
			if(standard_cnt > MAX_PWM_VAL)   //达到最小输出
			{
				standard_cnt = 0;
				cnt = 0;
				*flg = 0;
				return true;
			}
		}
		
		cnt++;
	}
	
	return false;
}

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
static bool FRICWHEEL_IfOpen(void)
{
	if(fric_info.state == FRIC_STATE_ON)
		return true;
	else if(fric_info.state == FRIC_STATE_OFF)
		return false;
	return false;
}

static bool FRICWHEEL_IfReady(void)
{
	if(fric_info.state == FRIC_STATE_OFF)
	   return false;
	return (abs(fric_ramp.speed.target - fric_ramp.speed.measure)<=FRICADAPTMAX*5)?true:false;
}

void FRICWHEEL_GetSysInfo(void)
{
	if(sys.remote_mode == RC)
		fric_info.remote_mode = RC;
	else if(sys.remote_mode == KEY)
		fric_info.remote_mode = KEY;
}
	
void FRICWHEEL_GetJudgeInfo(void)
{    
//    static uint16_t pwm_delay_cnt = 0;
//    judge_sensor_t *judge_sen = fric_dev.judge_sensor;
    fric_info.level = FRIC_LEVEL_HIGH;
    
//    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
//        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
//        (judge_sen->info->aerial_robot_energy.attack_time != 0)) {
//        // 等待3s
//        if(pwm_delay_cnt < 1250) {
//            pwm_delay_cnt++;
//            fric_info.state = FRIC_STATE_OFF;
//        } else {
//            fric_info.state = FRIC_STATE_ON;
//        }
//    } else {
//        fric_info.state = FRIC_STATE_OFF;
//        fric_ramp.speed.target = 0;
//        fric_ramp.speed.measure = 0;
//        pwm_delay_cnt = 0;
//    }
    
    // 无人机射速上限为30m/s
//    if(judge_info.game_robot_status.mains_power_shooter_output) {
//        fric_info.local_mode = FRIC_MODE_NORMAL;
//        fric_info.state = FRIC_STATE_ON;
//    } else {
//        fric_info.local_mode = FRIC_MODE_BAN;
//        fric_info.state = FRIC_STATE_OFF;
//    }
}

void FRICWHEEL_GetRcInfo(void)
{
    rc_sensor_t *rc_sen = fric_dev.rc_sensor;
    
    SW1_TRIG = rc_sen->copy_switch(&SW1, &rc_sen->info->SW1);
    SW2_TRIG = rc_sen->copy_switch(&SW2, &rc_sen->info->SW2);
    MOUSE_L_TRIG = rc_sen->copy_button(&MOUSE_L, &rc_sen->info->MOUSE_L);
}

/* 应用层 --------------------------------------------------------------------*/
bool fric_ctrl_enable;
void RC_SetFricState(void)
{
    static uint16_t pwm_delay_cnt = 0;
    judge_sensor_t *judge_sen = fric_dev.judge_sensor;

    /* 摩擦轮已经供电 */
//    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
//        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
//        (judge_sen->info->aerial_robot_energy.attack_time != 0)) 
    if(1)
    {
        /* 延迟未完成 */
        if(pwm_delay_cnt < 1000) {
            pwm_delay_cnt++;
            // 禁止启动摩擦轮(防止触发摩擦轮校准程序)
            fric_ctrl_enable = false;
        } 
        /* 延迟完成 */
        else {
            // 允许启动摩擦轮
            fric_ctrl_enable = true;
        }
    }
    /* 摩擦轮未供电(禁止启动) */
    else {
        pwm_delay_cnt = 0;
        fric_ctrl_enable = false;
    }
    
    if(fric_ctrl_enable) {
        if((SW2.state == RC_SW_DOWN) && (SW1_TRIG == SW_MID_TO_UP))
        {
            if(fric_info.state == FRIC_STATE_OFF) {
                fric_info.state = FRIC_STATE_ON;
                LASER_ON();
            }
            else if(fric_info.state == FRIC_STATE_ON) {
                fric_info.state = FRIC_STATE_OFF;
//                LASER_OFF();
            }
        }
    } 
    else {
        // 迅速复位摩擦轮pwm输出值(防止供电瞬间触发校准程序)
//        LASER_OFF();
        fric_info.state = FRIC_STATE_OFF;
        fric_ramp.speed.target = 0;
        fric_ramp.speed.measure = 0;   
    }
}

void KEY_SetFricState(void)
{
    static uint16_t pwm_delay_cnt = 0;

    // 比赛：呼叫空中支援之后自动开启摩擦轮
    judge_sensor_t *judge_sen = fric_dev.judge_sensor;
    
    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
        (judge_sen->info->aerial_robot_energy.attack_time != 0)) {
        // 等待2.5s
        if(pwm_delay_cnt < 1250) {
            pwm_delay_cnt++;
            fric_info.state = FRIC_STATE_OFF;
        } else {
            fric_info.state = FRIC_STATE_ON;
        }
    } else {
        fric_info.state = FRIC_STATE_OFF;
        fric_ramp.speed.target = 0;
        fric_ramp.speed.measure = 0;
        pwm_delay_cnt = 0;
    }
    
    // 调试：鼠标左键开启摩擦轮
//    judge_sensor_t *judge_sen = fric_dev.judge_sensor;
//    
//    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
//        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
//        (judge_sen->info->aerial_robot_energy.attack_time != 0)) 
//    {
//        // 延迟启动摩擦轮(防止供电瞬间触发校准程序)
//        if(pwm_delay_cnt < 1000) {
//            pwm_delay_cnt++;
//            fric_info.state = FRIC_STATE_OFF;
//        }
//        else {
//            if(MOUSE_L.state == PRESS) {
//                fric_info.state = FRIC_STATE_ON;
//                LASER_ON();
//            }
//        }
//    }
//    else 
//    {
//        // 迅速复位摩擦轮pwm输出值(防止供电瞬间触发校准程序)
//        pwm_delay_cnt = 0;
//        LASER_OFF();
//        fric_info.state = FRIC_STATE_OFF;
//        fric_ramp.speed.target = 0;
//        fric_ramp.speed.measure = 0;
//    }
}

typedef struct{
    float last_speed;
    uint32_t last_shoot_time;
    uint32_t time_err[5];   //两次打弹的时间差，用来判断发射状态
    uint8_t num;            //数组序号
    int8_t single_err;      //单发的pwm偏差量
    int8_t burst_err;       //连发的pwm偏差量
    uint8_t single_i;        //调节量
    uint8_t burst_i;         //调节量
    uint8_t shoot_state;    //发射状态，0:单发，1:连发
    uint32_t idel_time;     //距离上次打弹的时间，用来判断发射状态
    float target_speed;     //目标速度
}fric_adapt_t;

fric_adapt_t fa = {
    .single_err = 0,
    .burst_err = 0,
    .single_i = 2,
    .burst_i = 2,
    .target_speed = 24.5f,
};

void FRICWHEEL_Adapt(void)
{
    judge_info_t* judge_info = fric.dev->judge_sensor->info;
    if(fa.last_speed != judge_info->shoot_data.bullet_speed)
    {
//        fa.idel_time = 0;
        fa.last_speed = judge_info->shoot_data.bullet_speed;
        fa.time_err[fa.num] = HAL_GetTick() - fa.last_shoot_time;
        fa.last_shoot_time = HAL_GetTick();
//        fa.num = (fa.num + 1) % 5;
//        if(fa.time_err[0] < 150 && fa.time_err[1] < 150 && fa.time_err[2] < 150 \
//           && fa.time_err[3] < 150 && fa.time_err[4] < 150){
//            fa.shoot_state = 1;
//            fa.burst_err += constrain(fa.burst_i * (fa.target_speed - judge_info->shoot_data.bullet_speed), -FRICADAPTMAX, FRICADAPTMAX);
//            if(fa.burst_err > 40){
//                fa.burst_err = 40;
//            }
//            else if(fa.burst_err < -40){
//                fa.burst_err = -40;
//            }
//        }
//        else{
//            fa.shoot_state = 0;
            fa.single_err += constrain(fa.single_i * (fa.target_speed - judge_info->shoot_data.bullet_speed), -FRICADAPTMAX, FRICADAPTMAX);
            if(fa.single_err > 40){
                fa.single_err = 40;
            }
            else if(fa.single_err < -40){
                fa.single_err = -40;
            }
//        }
    }
//    else{
//        fa.idel_time++;
//        if(fa.idel_time > 400){
//            fa.shoot_state = 0;
//        }
//    }
}

/* 任务层 --------------------------------------------------------------------*/
void FRICWHEEL_Init(void)
{
	// 载入摩擦轮电机输出的本地驱动
	fric_drv[FRIC_L] = fric_dev.fric_motor[FRIC_L]->driver;
	fric_drv[FRIC_R] = fric_dev.fric_motor[FRIC_R]->driver;
}

void FRICWHEEL_GetInfo(void)
{
	// 获取系统信息
	FRICWHEEL_GetSysInfo();
	// 获取裁判系统信息
	FRICWHEEL_GetJudgeInfo();	
	// 更新遥控器数据
	FRICWHEEL_GetRcInfo();
}

void FRICWHEEL_SelfProtect(void)
{
	fric_info.state = FRIC_STATE_OFF;
	FRICWHEEL_Stop(&fric_ramp);
	FRICWHEEL_GetInfo();
}

void FRICWHEEL_RampCtrl(void)
{
    //弹速自适应
    fric.adapt();
    
	// 根据射速等级更新摩擦轮转速
	if(fric_info.state == FRIC_STATE_ON)
		fric_ramp.speed.target = fric_pwm_table[fric_info.level] + (fa.shoot_state ? fa.burst_err : fa.single_err);
	else
		fric_ramp.speed.target = 0;
    
	// 斜坡加速
    fric_ramp.speed.measure = RampInt(fric_ramp.speed.target, fric_ramp.speed.measure, FRIC_RAMP_STEP);
	fric_drv[FRIC_L]->output(fric_drv[FRIC_L], fric_ramp.speed.measure);
	fric_drv[FRIC_R]->output(fric_drv[FRIC_R], fric_ramp.speed.measure);
}

void FRICWHEEL_RcCtrl(void)
{
	RC_SetFricState();
}

void FRICWHEEL_KeyCtrl(void)
{
	KEY_SetFricState();
}

void FRICWHEEL_Ctrl(void)
{
	/*----信息读入----*/
	FRICWHEEL_GetInfo();
	/*----期望修改----*/
	if(fric_info.remote_mode == RC) {
		FRICWHEEL_RcCtrl();
	}
	else if(fric_info.remote_mode == KEY) {
		FRICWHEEL_KeyCtrl();
	}
	
	/*----最终输出----*/
    
	FRICWHEEL_RampCtrl();
}

void FRICWHEEL_Test(void)
{
    FRICWHEEL_GetInfo();
    
    FRICWHEEL_KeyCtrl();
    /*----最终输出----*/
	FRICWHEEL_RampCtrl();
}
