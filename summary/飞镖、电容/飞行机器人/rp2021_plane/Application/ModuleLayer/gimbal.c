/**
 * @file        gimbal.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        24-October-2020
 * @brief       Gimbal Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"

#include "can_potocol.h"
#include "device.h"
#include "rp_math.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "arm_math.h"
#include "drv_can.h"
#include "drv_haltick.h"
#include "vision.h"

#include "cmsis_os.h"
/* Private macro -------------------------------------------------------------*/
/*
    云台
    *pit        下-上+ (机械角度)
                下+上- (陀螺仪原始角度)
    *ratepit    下-上+ (机械角速度)
                下+上- (陀螺仪原始角速度)
    *yaw        逆-顺+ (机械角度)
                逆+顺- (陀螺仪原始角度)
    *rateyaw    逆-顺+ (机械角速度)
                逆-顺+ (陀螺仪原始角速度)
                
    以云台电机的方向作为标准，调整陀螺仪反馈数据的方向
*/
#define GIM_MECH_YAW_RIGH       (1750)
#define GIM_MECH_YAW_MID        (650)
#define GIM_MECH_YAW_LEFT       (7700)

#define GIM_MECH_PIT_UP			(3713)
#define GIM_MECH_PIT_MID		(4150)
#define GIM_MECH_PIT_DOWN		(4590)

float GIM_GYRO_PIT_UP	    = -200.f;
float GIM_GYRO_PIT_MID		= 0.f;
float GIM_GYRO_PIT_DOWN		= 550.f;
float GIM_MECH_PIT_BIAS     = 3600.f;

#define GIM_RAMP_BEGIN_YAW		2.f //1.f 0.75f
#define GIM_RAMP_BEGIN_PIT		0.7f  //0.35f

const float RC_GIM_MECH_YAW_SENS = 0.0080f;
const float RC_GIM_MECH_PIT_SENS = 0.0023f;
const float RC_GIM_GYRO_YAW_SENS = 0.0080f; //0.0003f
const float RC_GIM_GYRO_PIT_SENS = 0.0001f;	//0.0050f

float KEY_GIM_MECH_YAW_SENS = 0.040f;
float KEY_GIM_MECH_PIT_SENS = 0.030f;
float KEY_GIM_GYRO_YAW_SENS = 0.040f; //0.0003f
float KEY_GIM_GYRO_PIT_SENS = 0.0001f;

// 放大之后无人机云台会
float IMU_GIM_GYRO_ZOOM_INDEX = 1;	// 陀螺仪角度放大系数
    
/* Private function prototypes -----------------------------------------------*/
static float GIMBAL_GetRealMechAngleError(pid_ctrl_t *pid);
void GIMBAL_Init(void);
void GIMBAL_SelfProtect(void);
void GIMBAL_Ctrl(void);

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// 云台电机本地驱动
static drv_can_t			*gim_drv[GIMBAL_MOTOR_CNT];
static gimbal_motor_t		*gim_motor[GIMBAL_MOTOR_CNT];

// imu数据记录
static float pitch;
static float yaw;
static float last_yaw = 0.f;	

/* 卡尔曼滤波器 */
extKalman_t gim_angle_kalman_err[CO_PID_MODE_CNT][GIMBAL_MOTOR_CNT];
extKalman_t imu_rate_kalman_err[3];
extKalman_t imu_pit_kalman_err;
extKalman_t rc_rud_kalman_err;
extKalman_t rc_rlr_kalman_err;
extKalman_t mouse_x_kalman_err;
extKalman_t mouse_y_kalman_err;

/* 滑动平均滤波器 */
float_avr_filter_t mouse_x_avr = {
    .buff_len = 10,
    .cur_index = 0,
};
float_avr_filter_t mouse_y_avr = {
    .buff_len = 10,
    .cur_index = 0,
};

/* 自瞄算法 */
float start_gyro_yaw=0;
float now_gyro_yaw=0;
float last_gyro_yaw=0;
float delta_gyro_yaw=0;

/* 按键 */
static button_t KEY_W;
static button_flip_t KEY_W_TRIG;
static button_t KEY_S;
static button_flip_t KEY_S_TRIG;
static button_t KEY_A;
static button_flip_t KEY_A_TRIG;
static button_t KEY_D;
static button_flip_t KEY_D_TRIG;
static button_t KEY_SHIFT;
/* Exported variables --------------------------------------------------------*/
// 云台电机PID控制器
gimbal_pid_t 	gim_pid[CO_PID_MODE_CNT][GIMBAL_MOTOR_CNT] = {
	[MECH] = {
		[YAW] = {
			// 角度环
			.angle.integral_max = 0,
			.angle.out_max = 3500,
            .angle.loop_time = 0.002,
			// 速度环
			.speed.integral_max = 36000,
			.speed.out_max = 29999,
            .speed.loop_time = 0.002,
		},
		[PIT] = {
			// 角度环
			.angle.integral_max = 0,
			.angle.out_max = 1500,
            .angle.loop_time = 0.002,
			// 速度环
			.speed.integral_max = 24000,
			.speed.out_max = 29999, // GM3510:+-29000
            .speed.loop_time = 0.002,
		},
	},
	[GYRO] = {
		[YAW] = {
			// 角度环
			.angle.integral_max = 0,
			.angle.out_max = 3500,
            .angle.loop_time = 0.002,
			// 速度环
			.speed.integral_max = 36000,
			.speed.out_max = 29999,	
            .speed.loop_time = 0.002,
		},
		[PIT] = {
			// 角度环
			.angle.integral_max = 0,
			.angle.out_max = 1500,
            .angle.loop_time = 0.002,
			// 速度环
			.speed.integral_max = 24000,
			.speed.out_max = 29999, // GM3510:+-29000
            .speed.loop_time = 0.002,
		},
	},
};

// 云台模块控制器
gimbal_ctrl_t 	gim_ctrl = {
	.mech_yaw = &gim_pid[MECH][YAW],
	.gyro_yaw = &gim_pid[GYRO][YAW],
	.mech_pit = &gim_pid[MECH][PIT],
	.gyro_pit = &gim_pid[GYRO][PIT],
};

// 云台模块传感器
gimbal_dev_t 	gim_dev = {
	.yaw_motor = &gimbal_motor[YAW],
	.pit_motor = &gimbal_motor[PIT],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
    .vision_sensor = &vision_sensor
};

// 云台模块信息
gimbal_info_t 	gim_info = {
	.remote_mode = RC,
	.co_pid_mode = MECH,
	.local_mode = GIMBAL_MODE_NORMAL,
};

// 云台模块
gimbal_t 		gimbal = {
	.controller = &gim_ctrl,
	.dev = &gim_dev,
	.info = &gim_info,
	.init = GIMBAL_Init,
	.ctrl = GIMBAL_Ctrl,
	.self_protect = GIMBAL_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
/**
 *	@brief	云台电机PID参数初始化
 */
static void GIMBAL_PidParamsInit(gimbal_pid_t *pid, uint8_t motor_cnt)
{
	uint8_t i;
	for(i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].angle);
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}
}

/**
 *	@brief	云台电机卸力
 */
static void GIMBAL_Stop(gimbal_pid_t *pid)
{
    gim_drv[YAW]->add_tx_msg(gim_drv[YAW], (int16_t)0);
    gim_drv[PIT]->add_tx_msg(gim_drv[PIT], (int16_t)0);
    gim_drv[YAW]->start_tx(gim_drv[YAW]);
    gim_drv[PIT]->start_tx(gim_drv[PIT]);
}

/**
 *	@brief	云台电机pid输出
 *	@note	这里简化处理，要求yaw和pit挂载在同一CAN上且std_id相同
 */
static void GIMBAL_PidOut(gimbal_pid_t *pid)
{
    gim_drv[YAW]->add_tx_msg(gim_drv[YAW], (int16_t)pid[YAW].out);  // (int16_t)pid[YAW].out
    gim_drv[PIT]->add_tx_msg(gim_drv[PIT], (int16_t)pid[PIT].out);  // (int16_t)pid[PIT].out    
    gim_drv[YAW]->start_tx(gim_drv[YAW]);
}

/**
 *	@brief	云台电机位置环
 *	@note
 *			串环PID
 *			内环 期望值 期望角速度(实际上就是速度环)
 *				 输出值 期望角加速度
 *
 *			外环 期望值 期望角度
 *			     输出值 期望角速度
 */
float gyro_yaw_kalman_err;
static void GIMBAL_Angle_PidCalc(gimbal_pid_t *pid, gimbal_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	
	if(((gim_info.co_pid_mode == MECH) && (MOTORx == YAW || MOTORx == PIT)) || ((gim_info.co_pid_mode == GYRO) && MOTORx == YAW)) {
		pid[MOTORx].angle.err = GIMBAL_GetRealMechAngleError(&pid[MOTORx].angle);
	}
	
	if(gim_info.local_mode == GIMBAL_MODE_NORMAL) {
        pid[MOTORx].angle.err = KalmanFilter(&gim_angle_kalman_err[gim_info.co_pid_mode][MOTORx], pid[MOTORx].angle.err);
	}
    
	single_pid_ctrl(&pid[MOTORx].angle);
}

/**
 *	@brief	云台电机速度环
 */
float js_yaw_speed_target;
float js_yaw_speed_measure;
float js_yaw_speed_integral;
float js_pit_speed_target;
float js_pit_speed_measure;
float js_pit_speed_integral;
float js_yaw_speed_out;
float js_pit_speed_out;
float js_pit_motor_speed;
bool test_pi_inv_proc = false;
float k_pi_inv_proc_up = 1.f;
float k_pi_inv_proc_down = 1.f;
// 不能对误差再做卡尔曼滤波，会因为滞后导致控制不到位，从而引起机体更容易振荡
static void GIMBAL_Speed_PidCalc(gimbal_pid_t *pid, gimbal_motor_cnt_t MOTORx)
{
    js_yaw_speed_target = gim_pid[gim_info.co_pid_mode][YAW].speed.target;
    js_yaw_speed_measure = gim_pid[gim_info.co_pid_mode][YAW].speed.measure;
    js_pit_speed_target = gim_pid[gim_info.co_pid_mode][PIT].speed.target;
    js_pit_speed_measure = gim_pid[gim_info.co_pid_mode][PIT].speed.measure;
    js_pit_motor_speed = gim_motor[PIT]->info->speed;
    // TODO:加上i和p反向时的处理
//	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
//	single_pid_ctrl(&pid[MOTORx].speed);
//	pid[MOTORx].out = pid[MOTORx].speed.out;
    
    pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	// 积分
	pid[MOTORx].speed.integral += pid[MOTORx].speed.err;
	pid[MOTORx].speed.integral = constrain(pid[MOTORx].speed.integral, -pid[MOTORx].speed.integral_max, +pid[MOTORx].speed.integral_max);
	// p i d 输出项计算
	pid[MOTORx].speed.pout = pid[MOTORx].speed.kp * pid[MOTORx].speed.err;
	pid[MOTORx].speed.iout = pid[MOTORx].speed.ki * pid[MOTORx].speed.integral * pid[MOTORx].speed.loop_time;
	
    // 比例项与积分项反向
    if(test_pi_inv_proc && (MOTORx == PIT)) 
    {
        /* 向下降 */
        if((pid[MOTORx].speed.pout > 0) && (pid[MOTORx].speed.iout < 0)) 
        {
            pid[MOTORx].speed.integral += pid[MOTORx].speed.ki * k_pi_inv_proc_down / pid[MOTORx].speed.pout;
            pid[MOTORx].speed.integral = constrain(pid[MOTORx].speed.integral, -pid[MOTORx].speed.integral_max, +pid[MOTORx].speed.integral_max);
        }
        /* 向上抬 */
        if((pid[MOTORx].speed.pout < 0) && (pid[MOTORx].speed.iout > 0))
        {
            pid[MOTORx].speed.integral += pid[MOTORx].speed.ki * k_pi_inv_proc_up / pid[MOTORx].speed.pout;//pid[MOTORx].speed.pout * pid[MOTORx].speed.ki * 0.002f * k_pi_inv_proc_up;
            pid[MOTORx].speed.integral = constrain(pid[MOTORx].speed.integral, -pid[MOTORx].speed.integral_max, +pid[MOTORx].speed.integral_max);
        }
    }
    
    pid[MOTORx].speed.dout = pid[MOTORx].speed.kd * (pid[MOTORx].speed.err - pid[MOTORx].speed.last_err) / pid[MOTORx].speed.loop_time;
	// 累加pid输出值
	pid[MOTORx].speed.out = pid[MOTORx].speed.pout + pid[MOTORx].speed.iout + pid[MOTORx].speed.dout;
	pid[MOTORx].speed.out = constrain(pid[MOTORx].speed.out, -pid[MOTORx].speed.out_max, pid[MOTORx].speed.out_max);
	// 记录上次误差值
	pid[MOTORx].speed.last_err = pid[MOTORx].speed.err;
    
    // 最终输出 = 内环输出
    pid[MOTORx].out = pid[MOTORx].speed.out;
    
    js_yaw_speed_integral = gim_pid[gim_info.co_pid_mode][YAW].speed.integral;
    js_pit_speed_integral = gim_pid[gim_info.co_pid_mode][PIT].speed.integral;
    js_yaw_speed_out = gim_pid[gim_info.co_pid_mode][YAW].out;
    js_pit_speed_out = gim_pid[gim_info.co_pid_mode][PIT].out;
}

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
/**
 *	@brief	云台电机机械角度过零点
 */
static float GIMBAL_GetRealMechAngleError(pid_ctrl_t *pid)
{
	float err;
	err = pid->target - pid->measure;
	
	if(err >= 4096.f) {
		err = -8192.f + err;
	} else if(err <= -4096.f) {
		err = +8192.f + err;
	}
	
	return err;
}

/**
 *	@brief	云台电机目标机械角度过零点
 */
static float GIMBAL_GetRealMechAngleTarget(pid_ctrl_t *pid, float delta_target, gimbal_motor_cnt_t yaw_or_pit)
{
	float target;
	target = pid->target + delta_target;

    if(yaw_or_pit == YAW) {
        if(target > 8191.f) {
            target = (target - 8192.f);
        } else if(target < 0.f) {
            target = (target + 8192.f);
        }
        
        target = constrain(target, 0, 8191.f);
    }
    else if(yaw_or_pit == PIT) {
        if(target > 8191.f) {
            target = (target - 8192.f);
        } else if(target < 0.f) {
            target = (target + 8192.f);
        }
    }
    
	return target;	
}

static float GIMBAL_MechAngleConstrain(pid_ctrl_t *pid, gimbal_motor_cnt_t yaw_or_pit)
{
    if(yaw_or_pit == YAW) {
        /* 处于越界区域 */
        if(pid->target - GIM_MECH_YAW_RIGH > 0 && pid->target - GIM_MECH_YAW_RIGH < 1200){
            return GIM_MECH_YAW_RIGH;
        }
        else if(pid->target - GIM_MECH_YAW_LEFT < 0 && pid->target - GIM_MECH_YAW_LEFT > -1200){
            return GIM_MECH_YAW_LEFT;
        }
        /* 没有越界 */
        else {
            return pid->target;
        }
    }
    else if(yaw_or_pit == PIT) {
        /* 处于越界区域 */
        if((pid->target > GIM_MECH_PIT_DOWN) || (pid->target < GIM_MECH_PIT_UP)) {
            if(abs(pid->target - GIM_MECH_PIT_UP) < abs(pid->target - GIM_MECH_PIT_DOWN)) {
                return GIM_MECH_PIT_UP;
            }
            else {
                return GIM_MECH_PIT_DOWN;
            }
        }
        /* 没有越界 */
        else {
            return pid->target;
        }
    }
    return pid->target;
}

static float GIMBAL_GyroAngleConstrain(pid_ctrl_t *pid, gimbal_motor_info_t *moto, gimbal_motor_cnt_t yaw_or_pit)
{
    if(yaw_or_pit == PIT) {
        /* 处于越界区域 */
        if((pid->target - pid->measure) / 360.f * 8192.f > GIM_MECH_PIT_DOWN - moto->angle)
        {
            return (GIM_MECH_PIT_DOWN - moto->angle) / 8192.f * 360.f + pid->measure;
        }
        else if((pid->target - pid->measure) / 360.f * 8192.f < GIM_MECH_PIT_UP - moto->angle) 
        {
            return (GIM_MECH_PIT_UP - moto->angle) / 8192.f * 360.f + pid->measure;
        }
        /* 没有越界 */
        else {
            return pid->target;
        }
    }
    return pid->target;
}

/**
 *	@brief	云台获取系统信息
 */
void GIMBAL_GetSysInfo(void)
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		gim_info.remote_mode = RC;
        if(mouse_x_avr.cur_index != 0)
            MovingAverageFloat_Clear(&mouse_x_avr);
        if(mouse_y_avr.cur_index != 0)
            MovingAverageFloat_Clear(&mouse_y_avr);
	}
	else if(sys.remote_mode == KEY) {
		gim_info.remote_mode = KEY;
	}
	
	/*----本地模式修改----*/
	switch(sys.mode)
	{
		case SYS_MODE_NORMAL:
            {
                /* 刚进入常规模式 */
                if(gim_info.local_mode != GIMBAL_MODE_NORMAL) {
                    gim_pid[gim_info.co_pid_mode][YAW].angle.target = gim_pid[gim_info.co_pid_mode][YAW].angle.measure;
                    gim_pid[gim_info.co_pid_mode][PIT].angle.target = gim_pid[gim_info.co_pid_mode][PIT].angle.measure;
                }
                gim_info.local_mode = GIMBAL_MODE_NORMAL;
            }
            break;
		case SYS_MODE_AUTO:
            {
                /* 刚进入自瞄模式 */
                if(gim_info.local_mode == GIMBAL_MODE_NORMAL) {
                    flag.gimbal.prepare_auto = true;
                }
                    
                if(sys.act == SYS_ACT_AIM_OUTPOST)
                    gim_info.local_mode = GIMBAL_MODE_AIM_OUTPOST;
                else if(sys.act == SYS_ACT_AIM_ARMY)
                    gim_info.local_mode = GIMBAL_MODE_AIM_ARMY;
                else
                    gim_info.local_mode = GIMBAL_MODE_NORMAL;
            }
            break;
        default:
            break;
	}
	
	/*----云台底盘配合模式修改----*/
	if(sys.co_pid_mode != gim_info.co_pid_mode)
	{
		/* GYRO -> MECH */
		if(sys.co_pid_mode == MECH)
		{
			gim_pid[MECH][YAW].angle.target = gim_motor[YAW]->info->angle;
			gim_pid[MECH][YAW].speed.integral = gim_pid[GYRO][YAW].speed.iout / gim_pid[MECH][YAW].speed.ki; // 0
			gim_pid[MECH][PIT].angle.target = gim_motor[PIT]->info->angle;
			gim_pid[MECH][PIT].speed.integral = gim_pid[GYRO][PIT].speed.iout / gim_pid[MECH][PIT].speed.ki;  // gim_pid[MECH][PIT].speed.ki;
            gim_info.co_pid_mode = MECH;	
		}
		/* MECH -> GYRO */
		else if(sys.co_pid_mode == GYRO)
		{
			gim_pid[GYRO][YAW].angle.target = gim_motor[YAW]->info->angle;
			gim_pid[GYRO][YAW].speed.integral = gim_pid[MECH][YAW].speed.integral;
			gim_pid[GYRO][PIT].angle.target = gim_pid[GYRO][PIT].angle.measure;//gim_pid[GYRO][PIT].angle.measure;
			gim_pid[GYRO][PIT].speed.integral = gim_pid[MECH][PIT].speed.iout / gim_pid[GYRO][PIT].speed.ki;  // gim_pid[GYRO][PIT].speed.ki;
            gim_info.co_pid_mode = GYRO;	
		}
	}
    
}

void GIMBAL_GetJudgeInfo(void)
{
	
}

void GIMBAL_GetRcInfo(void)
{
    rc_sensor_t *rc_sen = gim_dev.rc_sensor;
    
    KEY_W_TRIG = rc_sen->copy_button(&KEY_W, &rc_sen->info->W);
    KEY_S_TRIG = rc_sen->copy_button(&KEY_S, &rc_sen->info->S);
    KEY_A_TRIG = rc_sen->copy_button(&KEY_A, &rc_sen->info->A);
    KEY_D_TRIG = rc_sen->copy_button(&KEY_D, &rc_sen->info->D);
    rc_sen->copy_button(&KEY_SHIFT, &rc_sen->info->SHIFT);
}

float rate_yaw;
float rate_pitch;
float rate_roll;
float rate_yaw_trans;
float delta_pitch;
int16_avr_filter_t rate_yaw_filter = {
    .buff_len = 10,
};
int16_avr_filter_t rate_pit_filter = {
    .buff_len = 10,
};

float js_rate_yaw_raw;
float js_rate_pit_raw;
float delta_pit;
float last_pitch;
void GIMBAL_UpdateController(void)
{
	imu_sensor_info_t *imu = gim_dev.imu_sensor->info;
	
	float delta_yaw;

	/* # Yaw # */
	yaw = -imu->yaw;
	delta_yaw = yaw - last_yaw;
	if(delta_yaw > +180.f) {
		delta_yaw = -360.f + delta_yaw;//(+360.f - delta_yaw);
	} else if(delta_yaw < -180.f) {
		delta_yaw = +360.f + delta_yaw;//(-360.f - delta_yaw);
	}
	last_yaw = yaw; 	
	
	/* # Pitch # */
	pitch = imu->pitch;
	delta_pitch = pitch - last_pitch;
    last_pitch = pitch;
    // 云台复位完成之后开始计算
    if(flag.gimbal.reset_ok == false) {
        delta_pitch = 0;
    }

    /* 对陀螺仪数据进行卡尔曼滤波 */
    rate_yaw = KalmanFilter(&imu_rate_kalman_err[0], imu->rate_yaw);
    rate_pitch = KalmanFilter(&imu_rate_kalman_err[1], imu->rate_pitch);
    rate_roll = KalmanFilter(&imu_rate_kalman_err[2], imu->rate_roll);
    
    js_rate_yaw_raw = rate_yaw;
    js_rate_pit_raw = rate_pitch;
    
	/* # Yaw # */
	/* # 机械模式下的角度反馈值利用yaw电机的角度值 */
	gim_pid[MECH][YAW].angle.measure = gim_motor[YAW]->info->angle;
	/* # 陀螺仪模式下的角度反馈值利用IMU的角度值 */
	gim_pid[GYRO][YAW].angle.measure = gim_motor[YAW]->info->angle;
	/* # 机械模式下的速度反馈值利用IMU的角速度值 */
	gim_pid[MECH][YAW].speed.measure = rate_yaw;	// GIMBAL_COMPENSATE_RATEYAW
	/* # 陀螺仪模式下的速度反馈值利用IMU的角速度值 */
	gim_pid[GYRO][YAW].speed.measure = rate_yaw;	// GIMBAL_COMPENSATE_RATEYAW
	
	/* # 机械模式下的角度反馈值利用Pitch机械角度反馈值 */
    gim_pid[MECH][PIT].angle.measure = gim_motor[PIT]->info->angle;
	/* # 陀螺仪模式下的角度反馈值利用IMU的角度值 */
	gim_pid[GYRO][PIT].angle.measure += delta_pitch;//pitch * IMU_GIM_GYRO_ZOOM_INDEX;	
	/* # 机械模式下的速度反馈值利用IMU的角速度值 */
	gim_pid[MECH][PIT].speed.measure = rate_pitch;	// pit电机输出为正时抬头，-rate_pitch对应抬头
	/* # 陀螺仪模式下的速度反馈值利用IMU的角速度值 */
	gim_pid[GYRO][PIT].speed.measure = rate_pitch;	// pit电机输出为正时抬头，-rate_pitch对应抬头
}

/* 应用层 --------------------------------------------------------------------*/
/* #云台 ---------------------------------------------------------------------*/
/**
 *	@brief	根据云台当前模式进行PID参数的切换
 */
float aim_static_yaw_angle_kp = 16.25f, aim_static_yaw_speed_kp = 16.25f , aim_static_yaw_speed_ki = 140.f;	
float aim_static_pit_angle_kp = 14.75f , aim_static_pit_speed_kp = 17.75f, aim_static_pit_speed_ki = 185.f;	
// pit 12 yaw 13
float aim_army_yaw_angle_kp = 10.f, aim_army_yaw_speed_kp = 16.25f , aim_army_yaw_speed_ki = 140.f;	
float aim_army_pit_angle_kp = 10.f , aim_army_pit_speed_kp = 17.75f, aim_army_pit_speed_ki = 185.f;	

//float normal_mech_yaw_angle_kp = 20.f , normal_mech_yaw_speed_kp = 15.f  , normal_mech_yaw_speed_ki = 160.f; 
//float normal_gyro_yaw_angle_kp = 125.f, normal_gyro_yaw_speed_kp = 16.25f, normal_gyro_yaw_speed_ki = 140.f;

//float normal_mech_yaw_angle_kp = 30.f, normal_mech_yaw_speed_kp = 10.f, normal_mech_yaw_speed_ki = 50.f; 
//float normal_gyro_yaw_angle_kp = 30.f, normal_gyro_yaw_speed_kp = 10.f, normal_gyro_yaw_speed_ki = 50.f;

//float normal_mech_yaw_angle_kp = 20.f, normal_mech_yaw_speed_kp = 7.f, normal_mech_yaw_speed_ki = 34.f; 
//float normal_gyro_yaw_angle_kp = 20.f, normal_gyro_yaw_speed_kp = 7.f, normal_gyro_yaw_speed_ki = 34.f;

float normal_mech_yaw_angle_kp = 21.f, normal_mech_yaw_speed_kp = 20.f, normal_mech_yaw_speed_ki = 270.f; 
float normal_gyro_yaw_angle_kp = 21.f, normal_gyro_yaw_speed_kp = 20.f, normal_gyro_yaw_speed_ki = 270.f;

// 主控上方夹长绿板
//float normal_mech_pit_angle_kp = 15.85f, normal_mech_pit_speed_kp = 17.75f, normal_mech_pit_speed_ki = 185.f;
//float normal_gyro_pit_angle_kp = 15.85f, normal_gyro_pit_speed_kp = 17.75f, normal_gyro_pit_speed_ki = 185.f;

// 主控上方夹短碳板
//float normal_mech_pit_angle_kp = 10.f, normal_mech_pit_speed_kp = 10.25f, normal_mech_pit_speed_ki = 135.f;
//float normal_gyro_pit_angle_kp = 10.f, normal_gyro_pit_speed_kp = 10.25f, normal_gyro_pit_speed_ki = 135.f;

//float normal_mech_pit_angle_kp = 20.f, normal_mech_pit_speed_kp = 3.f, normal_mech_pit_speed_ki = 120.f;
//float normal_gyro_pit_angle_kp = 350.f, normal_gyro_pit_speed_kp = 3.5f, normal_gyro_pit_speed_ki = 140.f;

float normal_mech_pit_angle_kp = 26.0f, normal_mech_pit_speed_kp = 14.0f, normal_mech_pit_speed_ki = 300.f;
//float normal_gyro_pit_angle_kp = 150.f, normal_gyro_pit_speed_kp = 3.5f, normal_gyro_pit_speed_ki = 75.f;
float normal_gyro_pit_angle_kp = 150.f, normal_gyro_pit_speed_kp = 14.0f, normal_gyro_pit_speed_ki = 300.f;

//float normal_mech_pit_angle_kp = 15.f, normal_mech_pit_speed_kp = 2.4f, normal_mech_pit_speed_ki = 80.f;
//float normal_gyro_pit_angle_kp = 140.f, normal_gyro_pit_speed_kp = 1.4f, normal_gyro_pit_speed_ki = 60.f;

void GIMBAL_PidParamsSwitch(void)
{
	switch(gim_info.local_mode) {
		case GIMBAL_MODE_NORMAL:
			gim_pid[MECH][YAW].speed.kp = normal_mech_yaw_speed_kp;
			gim_pid[MECH][YAW].speed.ki = normal_mech_yaw_speed_ki;
			gim_pid[MECH][YAW].speed.kd = 0;
			gim_pid[MECH][YAW].angle.kp = normal_mech_yaw_angle_kp;		
			gim_pid[MECH][YAW].angle.ki = 0;		
			gim_pid[MECH][YAW].angle.kd = 0;
		
			gim_pid[MECH][PIT].speed.kp = normal_mech_pit_speed_kp;
			gim_pid[MECH][PIT].speed.ki = normal_mech_pit_speed_ki;
			gim_pid[MECH][PIT].speed.kd = 0;                      
            gim_pid[MECH][PIT].angle.kp = normal_mech_pit_angle_kp;
			gim_pid[MECH][PIT].angle.ki = 0;		
			gim_pid[MECH][PIT].angle.kd = 0;

			gim_pid[GYRO][YAW].speed.kp = normal_gyro_yaw_speed_kp;		
			gim_pid[GYRO][YAW].speed.ki = normal_gyro_yaw_speed_ki;		
			gim_pid[GYRO][YAW].speed.kd = 0;
			gim_pid[GYRO][YAW].angle.kp = normal_gyro_yaw_angle_kp;		
			gim_pid[GYRO][YAW].angle.ki = 0;		
			gim_pid[GYRO][YAW].angle.kd = 0;
			
			gim_pid[GYRO][PIT].speed.kp = normal_gyro_pit_speed_kp;
			gim_pid[GYRO][PIT].speed.ki = normal_gyro_pit_speed_ki;
			gim_pid[GYRO][PIT].speed.kd = 0;
			gim_pid[GYRO][PIT].angle.kp = normal_gyro_pit_angle_kp;
			gim_pid[GYRO][PIT].angle.ki = 0;		
			gim_pid[GYRO][PIT].angle.kd = 0;
			break;
            
		case GIMBAL_MODE_AIM_OUTPOST:
			gim_pid[gim_info.co_pid_mode][YAW].speed.kp = aim_static_yaw_speed_kp;
			gim_pid[gim_info.co_pid_mode][YAW].speed.ki = aim_static_yaw_speed_ki;		
			gim_pid[gim_info.co_pid_mode][YAW].speed.kd = 0;
			gim_pid[gim_info.co_pid_mode][YAW].angle.kp = aim_static_yaw_angle_kp;		
			gim_pid[gim_info.co_pid_mode][YAW].angle.ki = 0;		
			gim_pid[gim_info.co_pid_mode][YAW].angle.kd = 0;

            gim_pid[gim_info.co_pid_mode][PIT].speed.kp = aim_static_pit_speed_kp;
			gim_pid[gim_info.co_pid_mode][PIT].speed.ki = aim_static_pit_speed_ki;
			gim_pid[gim_info.co_pid_mode][PIT].speed.kd = 0;
			gim_pid[gim_info.co_pid_mode][PIT].angle.kp = aim_static_pit_angle_kp;
			gim_pid[gim_info.co_pid_mode][PIT].angle.ki = 0;		
			gim_pid[gim_info.co_pid_mode][PIT].angle.kd = 0;
			break;
        
        case GIMBAL_MODE_AIM_ARMY:
            gim_pid[gim_info.co_pid_mode][YAW].speed.kp = aim_army_yaw_speed_kp;
			gim_pid[gim_info.co_pid_mode][YAW].speed.ki = aim_army_yaw_speed_ki;		
			gim_pid[gim_info.co_pid_mode][YAW].speed.kd = 0;
			gim_pid[gim_info.co_pid_mode][YAW].angle.kp = aim_army_yaw_angle_kp;		
			gim_pid[gim_info.co_pid_mode][YAW].angle.ki = 0;		
			gim_pid[gim_info.co_pid_mode][YAW].angle.kd = 0;

            gim_pid[gim_info.co_pid_mode][PIT].speed.kp = aim_army_pit_speed_kp;
			gim_pid[gim_info.co_pid_mode][PIT].speed.ki = aim_army_pit_speed_ki;
			gim_pid[gim_info.co_pid_mode][PIT].speed.kd = 0;
			gim_pid[gim_info.co_pid_mode][PIT].angle.kp = aim_army_pit_angle_kp;
			gim_pid[gim_info.co_pid_mode][PIT].angle.ki = 0;		
			gim_pid[gim_info.co_pid_mode][PIT].angle.kd = 0;
            break;
        
		default:
			break;
	}
}

void GIMBAL_Reset(void)
{
	float delta_angle;
	static uint16_t reset_ok_cnt = 0;
	static uint32_t time_prev;
	static uint32_t time_now;
	
	/* 刚进入复位 */
	if(flag.gimbal.reset_start == true) {
		// 记录上电时云台机械角度反馈值
		gim_pid[MECH][YAW].angle.target = gim_pid[MECH][YAW].angle.measure;
		gim_pid[MECH][PIT].angle.target = gim_pid[MECH][PIT].angle.measure;
		// 复位完成标志位和计数器清零
		reset_ok_cnt = 0;
		// 开始计时
		time_now = xTaskGetTickCount();
		time_prev = time_now;
		
		delta_angle = GIM_MECH_YAW_MID - gim_pid[MECH][YAW].angle.measure;
		/* 没有越界取最近 */
		if(abs(delta_angle) < 4096) {
			gim_pid[MECH][YAW].angle_ramp_target = delta_angle;
		} 
		/* 越界后作边界处理 */
		else {
			if(delta_angle >= 4096) {
				gim_pid[MECH][YAW].angle_ramp_target = -8192 + delta_angle;
			} 
			else if(delta_angle <= -4096) {
				gim_pid[MECH][YAW].angle_ramp_target = 8192 + delta_angle;
			}
		}			
		
		flag.gimbal.reset_start = false;
	}
	
	if(flag.gimbal.reset_ok == false) {
		// 复位计时
		time_now = xTaskGetTickCount();
		// 强制使用机械模式复位
		gim_info.co_pid_mode = MECH;
		/* 平缓地让云台移动到中间，防止上电狂甩
			斜坡函数给累加期望，防止突然增加很大的期望值 */
		// yaw
		if(gim_pid[MECH][YAW].angle_ramp_target > 0) {
			gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, GIM_RAMP_BEGIN_YAW, YAW);
		}
		else if(gim_pid[MECH][YAW].angle_ramp_target < 0) {
			gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, -GIM_RAMP_BEGIN_YAW, PIT);
		}
		
		gim_pid[MECH][YAW].angle_ramp_target = RampFloat(0, gim_pid[MECH][YAW].angle_ramp_target, GIM_RAMP_BEGIN_YAW);
		// pit
		gim_pid[MECH][PIT].angle.target = RampFloat(GIM_MECH_PIT_MID, gim_pid[MECH][PIT].angle.target, GIM_RAMP_BEGIN_PIT);
		
		/* 等待云台归中 */
		if(abs(gim_pid[MECH][YAW].angle.measure - GIM_MECH_YAW_MID) <= 1 
			&& abs(gim_pid[MECH][PIT].angle.measure - GIM_MECH_PIT_MID) <= 1) {
			reset_ok_cnt++;
		}
			
		if((reset_ok_cnt > 250) || ((time_now - time_prev) >= TIME_STAMP_5000MS)) {
			reset_ok_cnt = 0;
			flag.gimbal.reset_ok = true;
			/* 记录初始陀螺仪角度 */
			start_gyro_yaw = gim_pid[GYRO][YAW].angle.measure;
			last_gyro_yaw = start_gyro_yaw;
			delta_gyro_yaw = 0;
		}
	}
}

/* #遥控 ---------------------------------------------------------------------*/
void RC_SetGimbalAngle(void)
{
    float target_angle;
	if(gim_info.co_pid_mode == MECH) {
		// yaw 使用电机机械角度
        target_angle = RC_RIGH_CH_LR_VALUE * RC_GIM_MECH_YAW_SENS;
		gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, target_angle, YAW);
        gim_pid[MECH][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][YAW].angle, YAW);
		// pit 抬头机械角度减小
		target_angle = -RC_RIGH_CH_UD_VALUE * RC_GIM_MECH_PIT_SENS;
        gim_pid[MECH][PIT].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][PIT].angle, target_angle, PIT);
        gim_pid[MECH][PIT].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][PIT].angle, PIT);
	}
	else if(gim_info.co_pid_mode == GYRO) {
		// yaw 使用电机机械角度
		target_angle = RC_RIGH_CH_LR_VALUE * RC_GIM_GYRO_YAW_SENS;
		gim_pid[GYRO][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[GYRO][YAW].angle, target_angle, YAW);
        gim_pid[GYRO][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[GYRO][YAW].angle, YAW);
		// pit 陀螺仪角度
		target_angle = -RC_RIGH_CH_UD_VALUE * RC_GIM_GYRO_PIT_SENS;
        gim_pid[GYRO][PIT].angle.target += target_angle;
        gim_pid[GYRO][PIT].angle.target = GIMBAL_GyroAngleConstrain(&gim_pid[GYRO][PIT].angle, gim_dev.pit_motor->info, PIT);
	}
}

void KEY_SetGimbalAngle(void)
{
    float target_angle;
    
    if(KEY_SHIFT.state == PRESS)
    {
        if(gim_info.co_pid_mode == MECH) {
            // 键盘控制yaw电机机械角度
            if(KEY_A_TRIG == RELEASE_TO_PRESS) {
                gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, -8, YAW);
            }
            if(KEY_D_TRIG == RELEASE_TO_PRESS) {
                gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, +8, YAW);
            }
            gim_pid[MECH][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][YAW].angle, YAW);
            // 键盘控制pit电机机械角度
            if(KEY_W_TRIG == RELEASE_TO_PRESS) {
                gim_pid[MECH][PIT].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][PIT].angle, +6, PIT);
            }
            if(KEY_S_TRIG == RELEASE_TO_PRESS) {
                gim_pid[MECH][PIT].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][PIT].angle, -6, PIT);
            }
            gim_pid[MECH][PIT].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][PIT].angle, PIT);
        }
        else if(gim_info.co_pid_mode == GYRO) {
            // 键盘控制yaw电机角度
            // yaw 使用电机角度
            if(KEY_A_TRIG == RELEASE_TO_PRESS) {
                gim_pid[GYRO][YAW].angle.target -= 8;
            }
            if(KEY_D_TRIG == RELEASE_TO_PRESS) {
                gim_pid[GYRO][YAW].angle.target += 8;
            }        
            // 键盘控制pit电机陀螺仪角度
            if(KEY_W_TRIG == RELEASE_TO_PRESS) {
                gim_pid[GYRO][PIT].angle.target -= 0.25f;
            }
            if(KEY_S_TRIG == RELEASE_TO_PRESS) {
                gim_pid[GYRO][PIT].angle.target += 0.25f;
            }
            gim_pid[GYRO][PIT].angle.target = GIMBAL_GyroAngleConstrain(&gim_pid[GYRO][PIT].angle, gim_dev.pit_motor->info, PIT);
        }
    }
    else
    {
        if(gim_info.co_pid_mode == MECH) {
            // yaw 使用电机机械角度
            target_angle = KalmanFilter(&mouse_x_kalman_err, MOUSE_X_MOVE_SPEED * KEY_GIM_MECH_YAW_SENS);
            target_angle = MovingAverageFloat(&mouse_x_avr, target_angle);
            gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, target_angle, YAW);
            gim_pid[MECH][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][YAW].angle, YAW);
            // pit 抬头机械角度减小
            target_angle = KalmanFilter(&mouse_y_kalman_err, MOUSE_Y_MOVE_SPEED * KEY_GIM_MECH_PIT_SENS);
            target_angle = MovingAverageFloat(&mouse_y_avr, target_angle);
            gim_pid[MECH][PIT].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][PIT].angle, target_angle, PIT);
            gim_pid[MECH][PIT].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][PIT].angle, PIT);
        }
        else if(gim_info.co_pid_mode == GYRO) {
            // yaw 使用陀螺仪角度
            target_angle = KalmanFilter(&mouse_x_kalman_err, (float)MOUSE_X_MOVE_SPEED * KEY_GIM_GYRO_YAW_SENS);
            target_angle = MovingAverageFloat(&mouse_x_avr, target_angle);
            gim_pid[GYRO][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[GYRO][YAW].angle, target_angle, YAW);
            gim_pid[GYRO][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[GYRO][YAW].angle, YAW);
            // pit 陀螺仪角度
            target_angle = KalmanFilter(&mouse_y_kalman_err, (float)MOUSE_Y_MOVE_SPEED * KEY_GIM_GYRO_PIT_SENS);
            target_angle = MovingAverageFloat(&mouse_y_avr, target_angle);
            gim_pid[GYRO][PIT].angle.target += target_angle;
            gim_pid[GYRO][PIT].angle.target = GIMBAL_GyroAngleConstrain(&gim_pid[GYRO][PIT].angle, gim_dev.pit_motor->info, PIT);
        }
    }
}

/* 任务层 --------------------------------------------------------------------*/
void GIMBAL_Init(void)
{
	// 载入云台电机本地驱动
	gim_drv[YAW] = gim_dev.yaw_motor->driver;
	gim_drv[PIT] = gim_dev.pit_motor->driver;
	// 载入云台电机本地信息
	gim_motor[YAW] = gim_dev.yaw_motor;
	gim_motor[PIT] = gim_dev.pit_motor;
	// 卡尔曼滤波器初始化
	KalmanCreate(&gim_angle_kalman_err[MECH][YAW], 1, 40);
	KalmanCreate(&gim_angle_kalman_err[MECH][PIT], 1, 80);
	KalmanCreate(&gim_angle_kalman_err[GYRO][YAW], 1, 60);
	KalmanCreate(&gim_angle_kalman_err[GYRO][PIT], 1, 80);
    KalmanCreate(&imu_rate_kalman_err[0], 1, 5);
    KalmanCreate(&imu_rate_kalman_err[1], 1, 10);
    KalmanCreate(&imu_rate_kalman_err[2], 1, 5);
    KalmanCreate(&imu_pit_kalman_err, 1, 250);
    KalmanCreate(&rc_rud_kalman_err, 1, 60);
    KalmanCreate(&rc_rlr_kalman_err, 1, 40);
    KalmanCreate(&mouse_x_kalman_err, 1, 1200);
    KalmanCreate(&mouse_y_kalman_err, 1, 1200);
}

void GIMBAL_GetInfo(void)
{
	// 获取系统信息
	GIMBAL_GetSysInfo();
	// 获取裁判系统信息
	GIMBAL_GetJudgeInfo();
	// 更新控制器信息
	GIMBAL_UpdateController();
	// 计算自瞄预测信息
	// ..
    // 更新遥控信息
    GIMBAL_GetRcInfo();
}

void GIMBAL_SelfProtect(void)
{
	GIMBAL_PidParamsInit(gim_pid[MECH], GIMBAL_MOTOR_CNT);
	GIMBAL_PidParamsInit(gim_pid[GYRO], GIMBAL_MOTOR_CNT);
	GIMBAL_Stop(gim_pid[MECH]);
	GIMBAL_Stop(gim_pid[GYRO]);
	GIMBAL_GetInfo();
}

/**
 *	@brief	pid控制器最终输出
 */
uint8_t test_yaw_speed_pid = 0;
uint8_t test_pit_speed_pid = 0;
float   test_yaw_speed_max_target = 3500;//DEBUG:8000
float   test_pit_speed_max_target = 1500;//DEBUG:4000
float   js_yaw_angle_target;
float   js_yaw_angle_measure;
float   js_pit_angle_target;
float   js_pit_angle_measure;
void GIMBAL_PidCtrl(void)
{	
	// 更新云台与底盘的跟随模式
	co_pid_mode_t co_pid_mode = gim_info.co_pid_mode;

    // yaw角度环
    GIMBAL_Angle_PidCalc(gim_pid[co_pid_mode], YAW);        
	if(test_yaw_speed_pid == 0) {
        // 内环期望 = 外环输出
		gim_pid[co_pid_mode][YAW].speed.target = gim_pid[co_pid_mode][YAW].angle.out;
	} else {
        gim_pid[co_pid_mode][YAW].speed.target = KalmanFilter(&rc_rlr_kalman_err, RC_RIGH_CH_LR_VALUE/660.f * test_yaw_speed_max_target);
    }
	// yaw速度环
	GIMBAL_Speed_PidCalc(gim_pid[co_pid_mode], YAW);	

    // pit角度环
    GIMBAL_Angle_PidCalc(gim_pid[co_pid_mode], PIT);	
	if(test_pit_speed_pid == 0) {
        // 内环期望 = 外环输出
		gim_pid[co_pid_mode][PIT].speed.target = gim_pid[co_pid_mode][PIT].angle.out;
	} else {
        gim_pid[co_pid_mode][PIT].speed.target = KalmanFilter(&rc_rud_kalman_err, RC_RIGH_CH_UD_VALUE/660.f * test_pit_speed_max_target);
	}
    // pit速度环
	GIMBAL_Speed_PidCalc(gim_pid[co_pid_mode], PIT);

    js_yaw_angle_target = gim_pid[gim_info.co_pid_mode][YAW].angle.target;
    js_yaw_angle_measure = gim_pid[gim_info.co_pid_mode][YAW].angle.measure;
    js_pit_angle_target = gim_pid[gim_info.co_pid_mode][PIT].angle.target;
    js_pit_angle_measure = gim_pid[gim_info.co_pid_mode][PIT].angle.measure;
	
	// 云台电机输出响应
	GIMBAL_PidOut(gim_pid[co_pid_mode]);
}

void GIMBAL_NormalCtrl(void)
{
	KEY_SetGimbalAngle();
}

uint8_t vis_yaw_ramp_step = 8;
uint8_t vis_pit_ramp_step = 8;
float vis_yaw_err, vis_pit_err;
float vis_yaw_target, vis_pit_target;
//float gim_yaw_target, gim_pit_target;
float vis_k_yaw = 0.5f;
float vis_k_pit = 0.5f;
float vis_offset_yaw = 0.f;//12.f;
float vis_offset_pit = 0.f;
uint16_t vis_framedrop_cnt = 0;
uint8_t vis_lose_target = false;
void GIMBAL_AimStaticCtrl(void)
{
    vision_sensor_t *vis_sen = gim_dev.vision_sensor;
    
    if(flag.gimbal.prepare_auto)
    {
        flag.gimbal.prepare_auto = false;
        vis_yaw_target = gim_pid[MECH][YAW].angle.target;
        vis_pit_target = gim_pid[MECH][PIT].angle.target;
        vis_framedrop_cnt = 0;
        vis_lose_target = true;
    }
    
    if(vis_sen->get_flag_status(vis_sen, VIS_IF_DATA_UPDATE))
    {
        if(vis_sen->get_flag_status(vis_sen, VIS_IF_LOCK_TARGET))
        {
//            vis_yaw_err = vis_k_yaw * vis_sen->yaw_pixel_err(vis_sen) + vis_offset_yaw;
//            vis_pit_err = vis_k_pit * (-vis_sen->pit_pixel_err(vis_sen)) + vis_offset_pit;
            
//            vis_yaw_target = gim_pid[MECH][YAW].angle.measure + vis_yaw_err;
//            vis_pit_target = gim_pid[MECH][PIT].angle.measure + vis_pit_err;
            
            vis_yaw_target = vis_sen->info->rx_packet.rx_data.yaw + vis_offset_yaw;
            vis_pit_target = vis_sen->info->rx_packet.rx_data.pitch + vis_offset_pit;
            
            gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, vis_yaw_target - gim_pid[MECH][YAW].angle.target, YAW);
            gim_pid[MECH][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][YAW].angle, YAW);
            
            gim_pid[MECH][PIT].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][PIT].angle, vis_pit_target - gim_pid[MECH][PIT].angle.target, PIT);
            gim_pid[MECH][PIT].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][PIT].angle, PIT);
            
            vis_framedrop_cnt = 0;
            vis_lose_target = false;
        }
        else
        {
            vis_yaw_err = 0;
            vis_pit_err = 0;
            
            if(vis_framedrop_cnt < 20) {
                vis_framedrop_cnt++;
            }
        }
    }

    if((vis_framedrop_cnt >= 20) || (vis_sen->info->offline_cnt >= 400)) {
        vis_lose_target = true;
    }
    
    if(vis_lose_target == false) {
//        gim_pid[MECH][YAW].angle.target = RampFloat(vis_yaw_target, 
//                                                    gim_pid[MECH][YAW].angle.target, 
//                                                    vis_yaw_ramp_step);
//        
//        gim_pid[MECH][PIT].angle.target = RampFloat(vis_pit_target, 
//                                                    gim_pid[MECH][PIT].angle.target, 
//                                                    vis_pit_ramp_step);
//        
//        gim_pid[MECH][PIT].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][PIT].angle, PIT);

        if(KEY_W_TRIG == RELEASE_TO_PRESS) {
            vis_offset_pit -= 6;
        }
        if(KEY_S_TRIG == RELEASE_TO_PRESS) {
            vis_offset_pit += 6;
        }
        if(KEY_A_TRIG == RELEASE_TO_PRESS) {
            vis_offset_yaw -= 8;
        }
        if(KEY_D_TRIG == RELEASE_TO_PRESS) {
            vis_offset_yaw += 8;
        }
    } else {
        // 确认丢失目标后可自由移动云台
//        KEY_SetGimbalAngle();
        // 意外情况无法识别时，仍然可以通过键盘移动云台
        if(KEY_W_TRIG == RELEASE_TO_PRESS) {
            gim_pid[MECH][PIT].angle.target -= 3;
        }
        if(KEY_S_TRIG == RELEASE_TO_PRESS) {
            gim_pid[MECH][PIT].angle.target += 3;
        }
        if(KEY_A_TRIG == RELEASE_TO_PRESS) {
            gim_pid[MECH][YAW].angle.target -= 3;
        }
        if(KEY_D_TRIG == RELEASE_TO_PRESS) {
            gim_pid[MECH][YAW].angle.target += 3;
        }
        
        gim_pid[MECH][YAW].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][YAW].angle, 0, YAW);
        gim_pid[MECH][YAW].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][YAW].angle, YAW);
        
        gim_pid[MECH][PIT].angle.target = GIMBAL_GetRealMechAngleTarget(&gim_pid[MECH][PIT].angle, 0, PIT);
        gim_pid[MECH][PIT].angle.target = GIMBAL_MechAngleConstrain(&gim_pid[MECH][PIT].angle, PIT);
    }
}

void GIMBAL_AimArmyCtrl(void)
{
    GIMBAL_AimStaticCtrl();
}

/**
 *	@brief	调试过程中使用遥控控制
 */
void GIMBAL_RcCtrl(void)
{
    switch(gim_info.local_mode)
	{
		case GIMBAL_MODE_NORMAL:	
			RC_SetGimbalAngle();
			break;
		case GIMBAL_MODE_AIM_OUTPOST:
			GIMBAL_AimStaticCtrl();
			break;
        case GIMBAL_MODE_AIM_ARMY:
            GIMBAL_AimArmyCtrl();
		default:
			break;
	}
}

/**
 *	@brief	正式比赛中使用键鼠控制
 */
void GIMBAL_KeyCtrl(void)
{
	switch(gim_info.local_mode)
	{
		case GIMBAL_MODE_NORMAL:	
			GIMBAL_NormalCtrl();
			break;
		case GIMBAL_MODE_AIM_OUTPOST:
			GIMBAL_AimStaticCtrl();
			break;
        case GIMBAL_MODE_AIM_ARMY:
            GIMBAL_AimArmyCtrl();
		default:
			break;
	}
}

void GIMBAL_Ctrl(void)
{
	/*----信息读入----*/
	GIMBAL_GetInfo();
	/*----期望修改----*/
	if(flag.gimbal.reset_ok == false) {
        GIMBAL_Reset();
	}
	else {
		if(gim_info.remote_mode == RC) {
			GIMBAL_RcCtrl();
		}
		else if(gim_info.remote_mode == KEY) {
			GIMBAL_KeyCtrl();
		}
	}
	/* 根据云台模式切换PID参数 */
	GIMBAL_PidParamsSwitch();	
	/*----最终输出----*/
	GIMBAL_PidCtrl();
}

void GIMBAL_Test(void)
{
    /*----信息读入----*/
	GIMBAL_GetInfo();
	/*----期望修改----*/
	if(flag.gimbal.reset_ok == false) {
		// 记录上电时云台机械角度反馈值
		gim_pid[MECH][YAW].angle.target = gim_pid[MECH][YAW].angle.measure;
		gim_pid[MECH][PIT].angle.target = gim_pid[MECH][PIT].angle.measure;
        flag.gimbal.reset_ok = true;
	}
	else {
		if(gim_info.remote_mode == RC) {
			GIMBAL_RcCtrl();
		}
		else if(gim_info.remote_mode == KEY) {
			GIMBAL_KeyCtrl();
		}
	}
    
    /* 根据云台模式切换PID参数 */
	GIMBAL_PidParamsSwitch();    
    
	/*----最终输出----*/
	GIMBAL_PidCtrl();    
}
