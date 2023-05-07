/**
 * @file        rp_user_config.h
 * @author      RobotPilots
 * @Version     v1.0
 * @brief       RobotPilots Robots' User Configuration.
 * @update
 *              v1.0(7-November-2021)  
 */
#ifndef __RP_USER_CONFIG_H
#define __RP_USER_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* Exported macro ------------------------------------------------------------*/
#define EVENT_SEND_CAN1_MAILBOX (0x1UL << 0)
#define EVENT_SEND_CAN2_MAILBOX (0x1UL << 1)

#define SPEED_PID 0
#define POSITION_PID 1
/* Exported types ------------------------------------------------------------*/
/* 应用层 --------------------------------------------------------------------*/
/**
 *	@brief	pid控制器
 *	@class	controller
 */
typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float	integral;
	float 	integral_max;
	float 	out_max;
} pid_ctrl_t;

/* Remote Mode Enum */
typedef enum {
	RC = 0,
	KEY = 1,
	AUTO = 2,        //自动模式
	INSPECTION = 3,  //自检模式
	REMOTE_MODE_CNT = 4,
} remote_mode_t;

typedef enum {
	SYS_STATE_NORMAL,	// 系统正常
	SYS_STATE_RCLOST,	// 遥控失联
	SYS_STATE_RCERR,	// 遥控出错
	SYS_STATE_WRONG,	// 其它系统错误
} sys_state_t;

typedef enum {
	SYS_MODE_NORMAL,	// 常规模式
	SYS_MODE_CNT,
} sys_mode_t;

typedef struct {
	struct {
		uint8_t reset_start;
		uint8_t reset_ok;
	}gimbal;
} flag_t;

typedef struct {
	remote_mode_t		remote_mode;	// 控制方式
	sys_state_t			state;			// 系统状态
	sys_mode_t			mode;			// 系统模式
	bool switch_auto;         //模式切换
	bool switch_inspection;
	bool switch_rc;
} system_t;

typedef enum {
	FIRE_ON,     //允许开火
	FIRE_STOP,   //停止开火
} fire_judge_t;

typedef enum {
	FRIC_CLOSE = 0,     //摩擦轮关闭
	FRIC_OPEN,         //摩擦轮开启
} fric_ctrl_t;

typedef enum {
	DIAL_LOCK = 0,       //拨盘锁定
	DIAL_UNLOCK,         //拨盘解锁
} dial_lock_t;

typedef enum {
	SAFETY = 0,     //安全
	DANGER,         //危险
	HUGE_DANGER,    //非常危险
} chassis_safety_status_t;  

extern flag_t	flag;
extern system_t sys;

#endif
