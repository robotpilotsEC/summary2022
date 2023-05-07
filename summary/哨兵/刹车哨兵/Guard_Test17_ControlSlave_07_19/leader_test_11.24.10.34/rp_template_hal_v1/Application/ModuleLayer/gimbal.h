#ifndef __GIMBAL_H
#define __GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
#define GIMBAL_ANGLE_PITCH_MIN -288     //-157  -288   上
#define GIMBAL_ANGLE_PITCH_MAX 699 //481   //781  699  下

#define GIMBAL_ANGLE_PITCH_MIN_AUTO -288   //-180
#define GIMBAL_ANGLE_PITCH_MAX_AUTO 699   //250

#define GIMBAL_ANGLE_YAW_MIN -1000    //             -1028    左
#define GIMBAL_ANGLE_YAW_MAX 1100   //2314   //1352   1124 右

#define GIMBAL_ANGLE_MID_YAW 6471   //6489     //2329          6471
#define GIMBAL_ANGLE_MID_PITCH 7784  //3722   //5728  //1642   7784

#define Auto_Up 0
#define Auto_Down 1
#define Auto_Left 1
#define Auto_Right 0
/* Exported types ------------------------------------------------------------*/
typedef enum {
	GIMBAL_MODE_NORMAL 		= 0, // 正常模式
//	CHASSIS_MODE_BUFF   		= 1, // 打符模式
//	CHASSIS_MODE_RELOAD_BULLET	= 2, // 底盘低速补弹模式
//	CHASSIS_MODE_SZUPUP			= 3, // SZU爬坡模式
} gimbal_mode_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} gimbal_motor_pid_t;

typedef struct {
	gimbal_motor_pid_t		(*motor)[GIMBAL_MOTOR_CNT];
} gimbal_ctrl_t;

typedef struct {
	motor_t	*gimb_motor[GIMBAL_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} gimbal_dev_t;

typedef struct {
	remote_mode_t		remote_mode;
	gimbal_mode_t		local_mode;
}gimbal_info_t;

typedef struct gimbal{
	gimbal_ctrl_t	*controller;
	gimbal_dev_t	*dev;
	gimbal_info_t	*info;
	bool			test_open;
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}gimbal_t;

extern gimbal_t gimbal;
extern gimbal_motor_pid_t gimb_motor_pid[GIMBAL_MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

#endif
