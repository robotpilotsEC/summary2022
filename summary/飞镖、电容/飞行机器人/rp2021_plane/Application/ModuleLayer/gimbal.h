#ifndef __GIMBAL_H
#define __GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "gimbal_motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"
#include "vision_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	GIMBAL_MODE_NORMAL 		= 0, // 正常模式
	GIMBAL_MODE_AIM_ARMY    = 1, // 自瞄装甲板
    GIMBAL_MODE_AIM_OUTPOST = 2, // 自瞄前哨站绿灯
} gimbal_mode_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		angle_ramp_target;
	float		out;
} gimbal_pid_t;

typedef struct {
	gimbal_pid_t	*mech_yaw;
	gimbal_pid_t	*mech_pit;
	gimbal_pid_t	*gyro_yaw;
	gimbal_pid_t	*gyro_pit;
} gimbal_ctrl_t;

typedef struct {
	gimbal_motor_t	*yaw_motor;
	gimbal_motor_t	*pit_motor;
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
    vision_sensor_t *vision_sensor;
} gimbal_dev_t;

typedef struct {
	remote_mode_t	remote_mode;
	co_pid_mode_t	co_pid_mode;
	gimbal_mode_t	local_mode;
}gimbal_info_t;

typedef struct gimbal{
	gimbal_ctrl_t	*controller;
	gimbal_dev_t	*dev;
	gimbal_info_t	*info;
	void			(*init)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
}gimbal_t;

extern gimbal_t gimbal;
/* Exported functions --------------------------------------------------------*/

#endif
