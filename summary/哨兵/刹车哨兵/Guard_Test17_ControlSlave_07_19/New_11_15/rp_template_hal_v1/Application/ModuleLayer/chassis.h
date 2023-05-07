#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
#define LEFT -1
#define RIGHT 1

#define INITIAL_SPEED 1000
#define NORMALl_SPEED 3000
#define MID_SPEED 5000
#define HIGH_SPEED 7000

#define R_DRIVING_GEAR 0.07f
/* Exported types ------------------------------------------------------------*/
typedef enum {
	RUN_NORMAL, // 正常巡航
	RUN_COORDINATE, // 坐标模式
	RUN_MIXED,     //混合模式
} chassis_mode_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	uint8_t mode;
	float		out;
} chassis_motor_pid_t;

//typedef struct {
//	pid_ctrl_t	angle;
//	float 		out;
//} chassis_z_pid_t;

typedef struct {
	chassis_motor_pid_t		(*motor)[CHASSIS_MOTOR_CNT];
} chassis_ctrl_t;

typedef struct {
	motor_t	*chas_motor[CHASSIS_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;

typedef struct {
	remote_mode_t		remote_mode;
	chassis_mode_t		local_mode;
}chassis_info_t;

typedef struct chassis{
	chassis_ctrl_t	*controller;
	chassis_dev_t	*dev;
	chassis_info_t	*info;
	bool init_mileage;
	bool brake_check;
	bool brake_start;
	bool brake_finished;
	int32_t rail_mileage;
	int8_t direction;
	int8_t run_dir;
	bool      spare_runnig;  //备用跑轨，刹车机构失灵下启用
	bool			test_open;
	bool      check_road;      //云台手控制状态，查看公路
	bool      escape;           //光速跑路
	bool      ctrl_sentry;       //接管哨兵
	bool      test_sentry;      //测试哨兵
	int8_t    control_dir;           //-1和1，-1向左
	bool      hurt_data_update;   //伤害更新
	chassis_safety_status_t safety_status;

	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}chassis_t;

extern chassis_t chassis;

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

#endif
