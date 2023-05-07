#ifndef __DIAL_H
#define __DIAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
#define One_Shot_17mm 10
#define COOLING_LIMIT 320

#define FIRING_RATE_LOW   345
#define FIRING_RATE_MID   400
#define FIRING_RATE_HIGH  660    /*26m射速660*/ 
#define SHOOT_FREQ_ONE    270
#define SHOOT_FREQ_LOW    1080   /*4射频*/
#define SHOOT_FREQ_MID    2160   /*8射频*/
#define	SHOOT_FREQ_TEN    2700	/*10射频*/
#define SHOOT_FREQ_TWELVE 3240   /*12射频*/
#define SHOOT_FREQ_HIGH   4320   /*16射频*/
#define SHOOT_FREQ_HEATLIMIT  6480  /*24射频*/
#define SHOOT_FREQ_VERYHIGH   8640   /*32射频*/

#define SHOOT_SINGLE_ANGLE 36859.5f
#define SHOOT_TRIPLE_ANGLE (SHOOT_SINGLE_ANGLE * 3)

/*单发一格的角度*/
#define SINGLE_ANGLE   36859.5f
/*连发拨盘的转速*/
#define CONTIN_FREQ_20  20
#define CONTIN_FREQ_15  15
#define CONTIN_FREQ_12  12
#define CONTIN_FREQ_10  10  //射频  （发/秒）
#define CONTIN_FREQ_05   5
#define CONTIN_FREQ_03   3

#define MOTOR_1RPS_SPEED     2160          //60*36一秒转一圈1RPM*60 *36(1：36减速比)= 1RPS
#define GRID_NUM    8    //格子数
#define MOTOR_1GPS_SPEED   (MOTOR_1RPS_SPEED/GRID_NUM)//一秒转一格GPS: Grid Per Second 

#define MOTOR_SPEED_20  (CONTIN_FREQ_20 * MOTOR_1GPS_SPEED)   //固定射频的连发电机转速
#define MOTOR_SPEED_15  (CONTIN_FREQ_15 * MOTOR_1GPS_SPEED)   //固定射频的连发电机转速
#define MOTOR_SPEED_10  (CONTIN_FREQ_10 * MOTOR_1GPS_SPEED)   //固定射频的连发电机转速
#define MOTOR_SPEED_05  (CONTIN_FREQ_05 * MOTOR_1GPS_SPEED)   //固定射频的连发电机转速
#define MOTOR_SPEED_03  (CONTIN_FREQ_03 * MOTOR_1GPS_SPEED)   //固定射频的连发电机转速

/* Exported types ------------------------------------------------------------*/
typedef struct {
	float Target;
} Friction_t;

typedef enum {
	DIAL_MODE_NORMAL 		= 0, // 正常模式(单发)
	DIAL_MODE_TRIPLE_SHOT = 1, // 三连发模式
	DIAL_MODE_REPEATING	= 2, // 连发模式
} dial_mode_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} dial_motor_pid_t;


typedef struct {
	dial_motor_pid_t		(*motor)[DIAL_MOTOR_CNT];
} dial_ctrl_t;

typedef struct {
	motor_t	*dia_motor[DIAL_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} dial_dev_t;

typedef struct {
	remote_mode_t		remote_mode;
	dial_mode_t		local_mode;
	volatile bool	    	normal_init_flag;
	volatile bool	    	triple_init_flag;
	volatile bool	    	repeating_init_flag;
	volatile bool	    	stuck_init_flag;
}dial_info_t;

typedef struct dial{
	dial_ctrl_t	*controller;
	dial_dev_t	*dev;
	dial_info_t	*info;
	bool			test_open;
	bool      fire_open;     //开火开关
	bool      fric_open;     //摩擦轮开关
	bool      fric_unlock;   //摩擦轮解锁
	bool      access;       //枪口热量有关允许位	
	bool      stop_fire;   //云台手控制开火
	bool      lock;        //拨盘锁定
	bool      stuck;      //拨盘卡弹
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}dial_t;

extern dial_t dial;

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

#endif
