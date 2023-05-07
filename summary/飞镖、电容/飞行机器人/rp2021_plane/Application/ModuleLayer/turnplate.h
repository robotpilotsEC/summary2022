#ifndef __TURNPLATE_H
#define __TURNPLATE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "turnplate_motor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	TURNPLATE_MODE_NORMAL,
	TURNPLATE_MODE_AUTO,
	TURNPLATE_MODE_BAN,
} turnplate_mode_t;

typedef enum {
	TPLT_POSI_PID,
	TPLT_SPEED_PID,
} turnplate_pid_mode_t;

typedef enum {
	TPLT_STATE_OFF,
	TPLT_STATE_ON,
} turnplate_state_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		angle_ramp_target;
	float		out;
} turnplate_pid_t;

typedef struct {
	turnplate_pid_t	*tplt;
} turnplate_ctrl_t;

typedef struct {
	turnplate_motor_t	*turnplate_motor;
	rc_sensor_t			*rc_sensor;
} turnplate_dev_t;

typedef struct {
	remote_mode_t			remote_mode;
	turnplate_mode_t		local_mode;
	turnplate_pid_mode_t	pid_mode;
	turnplate_state_t		state;
	uint16_t				stuck_cnt;
	uint16_t				heat_real;
	uint16_t				heat_limit;
	uint8_t					shoot;
	uint8_t					shoot_total_cnt;
	uint32_t				shoot_time;
	uint8_t					shoot_freq;
	uint16_t				shoot_interval;
	uint16_t				shoot_ping;
} turnplate_info_t;

typedef struct turnplate{
	turnplate_ctrl_t	*controller;
	turnplate_dev_t		*dev;
	turnplate_info_t	*info;
	uint8_t				test_open;
	void				(*init)(void);
	void				(*update)(void);
	void				(*test)(void);
	void				(*ctrl)(void);
	void				(*self_protect)(void);
	void				(*output)(void);
} turnplate_t;

extern turnplate_t turnplate;
/* Exported functions --------------------------------------------------------*/

#endif
