#ifndef __FRICWHEEL_H
#define __FRICWHEEL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "fric_motor.h"
#include "rc_sensor.h"
#include "judge_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	FRIC_MODE_NORMAL,
	FRIC_MODE_BAN,
} fric_mode_t;

typedef enum {
	FRIC_STATE_OFF,
	FRIC_STATE_ON,
} fric_state_t;

typedef enum {
	FRIC_LEVEL_LOW,
	FRIC_LEVEL_MID,
	FRIC_LEVEL_HIGH,
} fric_level_t;

typedef struct {
	struct {
		int16_t target;
		int16_t measure;
	} speed;
} fric_ramp_t;

typedef struct {
	fric_ramp_t		*fric;
} fric_ctrl_t;

typedef struct {
	fric_motor_t	*fric_motor[FRIC_MOTOR_CNT];
	rc_sensor_t		*rc_sensor;
    judge_sensor_t  *judge_sensor;
} fric_dev_t;

typedef struct {
	remote_mode_t	remote_mode;
	fric_mode_t		local_mode;
	fric_state_t	state;
	fric_level_t	level;
}fric_info_t;

typedef struct fric{
	fric_ctrl_t		*controller;
	fric_dev_t		*dev;
	fric_info_t		*info;
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
	bool			(*adjust)(uint8_t *);
	void			(*adapt)(void);
	void			(*output)(void);
	bool			(*if_open)(void);
	bool			(*if_ready)(void);	
}fric_t;

extern fric_t fric;
/* Exported functions --------------------------------------------------------*/

#endif
