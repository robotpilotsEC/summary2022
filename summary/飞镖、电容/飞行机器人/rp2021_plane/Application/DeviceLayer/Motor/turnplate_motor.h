#ifndef __TURNPLATE_MOTOR_H
#define __TURNPLATE_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct turnplate_motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	int16_t		torque;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
}turnplate_motor_info_t;

typedef struct turnplate_motor_struct {
	turnplate_motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct turnplate_motor_struct *self);
	void					(*update)(struct turnplate_motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct turnplate_motor_struct *self);	
	void					(*heart_beat)(struct turnplate_motor_struct *self);
	dev_work_state_t		work_state;	
	dev_errno_t				errno;
	dev_id_t				id;
}turnplate_motor_t;

extern turnplate_motor_t turnplate_motor;

/* Exported functions --------------------------------------------------------*/
void turnplate_motor_test(void);

#endif
