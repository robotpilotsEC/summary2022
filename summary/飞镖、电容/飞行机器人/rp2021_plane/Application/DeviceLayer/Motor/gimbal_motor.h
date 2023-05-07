#ifndef __GIMBAL_MOTOR_H
#define __GIMBAL_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct gimbal_motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
} gimbal_motor_info_t;

typedef struct gimbal_motor_struct {
	gimbal_motor_info_t		*info;
	drv_can_t				*driver;
	void					(*init)(struct gimbal_motor_struct *self);
	void					(*update)(struct gimbal_motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct gimbal_motor_struct *self);	
	void					(*heart_beat)(struct gimbal_motor_struct *self);
	dev_work_state_t		work_state;			
	dev_errno_t				errno;
	dev_id_t				id;
} gimbal_motor_t;

extern gimbal_motor_t	gimbal_motor[GIMBAL_MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/

#endif
