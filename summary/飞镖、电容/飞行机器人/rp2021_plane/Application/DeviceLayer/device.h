#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rc_sensor.h"
#include "imu_sensor.h"
#include "judge_sensor.h"
#include "vision_sensor.h"
#include "gimbal_motor.h"
#include "turnplate_motor.h"
#include "fric_motor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
	judge_sensor_t		*judge_sen;
	vision_sensor_t		*vision_sen;
	gimbal_motor_t		*gim_mtr[GIMBAL_MOTOR_CNT];
	turnplate_motor_t	*tplt_mtr;
	fric_motor_t		*fric_mtr[FRIC_MOTOR_CNT];
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
