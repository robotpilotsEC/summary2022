#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "path_sensor.h"
#include "rc_sensor.h"
#include "imu_sensor.h"
#include "motor.h"
#include "vision_sensor.h"
#include "judge_sensor.h"
#include "control_task.h"
#include "vision_protocol.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
	path_t   *path_sen;
	motor_t		*mtr[MOTOR_CNT];
	vision_sensor_t *vision_sen;
	judge_sensor_t      *judge_sen;
	leader_t *leader_sen;
	rc_leader_t *rc_leader_sen;
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
