#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rc_sensor.h"
#include "imu_sensor.h"
#include "motor.h"
#include "vision_sensor.h"

#include "vision_protocol.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
	motor_t		*mtr[MOTOR_CNT];
	vision_sensor_t *vision_sen;
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
