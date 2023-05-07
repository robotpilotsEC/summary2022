/**
 * @file        device.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Devices' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen = &rc_sensor,
	.imu_sen = &imu_sensor,
	.vision_sen = &vision_sensor,
	.mtr[DIAL] = &motor[DIAL],
	.mtr[FRICTION_L] = &motor[FRICTION_L],
	.mtr[FRICTION_R] = &motor[FRICTION_R],
	.mtr[GIMB_PITCH] = &motor[GIMB_PITCH],
	.mtr[GIMB_YAW] = &motor[GIMB_YAW],
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEV_Init(void)
{
	dev_list.rc_sen->init(dev_list.rc_sen);
	dev_list.imu_sen->init(dev_list.imu_sen);
	dev_list.vision_sen->init(dev_list.vision_sen);
	dev_list.mtr[DIAL]->init(dev_list.mtr[DIAL]);
	dev_list.mtr[FRICTION_L]->init(dev_list.mtr[FRICTION_L]);
	dev_list.mtr[FRICTION_R]->init(dev_list.mtr[FRICTION_R]);
	dev_list.mtr[GIMB_PITCH]->init(dev_list.mtr[GIMB_PITCH]);
	dev_list.mtr[GIMB_YAW]->init(dev_list.mtr[GIMB_YAW]);
}
