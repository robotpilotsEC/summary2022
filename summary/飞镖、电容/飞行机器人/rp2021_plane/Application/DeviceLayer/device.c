/**
 * @file        device.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Devices' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen = &rc_sensor,
	.imu_sen = &imu_sensor,
	.judge_sen = &judge_sensor,
	.vision_sen = &vision_sensor,
	.gim_mtr[YAW] = &gimbal_motor[YAW],
	.gim_mtr[PIT] = &gimbal_motor[PIT],
	.tplt_mtr = &turnplate_motor,
	.fric_mtr[FRIC_L] = &fric_motor[FRIC_L],
	.fric_mtr[FRIC_R] = &fric_motor[FRIC_R],
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEV_Init(void)
{
	dev_list.rc_sen->init(dev_list.rc_sen);
    dev_list.imu_sen->init(dev_list.imu_sen);
	dev_list.judge_sen->init(dev_list.judge_sen);
	dev_list.vision_sen->init(dev_list.vision_sen);
	dev_list.gim_mtr[YAW]->init(dev_list.gim_mtr[YAW]);
	dev_list.gim_mtr[PIT]->init(dev_list.gim_mtr[PIT]);
	dev_list.tplt_mtr->init(dev_list.tplt_mtr);
}
