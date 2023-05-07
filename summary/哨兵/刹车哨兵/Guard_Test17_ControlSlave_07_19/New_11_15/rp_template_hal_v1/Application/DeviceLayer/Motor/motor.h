/**
 * @file        chassis_motor.h
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       Chassis Motor(RM3508).
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(24-October-2021)
 *                  1.将init与update函数定义在本文件中
 *              v1.1.1(8-November-2021)
 *                  1.更新驱动函数
 */

#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct motor_info_struct {
	volatile uint16_t		angle;
	volatile int16_t		speed;
	volatile int16_t		current;
	volatile int16_t		torque;	
  volatile uint8_t    temperature;
	volatile uint16_t		angle_prev;
	volatile int32_t		angle_sum;
	volatile bool	    	init_flag;
	volatile uint8_t    offline_cnt;
	const    uint8_t		offline_max_cnt;	
} motor_info_t;

typedef struct motor_struct {
	motor_info_t        *info;
	drv_can_t			        *driver;
	void				        (*init)(struct motor_struct *self);
	void				        (*update)(struct motor_struct *self, uint8_t *rxBuf);
	void				        (*check)(struct motor_struct *self);	
	void					    (*heart_beat)(struct motor_struct *self);
	volatile dev_work_state_t   work_state;
	volatile dev_errno_t		errno;
	const    dev_id_t			id;
} motor_t;

extern motor_t motor[MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/


#endif
