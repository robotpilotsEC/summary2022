/**
 * @file        fric_motor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        3-November-2020
 * @brief       Friction Motor(SunnySky).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "fric_motor.h"

#include "pwm_potocol.h"
#include "device.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
drv_pwm_t	fric_motor_driver[FRIC_MOTOR_CNT] = {
	[FRIC_L] = {
		.type = DRV_PWM_FRIC_L,
		.output = PWM_Output,
	},
	[FRIC_R] = {
		.type = DRV_PWM_FRIC_R,
		.output = PWM_Output,
	},
};

fric_motor_t	fric_motor[FRIC_MOTOR_CNT] = {
	[FRIC_L] = {
		.info = NULL,
		.driver = &fric_motor_driver[FRIC_L],
		.work_state = DEV_ONLINE,
		.errno = NONE_ERR,
		.id = DEV_ID_FRIC_L,
	},
	[FRIC_R] = {
		.info = NULL,
		.driver = &fric_motor_driver[FRIC_R],
		.work_state = DEV_ONLINE,
		.errno = NONE_ERR,
		.id = DEV_ID_FRIC_R,		
	},
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


