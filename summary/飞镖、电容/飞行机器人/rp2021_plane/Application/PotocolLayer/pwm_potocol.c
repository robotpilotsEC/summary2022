/**
 * @file        pwm_potocol.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        3-November-2020
 * @brief       PWM Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "pwm_potocol.h"

#include "drv_tim.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PWM_Output(drv_pwm_t *drv, int16_t pwm)
{
	if(drv->type == DRV_PWM_FRIC_L) {
		FRIC_PWM_L = pwm + 1000;
	}
	else if(drv->type == DRV_PWM_FRIC_R) {
		FRIC_PWM_R = pwm + 1000;
	}
	else if(drv->type == DRV_PWM_SERVO) {
		SERVO_PWM = pwm;
	}
}
