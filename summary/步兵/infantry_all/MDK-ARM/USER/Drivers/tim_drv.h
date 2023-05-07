#ifndef __DRV_TIM_H
#define __DRV_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/

#define SERVO_PWM	  TIM1->CCR2


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PWM_Init(void);
void COVER_PwmOut(int16_t pwm);
void COVER_SLEEP(void);
void COVER_WEAK(void);
#endif
