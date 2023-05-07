#ifndef __DRV_TIM_H
#define __DRV_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* Exported macro ------------------------------------------------------------*/
#define FRIC_PWM_L	TIM3->CCR1  //PA6   ио
#define FRIC_PWM_R	TIM3->CCR2  //PA7    об
//#define FRIC_PWM_R1	TIM3->CCR3

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern uint8_t standard_flg;

/* Exported functions --------------------------------------------------------*/
void PWM_Init(void);
void ENCODER_Init(void);
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
bool ESC_Standard(uint8_t *flg);
#endif
