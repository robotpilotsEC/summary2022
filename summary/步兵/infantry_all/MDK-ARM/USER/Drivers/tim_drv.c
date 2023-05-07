/**
 * @file        drv_tim.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        23-August-2020
 * @brief       TIMER Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "tim_drv.h"
#include "main.h"
#include "io_drv.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/
void PWM_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	COVER_PwmOut(0);
}

//168000000 1680 1000 100hz Õ¼¿Õ±Èx/1000
//PE11
void COVER_PwmOut(int16_t pwm)
{
	SERVO_PWM = pwm;
}


void COVER_SLEEP(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}	

void COVER_WEAK(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
