/**
 * @file        drv_tim.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        23-August-2020
 * @brief       TIMER Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/* Private macro -------------------------------------------------------------*/
#define MAX_PWM_VALUE 800

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
uint8_t standard_flg = 0;
/* Private functions ---------------------------------------------------------*/
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
//void COVER_PwmOut(int16_t pwm);

/* Exported functions --------------------------------------------------------*/
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
}

void ENCODER_Init(void)
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);	
}

/**
 *	@brief	���У׼�г�
 *  @param  У׼��־��1��0
 *  @return У׼��ɷ���true�����򷵻�false
 */
bool ESC_Standard(uint8_t *flg)
{
	static uint16_t standard_cnt = 0;   //У׼��ʱ����
	static uint16_t cnt = 0;    //��������ʱ����
	
	if(*flg)
	{

		if(cnt < 4000)  //�ȸ�������4s
		{
			FRICTION_PwmOut(MAX_PWM_VALUE,MAX_PWM_VALUE);
		}
		else           //ÿ1ms���ν��ͣ�ֱ����С���
		{
			cnt = 4001;
			
			FRICTION_PwmOut(MAX_PWM_VALUE - standard_cnt, MAX_PWM_VALUE - standard_cnt);
			
			standard_cnt++;
			if(standard_cnt > MAX_PWM_VALUE)   //�ﵽ��С���
			{
				standard_cnt = 0;
				cnt = 0;
				*flg = 0;
				return true;
			}
		}
		
		cnt++;
	}
	
	return false;
}

void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2)
{
	FRIC_PWM_L = pwm1 + 1000;
	FRIC_PWM_R = pwm2 + 1000;
//	FRIC_PWM_R1	= pwm2 + 1000;       //PB0���
}

