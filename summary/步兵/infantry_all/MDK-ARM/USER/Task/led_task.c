#include "led_task.h"//
#include "cmsis_os.h"//涉及osDELAY
#include "main.h"//涉及gpio
#include "io_drv.h"//涉及gpio
#include "Offline_check.h"

extern IWDG_HandleTypeDef hiwdg;
//C10 C11 C13 C14

void LED_CTRL(void);
	

/*
	只有第一个灯两秒周期闪烁：任务正常工作 无意外情况

	存在两灯长亮：遥控器失联

	存在两灯闪烁：存在电机失联
	
	同时按下ZXV复位芯片
	解决疯车等等

*/
uint16_t STUCK_CNT = 0;

uint32_t LED_CNT;
void Start_led(void const * argument)
{
  for(;;)
  {	
		STUCK_CNT = 0;
		
		HAL_IWDG_Refresh(&hiwdg);//131.072ms
		
		LED_CTRL();
		
		if(KEY_Z && KEY_X && KEY_V)
		{
			NVIC_SystemReset();
		}
		
	
		osDelay(1);
  }
}


void LED_CTRL(void)
{
	
	LED_CNT++;
	
#if POSITION == 1
		if(JUDGE_ONLINE && VISION_ONLINE && !RC_OFFLINE && !MOTOR.offline_num)
	  {
				
			if(((LED_CNT/100)%5 == 1)){
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);		
				PCout(11) = 1;
				PCout(13) = 1;
				PCout(14) = 1;
			}
			if(((LED_CNT/100)%5 == 2))HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		
			if(((LED_CNT/100)%5 == 3))HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);		
			if(((LED_CNT/100)%5 == 4))HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);		
				
		}
		else if(!RC_OFFLINE && !MOTOR.offline_num) 
		{
			if(!(LED_CNT%500)){
				
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);		
				PCout(11) = 1;
				PCout(13) = 1;
				PCout(14) = 1;
				
			}
		}
		else
		{
		
			if(RC_OFFLINE)
			{
				PCout(10) = 0;
				PCout(11) = 0;
			}
			else
			{
				PCout(10) = 1;
				PCout(11) = 1;
			}
			
			if(MOTOR.offline_num)
			{
				if(!(LED_CNT%250)){
					
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);		
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);	
					
				}					
			}
			else
			{
				PCout(13) = 1;
				PCout(14) = 1;
			}
		}

#endif
		
#if POSITION == 0		
		
		
		if(imu_sensor.work_state == DEV_OFFLINE &&!(LED_CNT%500)){
			
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);		
			
		}
		else 
		{
			if(((LED_CNT/100)%5 == 1)){
				
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);		
				PCout(11) = 1;
				PCout(13) = 1;
				PCout(14) = 1;
				
			}
			if(((LED_CNT/100)%5 == 2))HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		
			if(((LED_CNT/100)%5 == 3))HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);		
			if(((LED_CNT/100)%5 == 4))HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);		
		}
	

#endif
		
}


