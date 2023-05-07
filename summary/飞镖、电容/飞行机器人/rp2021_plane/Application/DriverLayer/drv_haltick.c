/**
 * @file        drv_haltick.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Haltick driver
 */
 
/* Includes ------------------------------------------------------------------*/
#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@note	
 *	使用cubemx生成FREERTOS后会建议将SYS的时基切换成除SysTick之外的定时器
 *	从而系统会存在两套时基，①用于RTOS的SysTick ②用于HAL的HalTick
 *	SysTick 使用cortex-m4内核的SysTick (SysTick->VAL会在启动任务调度器之后才更新)
 *	HalTick 在本工程里面使用TIM2 (TIM2->CNT可提供微妙级延时)
 *	# delay_us 和 delay_ms 不会引起任务调度
 */
uint32_t micros(void)
{
	uint32_t ms, us;

	ms = HAL_GetTick();
	// 选用定时器2作为HAL时基的TimeBase
	// Freq:1MHz => 1Tick = 1us
	// Period:1ms
	us = TIM2->CNT;
	
	return (ms*1000 + us);
}

uint32_t millis(void)
{
    uint32_t ms = HAL_GetTick();
    return ms;
}

void delay_us(uint32_t us)
{
	uint32_t now = micros();
	
	while((micros() - now) < us);
}

void delay_ms(uint32_t ms)
{
	while(ms--)
		delay_us(1000);
}


