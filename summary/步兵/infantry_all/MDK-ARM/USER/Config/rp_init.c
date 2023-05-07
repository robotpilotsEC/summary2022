#include "rp_init.h"
#include "judge_sensor.h"
#include "vision_potocol.h"
#include "vision_sensor.h"


void DRIVER_Init(void)
{
	

//	ADC_Init();
//	DAC_Init();
	USART2_Init();
	CAN1_Init();
	CAN2_Init();		
	
//	USART4_Init();
	PWM_Init();	
	
	USART5_Init();

	USART3_Init();
	USART1_Init();
	
	
}


void DEVICE_Init(void)
{	
	Init_Imu();
	Init_Rc();
	Init_Motor();
	Init_vision();
	
	
	judge_sensor.init(&judge_sensor);
	vision_sensor.init(&vision_sensor);
}


