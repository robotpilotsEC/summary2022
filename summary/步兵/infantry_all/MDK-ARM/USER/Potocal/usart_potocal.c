/*
*	 usart 通信接口
*/

#include "usart_potocal.h"

#include  "vision_potocol.h"
#include  "vision_sensor.h"
#include "judge_sensor.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;


/**
 *	@brief	在串口1中解析遥控数据协议
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart1);
	// 更新视觉数据
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
}

/*
	 更新检查数据
*/
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart2);
 	Rc_Data(rxBuf);
}

///**
// *	@brief	在串口3中解析遥控数据协议
// */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart3);

	if((CARR_MODE == 4 && POSITION == 1)){
		
		vision_sensor.update(&vision_sensor, rxBuf);
		vision_sensor.check(&vision_sensor);
		
	}	
}

/*
	 更新检查数据
*/
void USART4_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart4);

}


/**
 *	@brief	在串口5中解析裁判系统数据协议
 */

void USART5_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart5);	
	
	if((CARR_MODE == 0 && POSITION == 1)){
		
		vision_sensor.update(&vision_sensor, rxBuf);
		vision_sensor.check(&vision_sensor);
		
	}
	else{
	
		judge_sensor.update(&judge_sensor, rxBuf);
		judge_sensor.check(&judge_sensor);
		
	}
}


