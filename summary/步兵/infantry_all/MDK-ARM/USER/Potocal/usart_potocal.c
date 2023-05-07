/*
*	 usart ͨ�Žӿ�
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
 *	@brief	�ڴ���1�н���ң������Э��
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart1);
	// �����Ӿ�����
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
}

/*
	 ���¼������
*/
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart2);
 	Rc_Data(rxBuf);
}

///**
// *	@brief	�ڴ���3�н���ң������Э��
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
	 ���¼������
*/
void USART4_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart4);

}


/**
 *	@brief	�ڴ���5�н�������ϵͳ����Э��
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


