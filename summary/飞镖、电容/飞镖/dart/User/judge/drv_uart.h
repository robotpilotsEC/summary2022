#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "stm32f4xx_hal.h"


void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
//void USART2_Init(void);
void USART5_Init(void);



#endif
