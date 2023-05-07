#ifndef __DRV_UART_H
#define __DRV_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rp_config.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void USART1_Init(void);
void USART2_Init(void);
void USART3_Init(void);
void USART4_Init(void);
void USART5_Init(void);
void USART_TxByte(drv_uart_t *drv, uint8_t txByte);
void USART_TxBuff(drv_uart_t *drv, uint8_t *txBuff, uint16_t txSize);
void USART_TxBuff_DMA(drv_uart_t *drv, uint8_t *txBuff, uint16_t txSize);

#endif
