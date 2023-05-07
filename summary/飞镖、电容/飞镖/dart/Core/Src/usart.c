/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "rc.h"
#include "judge_infantrypotocol.h"
#include "string.h"

#define USART2_RX_DATA_FRAME_LEN	(18u)	// 串口2数据帧长度
#define USART2_RX_BUF_LEN			(USART2_RX_DATA_FRAME_LEN + 6u)	// 串口2接收缓冲区长度
#define USART5_RX_BUF_LEN			200

//__WEAK void USART2_rxDataHandler(uint8_t *rxBuf);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);
                                                    
uint8_t usart2_dma_rxbuf[2][USART2_RX_BUF_LEN];
uint8_t usart5_dma_rxbuf[USART5_RX_BUF_LEN];

/* USER CODE END 0 */

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5_TX Init */
    hdma_uart5_tx.Instance = DMA1_Stream7;
    hdma_uart5_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_tx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart5_tx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA3     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA3     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	// 将当前目标内存设置为Memory1
	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
	USART2_rxDataHandler(usart2_dma_rxbuf[0]);
}

static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	// 将当前目标内存设置为Memory0
	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
	USART2_rxDataHandler(usart2_dma_rxbuf[1]);
}

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in DRV_UART_IRQHandler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);	
	/* handle received data in idle interrupt */
	if (huart == &huart2)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//uint32_t status = taskENTER_CRITICAL_FROM_ISR();
		if ((USART2_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART2_RX_DATA_FRAME_LEN)
		{
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);
		}
		//taskEXIT_CRITICAL_FROM_ISR(status);

		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART2_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}
    else if (huart == &huart5)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* handle dbus data dbus_buf from DMA */
		USART5_rxDataHandler(usart5_dma_rxbuf);
		memset(usart5_dma_rxbuf, 0, USART5_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
 *	@brief	start double buffer dma transfer with no interrupt
 */
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/* Set the UART DMA transfer complete callback */
	/* Current memory buffer used is Memory 1 callback */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Current memory buffer used is Memory 0 callback */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/* Check callback functions */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
	hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
	return HAL_ERROR;
	}
	
	/* Process locked */
	__HAL_LOCK(hdma);
	
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Enable the Double buffer mode */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* Configure DMA Stream destination address */
		hdma->Instance->M1AR = SecondMemAddress;		

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;		
		
		/* Peripheral to Memory */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Memory to Peripheral */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/* Clear TC flags */
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Enable TC interrupts*/
//		hdma->Instance->CR  |= DMA_IT_TC;
		
		/* Enable the peripheral */
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);	  

		/* Return error status */
		status = HAL_BUSY;		
	}
	/* Process unlocked */
	__HAL_UNLOCK(hdma);

	return status; 	
}

static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;

		/* Memory to Peripheral */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;//外设地址

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;//储存器
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	} 
	return status; 	
}

/* Exported functions --------------------------------------------------------*/
/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // 判断是否为空闲中断
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**
 *	@brief	USART2 Initialization
 */
void USART2_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);	
	
	DMAEx_MultiBufferStart_NoIT(huart2.hdmarx, \
							    (uint32_t)&huart2.Instance->DR, \
							    (uint32_t)usart2_dma_rxbuf[0], \
							    (uint32_t)usart2_dma_rxbuf[1], \
							    USART2_RX_DATA_FRAME_LEN);
}

/**
 *	@brief	USART5 Initialization
 */
void USART5_Init(void)//接收
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);	
	
	DMA_Start(huart5.hdmarx, \
			  (uint32_t)&huart5.Instance->DR, \
			  (uint32_t)usart5_dma_rxbuf, \
			  USART5_RX_BUF_LEN);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
