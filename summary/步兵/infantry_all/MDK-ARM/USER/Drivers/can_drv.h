#ifndef __CAN_DRV_H
#define __CAN_DRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rp_config.h"
#include "rp_math.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;

typedef struct {
	CAN_TxHeaderTypeDef header;
} CAN_RmFrameTypeDef;


void CAN1_Init(void);
void CAN2_Init(void);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);

uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat);
uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat);
uint8_t CAN1_SendHalfData(uint32_t stdId, int16_t *dat);
uint8_t CAN2_SendHalfData(uint32_t stdId, int16_t *dat);
uint8_t CAN1_Remote(uint32_t stdId);
uint8_t CAN2_Remote(uint32_t stdId);

uint8_t CAN_Remote(CAN_HandleTypeDef *hcan, uint32_t stdId);
uint8_t HXZP_Tx_uint8(uint32_t std, uint8_t *data,char can,uint32_t DL);
void	CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void 	CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);

#endif
