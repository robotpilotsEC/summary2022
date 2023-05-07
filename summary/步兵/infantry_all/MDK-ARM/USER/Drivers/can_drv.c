/**
 * @file        drv_can.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        20-August-2020
 * @brief       CAN Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "can_drv.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig);
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);

//���ݴ���
__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);


/* ������can1 2 ���ͺͽ��յ���ȫ�ֱ���---------------------------------------------------------*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;
CAN_RmFrameTypeDef hcan1RmFrame;
CAN_RmFrameTypeDef hcan2RmFrame;
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	CAN ��ʶ����������λ��Ĭ������
 */
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// ������
	sFilterConfig->FilterMaskIdLow = 0;						// ������
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// ������������FIFO0
	sFilterConfig->FilterBank = 0;							// ���ù�����0
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
	sFilterConfig->SlaveStartFilterBank = 0;
}

/**
 *	@brief	CAN �����жϻص�����
 */
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		
		CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
	}
	else if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		
		CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
	}
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	CAN1 ��ʼ��
 */
void CAN1_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// ����CAN1
	HAL_CAN_Start(&hcan1);
}

/**
 *	@brief	CAN2 ��ʼ��
 */
void CAN2_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// ����CAN2
	HAL_CAN_Start(&hcan2);
}

/**
 *	@brief	ͨ��CAN��������
 */
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef *txFrame;
	
	if(hcan->Instance == CAN1)
		txFrame = &hcan1TxFrame;
	else if(hcan->Instance == CAN2)
		txFrame = &hcan2TxFrame;
	else
		return HAL_ERROR;
	//���ñ�����Ϣ
	txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 8;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame->data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txFrame->data[1] = (uint8_t)((int16_t)dat[0]);
	txFrame->data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txFrame->data[3] = (uint8_t)((int16_t)dat[1]);
	txFrame->data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txFrame->data[5] = (uint8_t)((int16_t)dat[2]);
	txFrame->data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txFrame->data[7] = (uint8_t)((int16_t)dat[3]);		
	//if�н����ķ��ͳ�ȥ
	if(HAL_CAN_AddTxMessage(hcan, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}



uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendData(&hcan1, stdId, dat);
}

uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendData(&hcan2, stdId, dat);
}

/**
 *	@brief	ͨ��CAN����yiban����
 */
uint8_t CAN_SendHalfData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef *txFrame;
	
	if(hcan->Instance == CAN1)
		txFrame = &hcan1TxFrame;
	else if(hcan->Instance == CAN2)
		txFrame = &hcan2TxFrame;
	else
		return HAL_ERROR;
	//���ñ�����Ϣ
	txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 4;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame->data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txFrame->data[1] = (uint8_t)((int16_t)dat[0]);
	txFrame->data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txFrame->data[3] = (uint8_t)((int16_t)dat[1]);
	
	//if�н����ķ��ͳ�ȥ
	if(HAL_CAN_AddTxMessage(hcan, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

uint8_t CAN1_SendHalfData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendHalfData(&hcan1, stdId, dat);
}

uint8_t CAN2_SendHalfData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendHalfData(&hcan2, stdId, dat);
}
/**
 *	@brief	ͨ��CAN����Զ��֡
 */
uint8_t CAN_Remote(CAN_HandleTypeDef *hcan, uint32_t stdId)
{
	uint8_t data[8];
	uint32_t txMailBox;
	CAN_RmFrameTypeDef *rmFrame;
	
	if(hcan->Instance == CAN1)
		rmFrame = &hcan1RmFrame;
	else if(hcan->Instance == CAN2)
		rmFrame = &hcan2RmFrame;
	else
		return HAL_ERROR;
	//���ñ�����Ϣ
	rmFrame->header.StdId = stdId;
	rmFrame->header.IDE = CAN_ID_STD;
	rmFrame->header.RTR = CAN_RTR_REMOTE;
	rmFrame->header.DLC = 0;	
	
	
	//if�н����ķ��ͳ�ȥ
	if(HAL_CAN_AddTxMessage(hcan, &rmFrame->header, &data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

uint8_t CAN1_Remote(uint32_t stdId)
{
	return CAN_Remote(&hcan1, stdId);
}

uint8_t CAN2_Remote(uint32_t stdId)
{
	return CAN_Remote(&hcan2, stdId);
}

/*************** ��Ϣ���ʹ��� ***************/

/**
 *	@brief	CAN ���͵�������
 */
void CAN_SendSingleData(drv_can_t *drv, int16_t txData)
{
	int16_t txArr[4] = {0, 0, 0, 0};
	
	txArr[drv->drv_id] = txData;
	if(drv->type == DRV_CAN1)
		CAN1_SendData(drv->std_id, txArr);
	else if(drv->type == DRV_CAN2)
		CAN2_SendData(drv->std_id, txArr);
}

/**
 *	@brief	CAN �������ݻ���
 */
void CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff)
{
	if(drv_type == DRV_CAN1)
		CAN1_SendData(std_id, txBuff);
	else if(drv_type == DRV_CAN2)
		CAN2_SendData(std_id, txBuff);
}






uint8_t HXZP_Tx_uint8(uint32_t std, uint8_t *data,char can,uint32_t DL)
{
	uint8_t i;
	CAN_TxFrameTypeDef TxMes;
	uint32_t CAN_Tx_Mailbox;
	
	TxMes.header.IDE = CAN_ID_STD;
	TxMes.header.RTR = CAN_RTR_DATA;
	TxMes.header.DLC = DL;    //
	TxMes.header.StdId = std; 	  //
	
	TxMes.data[0] = data[0];
	TxMes.data[1] = data[1];	
	TxMes.data[2] = data[2];	
	TxMes.data[3] = data[3];

	TxMes.data[4] = data[4];
	TxMes.data[5] = data[5];	
	TxMes.data[6] = data[6];	
	TxMes.data[7] = data[7];
	
	if(can == 1)i = HAL_CAN_AddTxMessage(&hcan1,&TxMes.header,TxMes.data,  &CAN_Tx_Mailbox);
	if(can == 2)i = HAL_CAN_AddTxMessage(&hcan2,&TxMes.header,TxMes.data,  &CAN_Tx_Mailbox);
	return i;
}



/*************** ��Ϣ���ʹ��� ***************/


/* Callback functions --------------------------------------------------------*/
/**
 *	@brief	��д CAN RxFifo �жϽ��պ���
 *	@note	��stm32f4xx_hal_can.c��������
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* CAN1 �����ж� */
	if(hcan->Instance == CAN1)
	{
		CAN_Rx_Callback(hcan);
		// HAL_CAN_DeactivateNotification
		// __HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	// ��ͣ����FIF00��Ϣ�Һ��жϣ�����Ϣ���������д�����ɺ���ʹ��
	}else
	if(hcan->Instance == CAN2)
	{
		CAN_Rx_Callback(hcan);
	}
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] ��Ҫ��  ��ʵ�־���� CAN1 ���ݴ���Э�� 
 */
__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}

/**
 *	@brief	[__WEAK] ��Ҫ��  ��ʵ�־���� CAN2 ���ݴ���Э��
 */
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}


