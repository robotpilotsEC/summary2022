/**
 * @file        can_protocol.c
 * @author      RobotPilots
 * @Version     v1.1.2
 * @brief       CAN Protocol.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  1.�޸�can_potocol.c/.h->can_protocol.c/.h
 *              v1.1.1(8-November-2021)
 *                  1.����can������Ϣ����Э�� 
 *              v1.1.2(13-November-2021)
 *                  1.ȥ��add_halfword��add_word�����������ǿ��ת��������ԭ��
 *                    add_word�д���ת����������⣬�����޸�
 */
 
/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"

#include "drv_can.h"
#include "motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	CAN �����Ϣ
 */
void CAN_AddMsg(drv_can_t *drv, uint8_t *data, uint8_t data_cnt)
{
    CAN_AddMsgByDriver(drv, data, data_cnt);
}

/**
 *	@brief	CAN ��ӵ��ֽ�����(Byte)
 */
void CAN_AddByte(drv_can_t *drv, uint8_t byte)
{
    CAN_AddMsgByDriver(drv, &byte, 1);
}

/**
 *	@brief	CAN ��Ӱ�������(HalfWord)
 */
void CAN_AddHalfWord(drv_can_t *drv, uint16_t hword)
{
    uint8_t data[2];
    data[0] = (uint8_t)(hword >> 8);
    data[1] = (uint8_t)(hword);
	CAN_AddMsgByDriver(drv, data, 2);
}

/**
 *	@brief	CAN ���������(Word)
 */
void CAN_AddWord(drv_can_t *drv, uint32_t word)
{
    uint8_t data[4];
    data[0] = (uint8_t)(word >> 24);
    data[1] = (uint8_t)(word >> 16);
    data[2] = (uint8_t)(word >> 8);
    data[3] = (uint8_t)(word);
	CAN_AddMsgByDriver(drv, data, 4);
}

/**
*	@brief	CAN �������ͱ���
 */
void CAN_ManualTx(drv_can_t *drv, uint8_t *data)
{
    CAN_StartTxByDriver(drv, data);
}

/**
 *	@brief	CAN1 ��������
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	/* ���ͨ�� */
	if(rxId == DATA_BY_MASTER_CAN_ID)
	{
		// �����յ�����̨������
		master_sensor.update(&master_sensor, rxBuf);
		master_sensor.check(&master_sensor);
	}	
	else if(rxId == DATA_BY_RC_MASTER_CAN_ID)
	{
		// �����յ�����̨������
		rc_master_sensor.update(&rc_master_sensor, rxBuf);
		rc_master_sensor.check(&rc_master_sensor);
	}	
}

/**
 *	@brief	CAN2 ��������
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	/* ��̨YAW��6020 */
	if(rxId == GIMBAL_CAN_ID_YAW)
	{
		// ���µ��̵������
		motor[GIMB_YAW].update(&motor[GIMB_YAW], rxBuf);
		motor[GIMB_YAW].check(&motor[GIMB_YAW]);
	}
	/* ��̨PITCH��6020 */
	else if(rxId == GIMBAL_CAN_ID_PITCH)
	{
		// ���µ��̵������
		motor[GIMB_PITCH].update(&motor[GIMB_PITCH], rxBuf);
		motor[GIMB_PITCH].check(&motor[GIMB_PITCH]);
	}
	/* ����2006 */
	else if(rxId == DIAL_CAN_ID)
	{
		// ���µ��̵������
		motor[DIAL].update(&motor[DIAL], rxBuf);
		motor[DIAL].check(&motor[DIAL]);
	}
	/* ��Ħ����3508 */
	else if(rxId == FRICTION_CAN_ID_L)
	{
		// ���µ��̵������
		motor[FRICTION_L].update(&motor[FRICTION_L], rxBuf);
		motor[FRICTION_L].check(&motor[FRICTION_L]);
	}
	/* ��Ħ����3508 */
	else if(rxId == FRICTION_CAN_ID_R)
	{
		// ���µ��̵������
		motor[FRICTION_R].update(&motor[FRICTION_R], rxBuf);
		motor[FRICTION_R].check(&motor[FRICTION_R]);
	}
	
}

