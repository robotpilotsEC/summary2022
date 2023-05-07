/**
 * @file        can_protocol.c
 * @author      RobotPilots
 * @Version     v1.1.2
 * @brief       CAN Protocol.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  1.修改can_potocol.c/.h->can_protocol.c/.h
 *              v1.1.1(8-November-2021)
 *                  1.新增can报文消息发送协议 
 *              v1.1.2(13-November-2021)
 *                  1.去掉add_halfword与add_word函数中冗余的强制转换，另外原先
 *                    add_word中存在转换错误的问题，现已修复
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
 *	@brief	CAN 添加消息
 */
void CAN_AddMsg(drv_can_t *drv, uint8_t *data, uint8_t data_cnt)
{
    CAN_AddMsgByDriver(drv, data, data_cnt);
}

/**
 *	@brief	CAN 添加单字节数据(Byte)
 */
void CAN_AddByte(drv_can_t *drv, uint8_t byte)
{
    CAN_AddMsgByDriver(drv, &byte, 1);
}

/**
 *	@brief	CAN 添加半字数据(HalfWord)
 */
void CAN_AddHalfWord(drv_can_t *drv, uint16_t hword)
{
    uint8_t data[2];
    data[0] = (uint8_t)(hword >> 8);
    data[1] = (uint8_t)(hword);
	CAN_AddMsgByDriver(drv, data, 2);
}

/**
 *	@brief	CAN 添加字数据(Word)
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
*	@brief	CAN 立即发送报文
 */
void CAN_ManualTx(drv_can_t *drv, uint8_t *data)
{
    CAN_StartTxByDriver(drv, data);
}

/**
 *	@brief	CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	/* 板间通信 */
	if(rxId == DATA_BY_MASTER_CAN_ID)
	{
		// 更新收到下云台的数据
		master_sensor.update(&master_sensor, rxBuf);
		master_sensor.check(&master_sensor);
	}	
	else if(rxId == DATA_BY_RC_MASTER_CAN_ID)
	{
		// 更新收到下云台的数据
		rc_master_sensor.update(&rc_master_sensor, rxBuf);
		rc_master_sensor.check(&rc_master_sensor);
	}	
}

/**
 *	@brief	CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	/* 云台YAW轴6020 */
	if(rxId == GIMBAL_CAN_ID_YAW)
	{
		// 更新底盘电机数据
		motor[GIMB_YAW].update(&motor[GIMB_YAW], rxBuf);
		motor[GIMB_YAW].check(&motor[GIMB_YAW]);
	}
	/* 云台PITCH轴6020 */
	else if(rxId == GIMBAL_CAN_ID_PITCH)
	{
		// 更新底盘电机数据
		motor[GIMB_PITCH].update(&motor[GIMB_PITCH], rxBuf);
		motor[GIMB_PITCH].check(&motor[GIMB_PITCH]);
	}
	/* 拨盘2006 */
	else if(rxId == DIAL_CAN_ID)
	{
		// 更新底盘电机数据
		motor[DIAL].update(&motor[DIAL], rxBuf);
		motor[DIAL].check(&motor[DIAL]);
	}
	/* 左摩擦轮3508 */
	else if(rxId == FRICTION_CAN_ID_L)
	{
		// 更新底盘电机数据
		motor[FRICTION_L].update(&motor[FRICTION_L], rxBuf);
		motor[FRICTION_L].check(&motor[FRICTION_L]);
	}
	/* 右摩擦轮3508 */
	else if(rxId == FRICTION_CAN_ID_R)
	{
		// 更新底盘电机数据
		motor[FRICTION_R].update(&motor[FRICTION_R], rxBuf);
		motor[FRICTION_R].check(&motor[FRICTION_R]);
	}
	
}

