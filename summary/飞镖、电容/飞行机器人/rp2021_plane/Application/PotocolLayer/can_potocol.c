/**
 * @file        can_potocol.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       CAN Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "can_potocol.h"

#include "drv_can.h"
#include "gimbal_motor.h"
#include "turnplate_motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t can1_txmail[CAN_STDID_COUNT][4];
int16_t can2_txmail[CAN_STDID_COUNT][4];
uint8_t can1_txmail_refresh[CAN_STDID_COUNT];
uint8_t can2_txmail_refresh[CAN_STDID_COUNT];

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	从CAN报文中读取电机的位置反馈
 */
static uint16_t CAN_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = ((uint16_t)rxData[0] << 8| rxData[1]);
	return angle;
}

/**
 *	@brief	从CAN报文中读取电机的转子转速反馈
 */
static int16_t CAN_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = ((uint16_t)rxData[2] << 8| rxData[3]);
	return speed;
}

/**
 *	@brief	从CAN报文中读取电机的实际转矩电流反馈
 */
//static int16_t CAN_GetMotorCurrent(uint8_t *rxData)
//{
//	int16_t current;
//	current = ((int16_t)rxData[4] << 8 | rxData[5]);
//	return current;
//}

/**
 *	@brief	从CAN报文中读取电机的实际输出转矩
 */
static int16_t CAN_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((int16_t)rxData[4] << 8 | rxData[5]);
	return torque;
}

///**
// *	@brief	RM3508 CAN标识符
// */
//static uint32_t RM3508_GetStdId(drv_can_t *drv)
//{
//	if((drv->can_id - 0x201U) < 4)
//		return 0x200;
//	else
//		return 0x1FF;
//}

///**
// *	@brief	RM3508 CAN数据下标
// */
//static uint8_t RM3508_GetDrvId(drv_can_t *drv)
//{
//	return (drv->can_id - 0x201U)%4;
//}

/**
 *	@brief	GM6020 CAN标识符
 */
static uint32_t GM6020_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x205U) < 4)
		return 0x1FF;
	else
		return 0x2FF;
}

/**
 *	@brief	GM6020 CAN数据下标
 */
static uint8_t GM6020_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x205U)%4;
}

/**
 *	@brief	RM2006 CAN标识符
 */
static uint32_t RM2006_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x201U) < 4)
		return 0x200;
	else
		return 0x1FF;
}

/**
 *	@brief	RM2006 CAN数据下标
 */
static uint8_t RM2006_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x201U)%4;
}

///**
// *	@brief	GM3510 CAN标识符
// */
//static uint32_t GM3510_GetStdId(drv_can_t *drv)
//{
//    return 0x1FF;
//}

///**
// *	@brief	GM3510 CAN数据下标
// */
//static uint8_t GM3510_GetDrvId(drv_can_t *drv)
//{
//	return (drv->can_id - 0x205U)%4;
//}

/* Exported functions --------------------------------------------------------*/
void gimbal_motor_update(gimbal_motor_t *gim_motor, uint8_t *rxBuf)
{
	gimbal_motor_info_t *motor_info = gim_motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	
	motor_info->offline_cnt = 0;
}

void turnplate_motor_update(turnplate_motor_t *tplt_motor, uint8_t *rxBuf)
{
	turnplate_motor_info_t *motor_info = tplt_motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->torque = CAN_GetMotorTorque(rxBuf);
	
	motor_info->offline_cnt = 0;
}

void gimbal_motor_init(gimbal_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;

	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_GIMBAL_YAW) {
		drv_can->drv_id = GM6020_GetDrvId(drv_can);
		drv_can->std_id = GM6020_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_GIMBAL_PIT) {
		drv_can->drv_id = GM6020_GetDrvId(drv_can);
		drv_can->std_id = GM6020_GetStdId(drv_can);
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}

void turnplate_motor_init(turnplate_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;

	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_TURNPLATE) {
		drv_can->drv_id = RM2006_GetDrvId(drv_can);
		drv_can->std_id = RM2006_GetStdId(drv_can);
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}

/**
 *	@brief	CAN 发送单独数据
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
 *	@brief	CAN 发送数据缓冲
 */
void CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff)
{
	if(drv_type == DRV_CAN1)
		CAN1_SendData(std_id, txBuff);
	else if(drv_type == DRV_CAN2)
		CAN2_SendData(std_id, txBuff);
}

void CAN1_SendDataBuff(uint8_t std_id, int16_t *txBuff)
{
    switch(std_id)
    {
        case CAN_STDID_0x1FF:
            CAN1_SendData(0x1FF, txBuff);
            break;
        
        case CAN_STDID_0x200:
            CAN1_SendData(0x200, txBuff);
            break;
        
        case CAN_STDID_0x2FF:
            CAN1_SendData(0x2FF, txBuff);
            break;
        
        default:
            break;
    }
}

void CAN2_SendDataBuff(uint8_t std_id, int16_t *txBuff)
{
    switch(std_id)
    {
        case CAN_STDID_0x1FF:
            CAN2_SendData(0x1FF, txBuff);
            break;
        
        case CAN_STDID_0x200:
            CAN2_SendData(0x200, txBuff);
            break;
        
        case CAN_STDID_0x2FF:
            CAN2_SendData(0x2FF, txBuff);
            break;
        
        default:
            break;
    }
}

void CAN_AddTxMessage(drv_can_t *drv, int16_t txData)
{
//    int16_t (*txmail)[CAN_STDID_COUNT][4];
    uint8_t *txmail_refresh;
    
    //TODO:加入drv_can的errno
    if(drv->drv_id > 4)
        return;
    
    if(drv->type == DRV_CAN1) {
//        txmail = &can1_txmail;
        txmail_refresh = can1_txmail_refresh;
    }
    else if(drv->type == DRV_CAN2) {
//        txmail = &can2_txmail;
        txmail_refresh = can2_txmail_refresh;
    }
    else {
        return;
    }
    
    switch(drv->std_id)
    {
        case 0x1FF:
            if(drv->type == DRV_CAN1) {
                can1_txmail[CAN_STDID_0x1FF][drv->drv_id] = txData;            
            }
            else if(drv->type == DRV_CAN2) {
                can2_txmail[CAN_STDID_0x1FF][drv->drv_id] = txData;
            }
//            *txmail[CAN_STDID_0x1FF][drv->drv_id] = txData;
            txmail_refresh[CAN_STDID_0x1FF] = 1;
            break;

        case 0x200:
            if(drv->type == DRV_CAN1) {
                can1_txmail[CAN_STDID_0x200][drv->drv_id] = txData;            
            }
            else if(drv->type == DRV_CAN2) {
                can2_txmail[CAN_STDID_0x200][drv->drv_id] = txData;
            }
//            *txmail[CAN_STDID_0x200][drv->drv_id] = txData;
            txmail_refresh[CAN_STDID_0x200] = 1;
            break;
        
        case 0x2FF:
            if(drv->type == DRV_CAN1) {
                can1_txmail[CAN_STDID_0x2FF][drv->drv_id] = txData;            
            }
            else if(drv->type == DRV_CAN2) {
                can2_txmail[CAN_STDID_0x2FF][drv->drv_id] = txData;
            }            
//            *txmail[CAN_STDID_0x2FF][drv->drv_id] = txData;
            txmail_refresh[CAN_STDID_0x2FF] = 1;
            break;
        
        default:
            break;
    }
}

void CAN_StartTx(drv_can_t *drv)
{
    if(drv->type == DRV_CAN1)
    {
        for(uint8_t i = 0; i < CAN_STDID_COUNT; i++)
        {
            if(can1_txmail_refresh[i]) {
                CAN1_SendDataBuff(i, can1_txmail[i]);
                can1_txmail_refresh[i] = 0;
            }
        }
    }
    else if(drv->type == DRV_CAN2)
    {
        for(uint8_t i = 0; i < CAN_STDID_COUNT; i++)
        {
            if(can2_txmail_refresh[i]) {
                CAN2_SendDataBuff(i, can2_txmail[i]);
                can2_txmail_refresh[i] = 0;
            }
        }        
    }
}

/**
 *	@brief	CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	/* Yaw轴 */
	if(canId == GIMBAL_CAN_ID_YAW)
	{
		// 更新Yaw轴电机数据
		gimbal_motor[YAW].update(&gimbal_motor[YAW], rxBuf);
		gimbal_motor[YAW].check(&gimbal_motor[YAW]);
	}
	/* Pitch轴 */
	else if(canId == GIMBAL_CAN_ID_PIT)
	{
		// 更新Pitch轴电机数据
		gimbal_motor[PIT].update(&gimbal_motor[PIT], rxBuf);
		gimbal_motor[PIT].check(&gimbal_motor[PIT]);
	}
	else if(canId == TURNPLATE_CAN_ID)
	{
		// 更新拨盘电机数据
		turnplate_motor.update(&turnplate_motor, rxBuf);
		turnplate_motor.check(&turnplate_motor);		
	}    
}

/**
 *	@brief	CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}

