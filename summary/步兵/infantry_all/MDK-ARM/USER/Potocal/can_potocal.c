/**
 * @file        can_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       CAN Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "can_potocal.h"
#include "judge_infantrypotocol.h"


extern judge_sensor_t	judge_sensor;
void Judge_Feedback(uint32_t ID, judge_sensor_t *judge ,uint8_t *date);

/*************** 信息接收处理 ***************/
/**
 *	@brief	CAN1 接收数据 数据处理
 */

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
#if (CARR_MODE == 0 && POSITION == 1)
	
	Up_RX(canId, rxBuf);
	
#endif
	

	
	MOTOR_CAN1_RX(canId, rxBuf);
	
	CAP_RP_2022.updata(canId, rxBuf);
	
}


/**
 *	@brief	CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
#if POSITION == 1

	#if CARR_MODE == 1 || CARR_MODE == 2 || CARR_MODE == 3 || CARR_MODE == 4
	
		Up_RX(canId, rxBuf);	
	
	#endif	
	
#endif	

#if POSITION == 0

	#if CARR_MODE == 0
	
		MOTOR_CAN1_RX(canId, rxBuf);
	  
  #endif
	
	#if CARR_MODE == 0 || CARR_MODE == 2 || CARR_MODE == 3 || CARR_MODE == 4
	
	  Down_RX(canId, rxBuf);
	
	#endif	
	
#endif	
	
	MOTOR_CAN2_RX(canId, rxBuf);
	
	CAP_RP_2022.updata(canId, rxBuf);
}


