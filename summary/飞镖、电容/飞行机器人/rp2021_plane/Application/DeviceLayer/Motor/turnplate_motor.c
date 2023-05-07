/**
 * @file        tunrplate_motor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Turnplate Motor(RM2006).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "turnplate_motor.h"

#include "rp_math.h"
#include "can_potocol.h"
#include "device.h"

extern void turnplate_motor_update(turnplate_motor_t *motor, uint8_t *rxBuf);
extern void turnplate_motor_init(turnplate_motor_t *motor);

/* Private macro -------------------------------------------------------------*/
#define TPLT_OFFLINE_MAX_CNT    20
/* Private function prototypes -----------------------------------------------*/
static void turnplate_motor_check(turnplate_motor_t *motor);
static void turnplate_motor_heart_beat(turnplate_motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
// ���̵������
drv_can_t		turnplate_motor_driver = {
	.type = DRV_CAN1,
	.can_id = TURNPLATE_CAN_ID,
//	.std_id = RM2006_GetStdId,
//	.drv_id = RM2006_GetDrvId, 
	.add_tx_msg = CAN_AddTxMessage,
    .start_tx = CAN_StartTx
};

// ���̵����Ϣ
turnplate_motor_info_t turnplate_motor_info = {
	.offline_max_cnt = TPLT_OFFLINE_MAX_CNT,
};

// ���̵��
turnplate_motor_t turnplate_motor = {
	.info = &turnplate_motor_info,
	.driver = &turnplate_motor_driver,
	.init = turnplate_motor_init,
	.update = turnplate_motor_update,
	.check = turnplate_motor_check,
	.heart_beat = turnplate_motor_heart_beat,
	.work_state = DEV_OFFLINE,
	.id = DEV_ID_TURNPLATE,
};

/* Private functions ---------------------------------------------------------*/
static void turnplate_motor_check(turnplate_motor_t *motor)
{
	int16_t err;
	turnplate_motor_info_t *motor_info = motor->info;
	
	/* δ��ʼ�� */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* ����� */
	if(abs(err) > 4095)
	{
		/* 0�� -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8191 + err;
		/* 8191�� -> 0 */
		else
			motor_info->angle_sum += 8191 + err;
	}
	/* δ����� */
	else
	{
		motor_info->angle_sum += err;
	}
	
	motor_info->angle_prev = motor_info->angle;		
}

static void turnplate_motor_heart_beat(turnplate_motor_t *motor)
{
	turnplate_motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/
