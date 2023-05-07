#include "motor.h"
#include "can.h"
#include "rc.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void motor_check(motor_t *motor);
static void motor_heart_beat(motor_t *motor);
void motor_update(motor_t *motor, uint8_t *rxBuf);

#define abs(x) 					((x)>0? (x):(-(x)))

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 电机驱动
drv_can_t		motor_driver[] = {
	[BELT_MID] = {
		.type = DRV_CAN1,
		.can_id = BELT_SIDE_CAN_ID,
		.std_id = 0x200,
		.drv_id = 1,
		.tx_data = CAN_SendSingleData,
	},
	[BELT_SIDE] = {
		.type = DRV_CAN1,
		.can_id = PUSH_TURN_CAN_ID,
		.std_id = 0x200,
		.drv_id = 2,
		.tx_data = CAN_SendSingleData,
	},
	[PUSH_TURN] = {
		.type = DRV_CAN1,
		.can_id = DART_WHEEL_CAN_ID,
		.std_id = 0x200,
		.drv_id = 3,
		.tx_data = CAN_SendSingleData,
	},
	[DART_WHEEL] = {
		.type = DRV_CAN2,
		.can_id = GIMBAL_CAN_ID,
		.std_id = 0x200,
		.drv_id = 4,
		.tx_data = CAN_SendSingleData,
	},
    [PITCH_M] = {
        .type = DRV_CAN2,
		.can_id = PITCH_M_CAN_ID,
		.std_id = 0x1FF,
		.drv_id = 5,
		.tx_data = CAN_SendSingleData,
	},
    [YAW_M] = {
        .type = DRV_CAN2,
		.can_id = YAW_M_CAN_ID,
		.std_id = 0x1FF,
		.drv_id = 6,
		.tx_data = CAN_SendSingleData,
	},
    [GIMBAL] = {
        .type = DRV_CAN2,
		.can_id = BELT_MID_CAN_ID,
		.std_id = 0x1FF,
		.drv_id = 7,
		.tx_data = CAN_SendSingleData,
	},
    
};

//电机复位
motor_reset_t motor_res[] = {
    [BELT_MID] = {
		.speed = 1000,
        .direction = 1,
        .temp_outmax = 2400,
        .position = -8000,  //position与direction要反号，下同
	},
    [BELT_SIDE] = {
		.speed = 500,
        .direction = 1,
        .temp_outmax = 4000,
        .position = -60000//-12500,
	},
    [PUSH_TURN] = {
		.speed = 500,
        .direction = 1,
        .temp_outmax = 2000,
        .position = -60000,
	},
    [DART_WHEEL] = {
		.speed = 200,
        .direction = 1,
        .temp_outmax = 5000,
        .position = -32550,
	},
    [PITCH_M] = {
		.speed = 700,
        .direction = -1,
        .temp_outmax = 3000,
        .position = 520273//43154,
	},
    [YAW_M] = {
		.speed = 2000,
        .direction = 1,
        .temp_outmax = 3000,
        .position = -247956//-408020//-2388150//-1540815 //-1989135,
	},
    [GIMBAL] = {
		.speed = 100,
        .direction = -1,
        .temp_outmax = 2000,
        .position = 26697//21220,
	},
};

// 电机信息
motor_info_t 	motor_info[] = {
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
};

//电机传感器
motor_t		motor[] = {
	[BELT_MID] = {
		.info = &motor_info[BELT_MID],
		.driver = &motor_driver[BELT_MID],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[BELT_MID],
        .pid_s = &pid_speed[BELT_MID],
        .pid_a = &pid_angle[BELT_MID],
		.work_state = DEV_OFFLINE,
	},
	[BELT_SIDE] = {
		.info = &motor_info[BELT_SIDE],
		.driver = &motor_driver[BELT_SIDE],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[BELT_SIDE],
        .pid_s = &pid_speed[BELT_SIDE],
        .pid_a = &pid_angle[BELT_SIDE],
		.work_state = DEV_OFFLINE,
	},
	[PUSH_TURN] = {
		.info = &motor_info[PUSH_TURN],
		.driver = &motor_driver[PUSH_TURN],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[PUSH_TURN],
        .pid_s = &pid_speed[PUSH_TURN],
        .pid_a = &pid_angle[PUSH_TURN],
		.work_state = DEV_OFFLINE,
	},
	[DART_WHEEL] = {
		.info = &motor_info[DART_WHEEL],
		.driver = &motor_driver[DART_WHEEL],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[DART_WHEEL],
        .pid_s = &pid_speed[DART_WHEEL],
        .pid_a = &pid_angle[DART_WHEEL],
		.work_state = DEV_OFFLINE,
	},
	[PITCH_M] = {
		.info = &motor_info[PITCH_M],
		.driver = &motor_driver[PITCH_M],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[PITCH_M],
        .pid_s = &pid_speed[PITCH_M],
        .pid_a = &pid_angle[PITCH_M],
		.work_state = DEV_OFFLINE,
	},
	[YAW_M] = {
		.info = &motor_info[YAW_M],
		.driver = &motor_driver[YAW_M],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[YAW_M],
        .pid_s = &pid_speed[YAW_M],
        .pid_a = &pid_angle[YAW_M],
		.work_state = DEV_OFFLINE,
	},
	[GIMBAL] = {
		.info = &motor_info[GIMBAL],
		.driver = &motor_driver[GIMBAL],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
        .reset = motor_reset,
        .res = &motor_res[GIMBAL],
        .pid_s = &pid_speed[GIMBAL],
        .pid_a = &pid_angle[GIMBAL],
		.work_state = DEV_OFFLINE,
	},
};

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
static int16_t CAN_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = ((int16_t)rxData[4] << 8 | rxData[5]);
	return current;
}

static void motor_check(motor_t *motor)
{
	int16_t err;
	motor_info_t *motor_info_temp = motor->info;
	
	/* 未初始化 */
	if( !motor_info_temp->init_flag )
	{
		motor_info_temp->init_flag = 1;
		motor_info_temp->angle_prev = motor_info_temp->angle;
	}
	
	err = motor_info_temp->angle - motor_info_temp->angle_prev;
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info_temp->angle_sum += -8191 + err;
		/* 8191↑ -> 0 */
		else
			motor_info_temp->angle_sum += 8191 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info_temp->angle_sum += err;
	}
	
	motor_info_temp->angle_prev = motor_info_temp->angle;		
}

static void motor_heart_beat(motor_t *motor)
{
	motor_info_t *motor_info_temp = motor->info;
	
	motor_info_temp->offline_cnt++;
	if(motor_info_temp->offline_cnt > motor_info_temp->offline_max_cnt) {
		motor_info_temp->offline_cnt = motor_info_temp->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

void motor_update(motor_t *motor, uint8_t *rxBuf)
{
	motor_info_t *motor_info_temp = motor->info;
	
	motor_info_temp->angle = CAN_GetMotorAngle(rxBuf);
	motor_info_temp->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info_temp->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info_temp->offline_cnt = 0;
}

void motor_init(motor_t *motor)
{
//	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	
}

void motor_reset(motor_t *motor)
{
    pid_t *pid_s = motor->pid_s;
    pid_t *pid_a = motor->pid_a;
    motor_reset_t *res = motor->res;
    if(res->state == 0)return;
    else if(res->state == 1)
    {
        pid_a->sw = 0;
        pid_s->sw = 1;
        pid_s->set = res->speed * res->direction;
        res->ori_outmax = pid_s->outmax;
        pid_s->outmax = res->temp_outmax;
        res->time = 0;
        res->state = 2;
    }
    else if(res->state == 2)
    {
        res->time = abs(pid_s->actual) < 20 ? ++res->time : 0;
        if(res->time > 30)
        {
            res->state = 3;
            res->time = 0;
            motor->info->angle_sum = 0;
            pid_s->outmax = res->ori_outmax;
            pid_a->sw = 1;
            pid_s->sw = 1;
            pid_a->set = motor->res->position;
        }
    }
    else if(res->state == 3)
    {
        res->time = abs(pid_a->err) < 200 ? ++res->time : 0;
        if(res->time > 100)
        {
            res->state = 4;
            res->time = 0;
        }
    }
}

int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
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

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(canId == BELT_MID_CAN_ID)
	{
		motor[BELT_MID].update(&motor[BELT_MID], rxBuf);
		motor[BELT_MID].check(&motor[BELT_MID]);
        if(motor[BELT_MID].info->speed == -3)motor[BELT_MID].info->speed = 0;
	}
    else if(canId == BELT_SIDE_CAN_ID)
	{
		motor[BELT_SIDE].update(&motor[BELT_SIDE], rxBuf);
		motor[BELT_SIDE].check(&motor[BELT_SIDE]);
	}
    else if(canId == PUSH_TURN_CAN_ID)
	{
		motor[PUSH_TURN].update(&motor[PUSH_TURN], rxBuf);
		motor[PUSH_TURN].check(&motor[PUSH_TURN]);
        if(motor[PUSH_TURN].info->speed == -4)motor[PUSH_TURN].info->speed = 0;
	}
    else if(canId == DART_WHEEL_CAN_ID)
	{
		motor[DART_WHEEL].update(&motor[DART_WHEEL], rxBuf);
		motor[DART_WHEEL].check(&motor[DART_WHEEL]);
        if(motor[DART_WHEEL].info->speed == -3)motor[DART_WHEEL].info->speed = 0;
	}
}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == PITCH_M_CAN_ID)
	{
		motor[PITCH_M].update(&motor[PITCH_M], rxBuf);
		motor[PITCH_M].check(&motor[PITCH_M]);
        if(motor[PITCH_M].info->speed == -3)motor[PITCH_M].info->speed = 0;
	}
	else if(canId == YAW_M_CAN_ID)
	{
		motor[YAW_M].update(&motor[YAW_M], rxBuf);
		motor[YAW_M].check(&motor[YAW_M]);
	}
    else if(canId == GIMBAL_CAN_ID)
	{
		motor[GIMBAL].update(&motor[GIMBAL], rxBuf);
		motor[GIMBAL].check(&motor[GIMBAL]);
	}
}






