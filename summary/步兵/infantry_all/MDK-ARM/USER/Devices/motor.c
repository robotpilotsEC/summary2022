/*
	电机添加：
	can通信标识符添加
	电机类型设置：chassis gimb……
	MOTOR添加
	初始化添加
	失联添加
*/


#include "motor.h"
#include "S_function.h"
#include "slave.h"
/*-------------------------------------------------------------*/
/*-----------------------------------------------*/

/*  电机总和  */
extern motor_t Chas_Motor[CHAS_MOTOR_CNT];
extern motor_t Gimb_Motor[GIMB_MOTOR_CNT];
extern motor_t Shot_Motor[SHOT_MOTOR_CNT];

motor_combination_t MOTOR = {
	.chas[POWER_0] = &Chas_Motor[POWER_0],
	.chas[POWER_1] = &Chas_Motor[POWER_1],
	.chas[POWER_2] = &Chas_Motor[POWER_2],	
	.chas[POWER_3] = &Chas_Motor[POWER_3],	

	.chas[TURN_0] = &Chas_Motor[TURN_0],
	.chas[TURN_1] = &Chas_Motor[TURN_1],
	.chas[TURN_2] = &Chas_Motor[TURN_2],	
	.chas[TURN_3] = &Chas_Motor[TURN_3],	
	
	.gimb[GIMB_Y] = &Gimb_Motor[GIMB_Y],
	.gimb[GIMB_P] = &Gimb_Motor[GIMB_P],
	
	.shot[FRIC_L] = &Shot_Motor[FRIC_L],
	.shot[FRIC_R] = &Shot_Motor[FRIC_R],
	.shot[BOX]    = &Shot_Motor[BOX],
	.shot[BARREL] = &Shot_Motor[BARREL],	
	
	.init       = motor_init,
	.update     = motor_update,
	.check      = motor_check,
	.heart_beat = motor_heart_beat,
};


/**====================================**/
void Init_Motor(void)
{	
	MOTOR.init(&Chas_Motor[POWER_0]);
	MOTOR.init(&Chas_Motor[POWER_1]);
	MOTOR.init(&Chas_Motor[POWER_2]);
	MOTOR.init(&Chas_Motor[POWER_3]);

	MOTOR.init(&Chas_Motor[TURN_0]);
	MOTOR.init(&Chas_Motor[TURN_1]);
	MOTOR.init(&Chas_Motor[TURN_2]);
	MOTOR.init(&Chas_Motor[TURN_3]);
	
	MOTOR.init(&Gimb_Motor[GIMB_Y]);
	MOTOR.init(&Gimb_Motor[GIMB_P]);
	
	MOTOR.init(&Shot_Motor[FRIC_L]);
	MOTOR.init(&Shot_Motor[FRIC_R]);
	MOTOR.init(&Shot_Motor[BOX]);
	MOTOR.init(&Shot_Motor[BARREL]);
	
}


char motor_offline_check(void)
{
	char offline_num = 0;
	
	MOTOR.heart_beat(&Chas_Motor[POWER_0]);
	MOTOR.heart_beat(&Chas_Motor[POWER_1]);
	MOTOR.heart_beat(&Chas_Motor[POWER_2]);
	MOTOR.heart_beat(&Chas_Motor[POWER_3]);

	MOTOR.heart_beat(&Chas_Motor[TURN_0]);
	MOTOR.heart_beat(&Chas_Motor[TURN_1]);
	MOTOR.heart_beat(&Chas_Motor[TURN_2]);
	MOTOR.heart_beat(&Chas_Motor[TURN_3]);
	
	MOTOR.heart_beat(&Gimb_Motor[GIMB_Y]);
	MOTOR.heart_beat(&Gimb_Motor[GIMB_P]);
	
	MOTOR.heart_beat(&Shot_Motor[FRIC_L]);
	MOTOR.heart_beat(&Shot_Motor[FRIC_R]);
	MOTOR.heart_beat(&Shot_Motor[BOX]);
	MOTOR.heart_beat(&Shot_Motor[BARREL]);	
  
	for(char i=0 ; i < 8 ; i++ )
	{     
		if(MOTOR.chas[i]->work_state == DEV_OFFLINE)offline_num++;
	}	
	if(CARR_MODE != 1)offline_num -= 4;
	
	for(char i=0 ; i < 2 ; i++ )
	{     
		if(MOTOR.gimb[i]->work_state == DEV_OFFLINE)offline_num++;
	}	
	for(char i=0 ; i < 4 ; i++ )
	{     
		if(MOTOR.shot[i]->work_state == DEV_OFFLINE)offline_num++;
	}
	if(CARR_MODE != 3)offline_num -= 1;
	
	MOTOR.offline_num = offline_num;//失联电机个数 需要减去无效电机
	return offline_num;
}



/**====================================**/



/*************** 信息协议处理 ***************/
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

/**
 *	@brief	从CAN报文中读取电机的实际输出转矩
 */
static int16_t CAN_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((int16_t)rxData[4] << 8 | rxData[5]);
	return torque;
}

/**
 *	@brief	从CAN报文中读取电机的实际温度
 */
static uint8_t CAN_GetMotorTemperature(uint8_t *rxData)
{
	uint8_t temperature;
	temperature = rxData[6];
	return temperature;
}
/*************** 信息协议处理 ***************/
/*************** 信息对象处理 ***************/

/** RM3508 CAN标识符 **/
static uint32_t RM3508_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x201U) < 4)
		return 0x200;
	else
		return 0x1FF;
}

/** RM3508 CAN数据下标 **/
static uint8_t RM3508_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x201U)%4;
}

/** GM6020 CAN标识符 **/
static uint32_t GM6020_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x205U) < 4)
		return 0x1FF;
	else
		return 0x2FF;
}

/** GM6020 CAN数据下标 **/
static uint8_t GM6020_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x205U)%4;
}

/** RM2006 CAN标识符 **/
static uint32_t RM2006_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x201U) < 4)
		return 0x200;
	else
		return 0x1FF;
}

/** RM2006 CAN数据下标 **/

static uint8_t RM2006_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x201U)%4;
}

/*************** 信息对象处理 ***************/




/*结构体函数begin*/
void motor_check(motor_t *motor)
{
	int16_t err;
	motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8191 + err;
		/* 8191↑ -> 0 */
		else
			motor_info->angle_sum += 8191 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->angle_sum += err;
	}
	
	motor_info->angle_prev = motor_info->angle;		
}

void motor_heart_beat(motor_t *motor)
{
	motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) 
	{
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else 
	{
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

void motor_update(motor_t *motor, uint8_t *rxBuf)
{
	motor_info_t *motor_info = motor->info;
	
#if !SET_MID_MODE
	
	if(motor->type == CHASSIS_TURN)
		MID_RESET(motor, motor->info->MID , CAN_GetMotorAngle(rxBuf));	
	else 
		
#endif
	
	motor_info->angle   = CAN_GetMotorAngle(rxBuf);	
	
	motor_info->speed   = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->temperature = CAN_GetMotorTemperature(rxBuf);
	
	motor_info->Torque = CAN_GetMotorTorque(rxBuf);
	
	motor_info->offline_cnt = 0;
}


void motor_init(motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	
	if(motor->type == CHASSIS_POWER || motor->type == SHOOT_FRIC) 
	{
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->type == SHOOT_BOX || motor->type == SHOOT_BARREL) 
	{
		drv_can->drv_id = RM2006_GetDrvId(drv_can);
		drv_can->std_id = RM2006_GetStdId(drv_can);
	}
	else if(motor->type == GIMBAL || motor->type == CHASSIS_TURN) 
	{
		drv_can->drv_id = GM6020_GetDrvId(drv_can);
		drv_can->std_id = GM6020_GetStdId(drv_can);	
	}
	
	else{
		motor->errno = DEV_TYPE_ERR;
	}
}

/*结构体函数end*/



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t Send_Current(uint8_t CAN, uint32_t stdId, int16_t *dat)
{

	uint8_t ret,num = 0;
	
	for(char i = 0;i < 4; i++)
	{
		if(dat[i] == NULL)dat[i] = 0;
		if(dat[i] != 0)num++;
	}
	
	if(CAN == 1 && num)
	{
		ret = CAN_SendData(&hcan1, stdId, dat);
	}
	else if(CAN == 2 && num)
	{
		ret = CAN_SendData(&hcan2, stdId, dat);
	}
	
	for(char i = 0;i < 4; i++)
	dat[i] = 0;
	
	return ret;
}


void MOTOR_CAN1_RX(uint32_t canId, uint8_t *rxBuf)
{

	switch(canId)
	{
		case CAN_ID_POWER_0:
			MOTOR.update(&Chas_Motor[POWER_0], rxBuf);
			MOTOR.check(&Chas_Motor[POWER_0]);
			break;
		case CAN_ID_POWER_1:
			MOTOR.update(&Chas_Motor[POWER_1], rxBuf);
			MOTOR.check(&Chas_Motor[POWER_1]);
			break;		
		case CAN_ID_POWER_2:
			MOTOR.update(&Chas_Motor[POWER_2], rxBuf);
			MOTOR.check(&Chas_Motor[POWER_2]);
			break;	
		case CAN_ID_POWER_3:
			MOTOR.update(&Chas_Motor[POWER_3], rxBuf);
			MOTOR.check(&Chas_Motor[POWER_3]);
			break;		
		case CAN_ID_GIMB_Y:
			MOTOR.update(&Gimb_Motor[GIMB_Y], rxBuf);
			MOTOR.check(&Gimb_Motor[GIMB_Y]);
			break;	
		case CAN_ID_GIMB_P:
			MOTOR.update(&Gimb_Motor[GIMB_P], rxBuf);
			MOTOR.check(&Gimb_Motor[GIMB_P]);
			break;		
	}
}

void MOTOR_CAN2_RX(uint32_t canId, uint8_t *rxBuf)
{

		switch(canId)
		{
			case CAN_ID_TURN_0:
				MOTOR.update(&Chas_Motor[TURN_0], rxBuf);
				MOTOR.check(&Chas_Motor[TURN_0]);
				break;
			case CAN_ID_TURN_1:
				MOTOR.update(&Chas_Motor[TURN_1], rxBuf);
				MOTOR.check(&Chas_Motor[TURN_1]);
				break;		
			case CAN_ID_TURN_2:
				MOTOR.update(&Chas_Motor[TURN_2], rxBuf);
				MOTOR.check(&Chas_Motor[TURN_2]);
				break;		
			case CAN_ID_TURN_3:
				MOTOR.update(&Chas_Motor[TURN_3], rxBuf);
				MOTOR.check(&Chas_Motor[TURN_3]);
				break;	

			case CAN_ID_FRIC_L:
				MOTOR.update(&Shot_Motor[FRIC_L], rxBuf);
				MOTOR.check(&Shot_Motor[FRIC_L]);
				break;
			case CAN_ID_FRIC_R:
				MOTOR.update(&Shot_Motor[FRIC_R], rxBuf);
				MOTOR.check(&Shot_Motor[FRIC_R]);
				break;		
			case CAN_ID_BOX:
				MOTOR.update(&Shot_Motor[BOX], rxBuf);
				MOTOR.check(&Shot_Motor[BOX]);
				break;		
			case CAN_ID_BARREL:
				MOTOR.update(&Shot_Motor[BARREL], rxBuf);
				MOTOR.check(&Shot_Motor[BARREL]);
				break;		
		}
}
