#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "rp_math.h"
#include "ALGO.h"
#include "Chassis_motor.h"
#include "Gimbal_motor.h"
#include "Shoot_motor.h"



typedef struct motor_combination_struct{
	motor_t *chas[CHAS_MOTOR_CNT];
	motor_t *gimb[GIMB_MOTOR_CNT];
	motor_t *shot[SHOT_MOTOR_CNT];
	
	void		(*init)(struct motor_struct *self);
	void		(*update)(struct motor_struct *self, uint8_t *rxBuf);
	void		(*check)(struct motor_struct *self);	
	void		(*heart_beat)(struct motor_struct *self);
	
	char    offline_num;
} motor_combination_t;

extern motor_combination_t	MOTOR;

void motor_init(motor_t *motor);
void motor_check(motor_t *motor);
void motor_heart_beat(motor_t *motor);
void motor_update(motor_t *motor, uint8_t *rxBuf);


void Init_Motor(void);
char motor_offline_check(void);

uint8_t Send_Current(uint8_t CAN, uint32_t stdId, int16_t *dat);
void MOTOR_CAN1_RX(uint32_t canId, uint8_t *rxBuf);
void MOTOR_CAN2_RX(uint32_t canId, uint8_t *rxBuf);
#endif


