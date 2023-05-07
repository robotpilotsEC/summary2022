#ifndef __SHOOT_H
#define __SHOOT_H

#include "POTOCAL.h"
#include "S_function.h"
#include "tim_drv.h"
#include "io_drv.h"


extern int16_t FRI_SPEED_TAR;
#define NOW_SPEED_TARGET FRI_SPEED_TAR


typedef  struct
{
	int16_t cnt_11;
	int16_t cnt_12;
	int16_t cnt_13;
	int16_t cnt_14;
	int16_t cnt_15;

	int16_t cnt_16;
	int16_t cnt_17;
	int16_t cnt_18;
	int16_t cnt_19;
	int16_t cnt_20;
	
	int16_t cnt_21;
	int16_t cnt_22;
	int16_t cnt_23;
	int16_t cnt_24;
	int16_t cnt_25;	
	
	int16_t cnt_26;
	int16_t cnt_27;
	int16_t cnt_28;
	int16_t cnt_29;
	int16_t cnt_30;
}shoot_sp_t;



typedef  struct shoot_recode_str
{
	float pj,sp_cnt,max,min,jicha;
	float speed, pre_speed;
	
	int16_t cnt;
	shoot_sp_t sp;
	
	
}shoot_recode_t;










void SHOOT_CTRL(void);
void FRIC_INIT(void);
void SHOT_INIT(void);

bool Fric_Err_OK(void);
void Shot_Stop(void);
#endif
