#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "POTOCAL.h"
#include "S_function.h"



void GIMBAL_CTRL(void);
void SET_GIMB_PID(void);
	
void GIMBAL_PIT_INIT(void);
void GIMBAL_YAW_INIT(void);
void GIMBAL_INIT(void);
void GIMBAL_INIT_NOW(void);
void GIMBAL_INIT_CTRL(void);
void GIMBAL_AUTO_Ctrl(void);

void VISION_TEST(void);
void GIMBAL_PIT_NOW_SET(void);
float Yaw_Turn_Err(void);
bool ARRIVE(void);
bool Find_Tar(void);
#endif

