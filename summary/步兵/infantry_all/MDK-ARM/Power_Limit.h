#ifndef __POWER_LIMIT_H
#define __POWER_LIMIT_H


#include "rp_config.h"
#include "POTOCAL.h"
#include "S_function.h"

void Chassis_Motor_Power_Limit(int16_t *data);
	
void Chassis_2022_CAP_Power_Limit(int16_t *data);
	
void Chassis_Turn_Power_Limit(int16_t *data);
	
void Judge_Offline_Power_Limit(int16_t *data);
		
#endif

