#ifndef __OFFLINE_CHECK_H
#define __OFFLINE_CHECK_H

#include "POTOCAL.h"
#include "Chassis.h"
#include "Shoot.h"

#include "judge_sensor.h"
#include "vision_potocol.h"
#include "vision_sensor.h"




typedef struct{
	uint32_t time;
}sleep_t;


void OFFLINE_CHECK_TASK(void);




#endif



