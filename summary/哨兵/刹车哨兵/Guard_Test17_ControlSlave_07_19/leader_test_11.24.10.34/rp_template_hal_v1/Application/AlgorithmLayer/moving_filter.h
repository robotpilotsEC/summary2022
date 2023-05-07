#ifndef __MOVING_FILTER_H
#define __MOVING_FILTER_H
#include "stdint.h"

#define SF_LENGTH 70

typedef struct
{
	float SF[SF_LENGTH]; //Ä¬ÈÏ³¤¶È70
	uint8_t i;
	float sum;
}extmoving_filter;


float Moving_Filter_Create(extmoving_filter *extern_data,float get_da);
void MovingFilter_Init(void);


extern extmoving_filter GYRO_SpeedError_SF;
extern extmoving_filter vision_pitch_givenSF,vision_yaw_givenSF;



#endif

