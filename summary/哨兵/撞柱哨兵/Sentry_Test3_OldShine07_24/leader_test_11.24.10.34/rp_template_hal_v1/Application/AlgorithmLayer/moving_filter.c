#include "moving_filter.h"

extmoving_filter GYRO_SpeedError_SF;
extmoving_filter Mouse_YSpeed_SF;
extmoving_filter vision_pitch_givenSF,vision_yaw_givenSF;



/*** 滑动滤波算法 ***/
void Moving_Filter_Init(extmoving_filter *extern_data)
{
	extern_data->i = 0;
	extern_data->SF[0]= 0;
	extern_data->sum = 0;
}


float Moving_Filter_Create(extmoving_filter *extern_data,float get_da)
{
	extern_data->sum = 0;            //清零
	extern_data->SF[extern_data->i++] = get_da;
	
	if(extern_data->i == SF_LENGTH) extern_data->i = 0;
		
	
	for(uint8_t count = 0;count < SF_LENGTH;count++)
		extern_data->sum += extern_data->SF[count];
	
	return (extern_data->sum / SF_LENGTH);
}

void MovingFilter_Init(void)
{
	//视觉给定数据
	Moving_Filter_Init(&vision_pitch_givenSF);
	Moving_Filter_Init(&vision_yaw_givenSF);
	//陀螺仪速度环差值
	Moving_Filter_Init(&GYRO_SpeedError_SF);
	
}
