#include "rp_math.h"

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

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;
	
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

float DeathZoom(float input, float center, float death)
{
	if(abs(input - center) < death)
		return center;
	return input;
}

bool IfInDeathZoomInt(int16_t input, int16_t min, int16_t max)
{
    if(min > max)
        return false;
    
    if(input < min || input > max)
        return false;
    return true;
}

float MovingAverageInt(int16_avr_filter_t *filter, int16_t new_dat)
{
    int32_t avr_sum = 0;
        
    /* 数据未满 */
    if(filter->cur_index < filter->buff_len) {
        filter->buff[filter->cur_index++] = new_dat;
        for(int16_t i=0; i<filter->cur_index; i++)
            avr_sum += filter->buff[i];
    } 
    /* 数据满了 */
    else {
        for(int16_t i=0; i<filter->buff_len-1; i++)
        {
            filter->buff[i] = filter->buff[i+1];
            avr_sum += filter->buff[i];
        }        
        filter->buff[filter->buff_len-1] = new_dat;
        avr_sum += new_dat;
    }
    
    filter->avr_float = (float)avr_sum/filter->cur_index;
    filter->avr_int = avr_sum/filter->cur_index;

    return filter->avr_float;
}

float MovingAverageFloat(float_avr_filter_t *filter, float new_dat)
{
    float avr_sum = 0;
        
    /* 数据未满 */
    if(filter->cur_index < filter->buff_len) {
        filter->buff[filter->cur_index++] = new_dat;
        for(int16_t i=0; i<filter->cur_index; i++)
            avr_sum += filter->buff[i];
    } 
    /* 数据满了 */
    else {
        for(int16_t i=0; i<filter->buff_len-1; i++)
        {
            filter->buff[i] = filter->buff[i+1];
            avr_sum += filter->buff[i];
        }        
        filter->buff[filter->buff_len-1] = new_dat;
        avr_sum += new_dat;
    }
    
    filter->avr_float = (float)avr_sum/filter->cur_index;

    return filter->avr_float;
}

float MovingAverageFloat_Clear(float_avr_filter_t *filter)
{
    for(int16_t i=0; i<filter->buff_len-1; i++)
    {
        filter->buff[i] = 0.f;
    }      
    filter->cur_index = 0;
    filter->avr_float = 0.f;
    
    return filter->avr_float;
}
