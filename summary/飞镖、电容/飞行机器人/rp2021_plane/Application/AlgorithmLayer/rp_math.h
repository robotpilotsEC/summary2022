#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "stm32f4xx_hal.h"

#include "pid.h"

typedef struct {
    int16_t buff[20];
    int16_t buff_len;
    int16_t cur_index;
    int16_t avr_int;
    float   avr_float;
} int16_avr_filter_t;

typedef struct {
    float buff[20];
    int16_t buff_len;
    int16_t cur_index;
    float   avr_float;
} float_avr_filter_t;

/* 数值函数 */
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define abs(x) 					((x)>0? (x):(-(x)))

int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
float RampFloat(float final, float now, float ramp);
float DeathZoom(float input, float center, float death);
bool IfInDeathZoomInt(int16_t input, int16_t min, int16_t max);
float MovingAverageInt(int16_avr_filter_t *filter, int16_t new_dat);
float MovingAverageFloat(float_avr_filter_t *filter, float new_dat);
float MovingAverageFloat_Clear(float_avr_filter_t *filter);

#endif

