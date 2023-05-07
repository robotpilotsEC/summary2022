/**
 * @file        rp_math.h
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.增加位操作函数
 */

#ifndef __RP_MATH_H
#define __RP_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* Exported macro ------------------------------------------------------------*/
/* 位操作函数 */
#define SET_EVENT(EVENT, FLAG)      ((EVENT) |= FLAG)     
#define CLEAR_EVENT(EVENT, FLAG)    ((EVENT) &= ~(FLAG))
#define GET_EVENT(EVENT, FLAG)      ((EVENT) & (FLAG))
/* 数值函数 */
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define abs(x) 					((x)>0? (x):(-(x)))

#define QUEUE_LENGTH 100  //定义队列长度
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	LENGTH = QUEUE_LENGTH,   //队列长度100
}QueueLength_t;

typedef struct
{
	uint16_t	queue_length;  //队列长度
	uint16_t  queue_nowlength;  //队列当前长度
	float queue_data[QUEUE_LENGTH];
	float queue_diff;      //队列差分
	float queue_sum;       //队列和
	float queue_average;   //队列均值
  uint16_t i;
}Queue_t;

/* Exported functions --------------------------------------------------------*/
/* 斜坡函数 */
int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
float RampFloat(float final, float now, float ramp);
/* 死区函数 */
float DeathZoom(float input, float center, float death);
/* 真实Yaw轴速度计算函数 */
float Gimbal_Real_YawRate(void);

void Queue_Init(Queue_t *queue);
void Queue_Update(Queue_t *Queue,float data);
void Queue_Clear(Queue_t *Queue);
float Queue_Get(Queue_t *queue,uint16_t number);

#endif

