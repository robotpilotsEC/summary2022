/**
 * @file        rp_math.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.����λ��������
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"
#include "imu_sensor.h"
#include "arm_math.h"
#include "arm_math.h"
#include "vision_task.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
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

float Gimbal_Real_YawRate(void)
{
	float pitch_angle;
	float pitch_cos,pitch_sin;
	float real_yaw_rate;
	pitch_angle = (imu_sensor.info->pitch*PI)/180.0f;
  /*����Pitch���ٶȺϳ�*/
  pitch_cos = arm_cos_f32(pitch_angle);
  pitch_sin = arm_sin_f32(pitch_angle);
	real_yaw_rate = imu_sensor.info->rate_yaw * pitch_cos - imu_sensor.info->rate_roll * pitch_sin;  
  /*..........................................*/
	return real_yaw_rate;
}


/**
 *	@brief	���г�ʼ��
 *  @note   ���ڳ�ʼ�����г���
 */
void Queue_Init(Queue_t *Queue)
{
	Queue->queue_length = LENGTH;
	Queue->queue_data[0]= 0;
	Queue->queue_average = 0;
	Queue->queue_diff = 0;
	Queue->queue_nowlength = 0;
	Queue->queue_sum = 0;
}


/**
 *	@brief	���и���
 *  @param  Queue_t *Queue,������е�data
 *  @return NULL
 *  @note   ��ȡ�Ƚ��ȳ��ķ�ʽ,���¶��еĸ���ֵ
 */
void Queue_Update(Queue_t *Queue,float data)
{
	
	if(Queue->i >= Queue->queue_length)  //��ǰ��������
	{
		Queue->i = Queue->queue_length - 1;
		for(uint16_t j = 0; j < Queue->i; j++)  //������λ���Ƚ��ȳ�
		{
			Queue->queue_data[j] = Queue->queue_data[j+1];
		}
	}
	Queue->queue_data[Queue->i] = data;
	
	Queue->queue_nowlength++;
	if(Queue->queue_nowlength > Queue->queue_length)  //��ֹ�������
	{
		Queue->queue_nowlength = Queue->queue_length;
	}
	//�������
	Queue->queue_sum -= Queue->queue_data[Queue->queue_length - 1];  //��ȥ�����һλ����
	Queue->queue_sum += Queue->queue_data[Queue->i];                      //�ټ������µ�����
	
	Queue->queue_average = Queue->queue_sum / Queue->queue_nowlength; 
	Queue->queue_diff = data - Queue->queue_average;
	Queue->i++;
}

/**
 *	@brief	�������
 */
void Queue_Clear(Queue_t *Queue)
{
	for(uint16_t i = 0; i < Queue->queue_length; i++)  
	{
		Queue->queue_data[i] = 0;
	}
}

/**
 *	@brief	��ȡ�����е�ĳ�����ݣ�����
 *  @note   number��Χ (0~length-1)������ԽС��ʾ����Խ��
 */
float Queue_Get(Queue_t *Queue,uint16_t number)
{
	uint16_t length;
	length = Queue->queue_length;
	if(Queue->queue_nowlength >= length)    //��������
	{
		return Queue->queue_data[length - number - 1];
	}
	else                               //����δ��
	{
		return Queue->queue_data[Queue->queue_nowlength - number - 1];
	}
}


