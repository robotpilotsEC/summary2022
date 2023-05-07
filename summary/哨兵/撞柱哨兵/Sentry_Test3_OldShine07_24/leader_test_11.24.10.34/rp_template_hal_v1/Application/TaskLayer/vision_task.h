#ifndef __VISION_TASK_H
#define __VISION_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "vision_sensor.h"
/* Exported macro ------------------------------------------------------------*/
#define ACTIVE_MAX_CNT  1
#define LOST_MAX_CNT    1	/*����ʶ��Ͷ�ʧ�ж�����ֵ*/

#define GIMBAL_RECALL_TIME 10     //1ms����һ�ζ���
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	Vision_Cmd_Id_t *Cmd;        //������
	Vision_Rx_Data_t *RxData;    //�����Ӿ�������
	Vision_State_t *State;      //�Ӿ�״̬
}Vision_RxData_t;

typedef struct
{
	Vision_RxData_t *rx_data;
	bool identify_ok; 
	bool target_update; 	
	float pitch_angle_raw;
	float yaw_angle_raw;

	float pitch_angle_err_corrected; //pitch��������ֵ 
	float yaw_angle_err_corrected;  //yaw��������ֵ
	
	float pitch_angle_err;   
	float yaw_angle_err;
	
	float gimbal_pitch_measure;
	float gimbal_yaw_measure;
	float distance_filtered;
	float yaw_angle_offset;   //��̨�򵯲�����
	float pitch_angle_offset;
		
}vision_t;

extern vision_t vision;
extern void QueueInit(void);
extern void QueueUpdate(void);
extern void Vision_Correct(void);

/* Exported functions --------------------------------------------------------*/
#endif
