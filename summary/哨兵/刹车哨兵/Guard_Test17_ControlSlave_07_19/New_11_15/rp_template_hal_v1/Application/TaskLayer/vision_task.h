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
	
	float distance_filtered;
	float yaw_angle_offset;   //��̨�򵯲�����
	float pitch_angle_offset;
	
//	float pitch_target_predict_speed;   //Ŀ����pitch���µ�Ԥ���ٶ�
//	float yaw_target_predict_speed;     //Ŀ����yaw���µ�Ԥ���ٶ�
	float pitch_angle_err_corrected; //pitch��������ֵ 
	float yaw_angle_err_corrected;  //yaw��������ֵ
	
	float pitch_angle_err;   
	float yaw_angle_err;

//	float pre_pitch_correct_val;  
//	float pre_yaw_correct_val;  
//	
//	float pre_pitch_angle;   //��һ�ν��յ����Ӿ�����
//	float pre_yaw_angle;
//	float pre_pitch_gimb_angle; //��һ�θ����Ӿ�����ʱ��̨�ĵ���Ƕ�
//	float pre_yaw_gimb_angle;
//	float now_pitch_angle;   //���ν��յ����Ӿ�����
//	float now_yaw_angle;
//	float now_pitch_gimb_angle;   //���θ����Ӿ�����ʱ�ĵ���Ƕ�
//	float now_yaw_gimb_angle;
//	
//	float pitch_target_angle_err;   //Ŀ������֡����ʱ�������߹��ĽǶ�ֵ
//	float yaw_target_angle_err;
	
}vision_t;

extern vision_t vision;
extern void QueueInit(void);
extern void QueueUpdate(void);
extern void Vision_Correct(void);

/* Exported functions --------------------------------------------------------*/
#endif
