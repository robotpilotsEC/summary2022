#ifndef __VISION_TASK_H
#define __VISION_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "vision_sensor.h"
/* Exported macro ------------------------------------------------------------*/
#define ACTIVE_MAX_CNT  1
#define LOST_MAX_CNT    1	/*对于识别和丢失判定的阈值*/

#define GIMBAL_RECALL_TIME 10     //1ms更新一次队列  
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	Vision_Cmd_Id_t *Cmd;        //命令码
	Vision_Rx_Data_t *RxData;    //接收视觉的数据
	Vision_State_t *State;      //视觉状态
}Vision_RxData_t;

typedef struct
{
	Vision_RxData_t *rx_data;
	bool identify_ok; 
	bool target_update; 	
	float pitch_angle_raw;
	float yaw_angle_raw;
	
	float distance_filtered;
	float yaw_angle_offset;   //云台打弹补偿角
	float pitch_angle_offset;
	
//	float pitch_target_predict_speed;   //目标在pitch轴下的预测速度
//	float yaw_target_predict_speed;     //目标在yaw轴下的预测速度
	float pitch_angle_err_corrected; //pitch轴误差矫正值 
	float yaw_angle_err_corrected;  //yaw轴误差矫正值
	
	float pitch_angle_err;   
	float yaw_angle_err;

//	float pre_pitch_correct_val;  
//	float pre_yaw_correct_val;  
//	
//	float pre_pitch_angle;   //上一次接收到的视觉数据
//	float pre_yaw_angle;
//	float pre_pitch_gimb_angle; //上一次更新视觉数据时云台的电机角度
//	float pre_yaw_gimb_angle;
//	float now_pitch_angle;   //本次接收到的视觉数据
//	float now_yaw_angle;
//	float now_pitch_gimb_angle;   //本次更新视觉数据时的电机角度
//	float now_yaw_gimb_angle;
//	
//	float pitch_target_angle_err;   //目标在两帧数据时间间隔里走过的角度值
//	float yaw_target_angle_err;
	
}vision_t;

extern vision_t vision;
extern void QueueInit(void);
extern void QueueUpdate(void);
extern void Vision_Correct(void);

/* Exported functions --------------------------------------------------------*/
#endif
