#ifndef __RP_SYS_H
#define __RP_SYS_H

#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
/* ʱ��� */
#define		TIME_STAMP_250MS	250
#define 	TIME_STAMP_500MS	500
#define   TIME_STAMP_400MS  400
#define		TIME_STAMP_5000MS	5000


///* Remote Mode Enum */
//typedef enum {
//    RC = 0,
//    KEY = 1,
//    REMOTE_MODE_CNT = 2,
//} remote_mode_t;

//typedef enum {
//    SYS_STATE_NORMAL,	// ϵͳ����
//    SYS_STATE_RCLOST,	// ң��ʧ��
//    SYS_STATE_RCERR,	// ң�س���
//    SYS_STATE_WRONG,	// ����ϵͳ����
//} sys_state_t;

//typedef enum {
//  SYS_MODE_NORMAL,		//��������ģʽ
//	SYS_MODE_AUTO,			//����ģʽ
//	SYS_MODE_LONGSHOOT,	//����ģʽ
//	SYS_MODE_PARK,			//��λģʽ
//	SYS_MODE_CNT,				//����
//} sys_mode_t;					//ϵͳ����ģʽ

//typedef enum {
//	CO_MECH,
//	CO_GYRO
//}co_mode_t;
////ǰ���ֿ���˳���־λ
//typedef enum {
//	level_0 = 0,//
//	level_1 = 1,
//	level_2 = 2,
//	level_3 = 3,
//	level_4 = 4,
//}co_sequent_t;
//typedef struct{
//	co_sequent_t  FBco_sequent;//ǰ���ֳ�ʼ����
//}co_sequent_list;
//typedef enum{
//	None = 0,
//	Set  = 1,
//	ReSet= 2,
//}init_mode_t;

////���ƻ��Ŀ���
//typedef struct{
//	init_mode_t Stand_circle;
//	init_mode_t Locat_circle;
//	init_mode_t Turnaround_circle;
//	init_mode_t FB_wheel;
//	init_mode_t F_wheel;
//	init_mode_t B_wheel;
//}circle_mode_t;

//typedef enum {
//	LOGIC_FRONT,
//	LOGIC_BACK
//	
//}co_angle_logic_t;
////typedef struct {
////	struct {
////		uint8_t reset_start;
////		uint8_t reset_ok;
////	}gimbal;
////} flag_t;
//typedef struct {
//    remote_mode_t		remote_mode;	// ���Ʒ�ʽ
//		co_mode_t 			co_mode;			// ����ģʽ	��̨/��е
//    sys_state_t			state;				// ϵͳ״̬
//    sys_mode_t			mode;					// ϵͳģʽ
//		circle_mode_t   circle_mode;  // ���ƻ�
//	uint8_t           init_stand_flag;//ֻ��һ��
//} system_t;

////extern flag_t	flag;
//extern system_t sys;
//extern co_sequent_list co_sequent;
#endif
