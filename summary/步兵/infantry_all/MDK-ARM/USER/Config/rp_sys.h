#ifndef __RP_SYS_H
#define __RP_SYS_H

#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
/* 时间戳 */
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
//    SYS_STATE_NORMAL,	// 系统正常
//    SYS_STATE_RCLOST,	// 遥控失联
//    SYS_STATE_RCERR,	// 遥控出错
//    SYS_STATE_WRONG,	// 其它系统错误
//} sys_state_t;

//typedef enum {
//  SYS_MODE_NORMAL,		//常规行走模式
//	SYS_MODE_AUTO,			//自瞄模式
//	SYS_MODE_LONGSHOOT,	//吊射模式
//	SYS_MODE_PARK,			//对位模式
//	SYS_MODE_CNT,				//凑数
//} sys_mode_t;					//系统各种模式

//typedef enum {
//	CO_MECH,
//	CO_GYRO
//}co_mode_t;
////前后轮控制顺序标志位
//typedef enum {
//	level_0 = 0,//
//	level_1 = 1,
//	level_2 = 2,
//	level_3 = 3,
//	level_4 = 4,
//}co_sequent_t;
//typedef struct{
//	co_sequent_t  FBco_sequent;//前后轮初始化；
//}co_sequent_list;
//typedef enum{
//	None = 0,
//	Set  = 1,
//	ReSet= 2,
//}init_mode_t;

////控制环的开关
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
//    remote_mode_t		remote_mode;	// 控制方式
//		co_mode_t 			co_mode;			// 控制模式	云台/机械
//    sys_state_t			state;				// 系统状态
//    sys_mode_t			mode;					// 系统模式
//		circle_mode_t   circle_mode;  // 控制环
//	uint8_t           init_stand_flag;//只做一次
//} system_t;

////extern flag_t	flag;
//extern system_t sys;
//extern co_sequent_list co_sequent;
#endif
