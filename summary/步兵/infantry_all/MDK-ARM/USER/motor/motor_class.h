#ifndef __MOTOR_CLASS_H
#define __MOTOR_CLASS_H

#include "rp_config.h"
#include "ALGO.h"

//发送邮箱ID号  std_id
//RM2006 RM3508
#define RM_F_ID 0x200
#define RM_B_ID 0x1FF
//GM6020
#define GM_F_ID 0x1FF
#define GM_B_ID 0x2FF

#define POWER_ID RM_F_ID
#define TURNN_ID GM_F_ID
#define GIMBA_ID GM_F_ID
#define SHOOT_ID RM_F_ID

#define POWER_CAN 1 //4
#define TURNN_CAN 2 //4
#define GIMBA_CAN 1 //2
#define SHOOT_CAN 2 //3


//CAN1
#define CAN_ID_POWER_0	0x201U//202
#define CAN_ID_POWER_1	0x202U//201
#define CAN_ID_POWER_2	0x203U//前轮
#define CAN_ID_POWER_3	0x204U//后轮

#define CAN_ID_GIMB_Y	0x205U//
#define CAN_ID_GIMB_P	0x206U//

//CAN2
#define CAN_ID_BOX	  0x203U//
#define CAN_ID_BARREL	0x204U//

//CAN2
#define CAN_ID_TURN_0	0x205U//
#define CAN_ID_TURN_1	0x206U//
#define CAN_ID_TURN_2	0x207U//
#define CAN_ID_TURN_3	0x208U//

#define MEC_MID_TURN_0	(6168)//4738//( 642 + 4095)
#define MEC_MID_TURN_1	(3407 -30)
#define MEC_MID_TURN_2	(3470)//791//(4887 - 4095)
#define MEC_MID_TURN_3	(4821 +30)

#define TURN_OUT_PID_P        -2.f
#define TURN_OUT_PID_I        0.f
#define TURN_OUT_PID_D        0.f
#define TURN_OUT_PID_I_MAX    5000.f
#define TURN_OUT_PID_O_MAX    25000.f
#define TURN_OUT_PID_B_ERR    0.f

/*
 *逆时针数值增大 类似坐标系
 */

#define TURN_IN_PID_P        20.f
#define TURN_IN_PID_I        0.f
#define TURN_IN_PID_D        0.f
#define TURN_IN_PID_I_MAX    5000.f
#define TURN_IN_PID_O_MAX    20000.f
#define TURN_IN_PID_B_ERR    0.f


/*---------------------------------------------------------------------------------------*/















/*-old_mec-*/
#if CARR_MODE == 0

/*电机驱动*/
//CAN2
#define CAN_ID_FRIC_L	0x201U//
#define CAN_ID_FRIC_R	0x202U//

/*电机中值*/

#define MEC_MID_Y_F	(6304)//
#define MEC_MID_Y_B	(6304 - 4095)//

#define MEC_MID_P	(1331)// 5270 4755 4216

/* PID */

#define POWER_PID_P       8.f
#define POWER_PID_I       0.6f
#define POWER_PID_D       0.f
#define POWER_PID_I_MAX   4000.f
#define POWER_PID_O_MAX   8000.f
#define POWER_PID_B_ERR   0.f

/*转向pid设置*/
#define YAW_TURN_KP 0.22f
#define YAW_IMU_KP  1.6f 
#define PIT_IMU_KP  13.f 

#define IMU_HIG_P	(3500)//抬头最高角度 按下G键    hig3700-4700down 496 404
#define IMU_MID_P	(4095)
#define IMU_LOW_P	(4520)

#endif




#if CARR_MODE == 1

//CAN2
#define CAN_ID_FRIC_L	0x202U//
#define CAN_ID_FRIC_R	0x201U//



/*电机中值*/

#define MEC_MID_Y_F	(2686 + 4095)//1004
#define MEC_MID_Y_B	(2686)//5100

#define MEC_MID_P	(6830)// 5270 4755 4216

/* PID */
#define POWER_PID_P       8.f//15耗电很多
#define POWER_PID_I       0.6f
#define POWER_PID_D       0.f
#define POWER_PID_I_MAX   5000.f
#define POWER_PID_O_MAX   9000.f
#define POWER_PID_B_ERR   0.f

/*Yaw转向pid设置*/
#define YAW_TURN_KP 0.2f
#define YAW_IMU_KP  1.6f 
#define PIT_IMU_KP  13.f 

#define IMU_HIG_P	(3450)//抬头最高角度 按下G键    hig3700-4700down 496 404
#define IMU_MID_P	(4095)
#define IMU_LOW_P	(4430)
#endif



/*omni*/
#if CARR_MODE == 2
//CAN2
#define CAN_ID_FRIC_L	0x201U//
#define CAN_ID_FRIC_R	0x202U//

/*电机中值*/

#define MEC_MID_Y_F	(1720)//
#define MEC_MID_Y_B	(1720 + 4095)//

#define MEC_MID_P	(7414)// 5270 4755 4216


/* PID */

#define POWER_PID_P       8.f
#define POWER_PID_I       0.6f
#define POWER_PID_D       0.f
#define POWER_PID_I_MAX   5000.f
#define POWER_PID_O_MAX   8000.f
#define POWER_PID_B_ERR   0.f

/*Yaw转向pid设置*/
#define YAW_TURN_KP 0.22f
#define YAW_IMU_KP  1.6f 
#define PIT_IMU_KP  20.f

#define IMU_HIG_P	(3500)//抬头最高角度 按下G键    hig3700-4700down 496 404
#define IMU_MID_P	(4095)
#define IMU_LOW_P	(4600)

#endif


/*double*/
#if CARR_MODE == 3
//CAN2
#define CAN_ID_FRIC_L	0x201U//
#define CAN_ID_FRIC_R	0x202U//

/*电机中值*/

#define MEC_MID_Y_F	(2026)//
#define MEC_MID_Y_B	(2026 + 4095)//

#define MEC_MID_P	(6700)// 5270 4755 4216


/* PID */

#define POWER_PID_P       8.f
#define POWER_PID_I       0.6f
#define POWER_PID_D       0.f
#define POWER_PID_I_MAX   5000.f
#define POWER_PID_O_MAX   8000.f
#define POWER_PID_B_ERR   0.f

/*Yaw转向pid设置*/
#define YAW_TURN_KP 0.5f
#define YAW_IMU_KP  1.6f 
#define PIT_IMU_KP  15.f 

#define IMU_HIG_P	(3200)//抬头最高角度 按下G键    hig3700-4700down 496 404
#define IMU_MID_P	(4095)
#define IMU_LOW_P	(4500)

#endif

/*new_mec*/
#if CARR_MODE == 4
//CAN2
#define CAN_ID_FRIC_L	0x201U//
#define CAN_ID_FRIC_R	0x202U//

/*电机中值*/

#define MEC_MID_Y_F	(4812)//
#define MEC_MID_Y_B	(4812 - 4095)//

#define MEC_MID_P	(6861)// 5270 4755 4216


/* PID */

#define POWER_PID_P       8.f
#define POWER_PID_I       0.6f
#define POWER_PID_D       0.f
#define POWER_PID_I_MAX   5000.f
#define POWER_PID_O_MAX   8000.f
#define POWER_PID_B_ERR   0.f

/*Yawpid设置*/
#define YAW_TURN_KP 0.22f
#define YAW_IMU_KP  1.6f 
#define PIT_IMU_KP  12.f 

#define IMU_HIG_P	(3500)//抬头最高角度 按下G键    hig3700-4700down 496 404
#define IMU_MID_P	(4095)
#define IMU_LOW_P	(4600)

#endif


















/* 电机类型索引 */
typedef enum {
	CHASSIS_POWER,
	CHASSIS_TURN,
	SHOOT_FRIC,
	SHOOT_BOX,	
	SHOOT_BARREL,
	GIMBAL,	
} motor_type_t;


/**电机设备索引**/

typedef enum {		
	POWER_0,
	POWER_1,
	POWER_2,
	POWER_3,	

	TURN_0,
	TURN_1,		
	TURN_2,	
	TURN_3,	
	
  CHAS_MOTOR_CNT,
} chas_motor_cnt_t;

typedef enum {		
	GIMB_Y,
	GIMB_P,
	
  GIMB_MOTOR_CNT,
} gimb_motor_cnt_t;

typedef enum {		
	FRIC_L,
	FRIC_R,	
	BOX,
	BARREL,
	
  SHOT_MOTOR_CNT,
} shot_motor_cnt_t;

/* PID类型索引 */

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	
	pid_ctrl_t	Turn_pid;
	
	pid_ctrl_t	current_pid;
	
	pid_ctrl_t  Imu_Out_Pid;
	pid_ctrl_t  Imu_Inn_Pid;	
	pid_ctrl_t  Mec_Out_Pid;
	pid_ctrl_t  Mec_Inn_Pid;
	
  float       out;
} motor_pid_t;

typedef enum {
	MOTOR_F,
	MOTOR_B
} motor_dir_t;

//电机信息
typedef struct motor_info_struct {
	uint16_t	  angle;
	int16_t		  speed;
	int16_t		  current;
	uint8_t     temperature;
	int16_t     Torque;
	uint16_t	  angle_prev;
	int32_t		  angle_sum;
	uint8_t		  init_flag;
	motor_dir_t dir;
	int16_t     MID;
	uint8_t		  offline_cnt;
	uint8_t		  offline_max_cnt;	
} motor_info_t;


/**
* 电机信息
* 电机pid
* 电机驱动
* 电机函数
* 设备状态
**/

typedef struct motor_struct{
	motor_info_t       *info;
	drv_can_t			     *driver;
	motor_pid_t        *pid;
	motor_type_t       type;
	dev_id_t				   id;
	dev_work_state_t	 work_state;
	dev_errno_t				 errno;
} motor_t;
#endif


