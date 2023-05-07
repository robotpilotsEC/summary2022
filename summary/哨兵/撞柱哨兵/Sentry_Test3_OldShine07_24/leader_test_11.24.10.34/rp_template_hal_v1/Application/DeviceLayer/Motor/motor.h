/**
 * @file        chassis_motor.h
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       Chassis Motor(RM3508).
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(24-October-2021)
 *                  1.将init与update函数定义在本文件中
 *              v1.1.1(8-November-2021)
 *                  1.更新驱动函数
 */

#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define		MASTER_LEN_PACKED		8		//数据包长度，可自定义
#define		RC_MASTER_LEN_PACKED		4		//数据包长度，可自定义

/* Exported types ------------------------------------------------------------*/
typedef struct motor_info_struct {
	volatile uint16_t		angle;
	volatile int16_t		speed;
	volatile int16_t		current;
	volatile int16_t		torque;	
  volatile uint8_t    temperature;
	volatile uint16_t		angle_prev;
	volatile int32_t		angle_sum;
	volatile bool	    	init_flag;
	volatile uint8_t    offline_cnt;
	const    uint8_t		offline_max_cnt;	
} motor_info_t;

typedef struct motor_struct {
	motor_info_t        *info;
	drv_can_t			        *driver;
	void				        (*init)(struct motor_struct *self);
	void				        (*update)(struct motor_struct *self, uint8_t *rxBuf);
	void				        (*check)(struct motor_struct *self);	
	void					    (*heart_beat)(struct motor_struct *self);
	volatile dev_work_state_t   work_state;
	volatile dev_errno_t		errno;
	const    dev_id_t			id;
} motor_t;

typedef __packed struct
{
  uint8_t online : 1;//遥控连接位
	uint8_t mode : 2;//控制模式位
	uint8_t attack_colour :1;//识别颜色位
	uint8_t stop_fire :1;  //云台手停止开火位
	uint8_t fric_open : 1;//摩擦轮控制
	uint8_t fire_open :1;//射击开启位
	uint8_t dial_lock_state :1; //拨盘解锁位

}master_mode_t;

typedef __packed struct 
{
	float bullet_speed;
	bool 	shoot_update;		// 射击数据更新
	uint16_t cooling_heat;
	master_mode_t ctrl_mode;
}Master_Rx_Packet_t;

typedef struct master_info_struct
{
	Master_Rx_Packet_t RxPacket;
	uint8_t     mode_pre;	
	bool        mode_siwtch;	
	volatile bool init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
}master_info_t;

typedef struct master_struct{
	master_info_t *info;
	void					(*init)(struct  master_struct *self);
	void					(*update)(struct  master_struct *self, uint8_t *rxBuf);	
	void				    (*check)(struct  master_struct *self);		
	void					(*heart_beat)(struct  master_struct *self);
	dev_work_state_t		work_state;
}master_t;

typedef __packed struct 
{
	int16_t rc_master_ch0;
	int16_t rc_master_ch1;	
}RC_Master_Rx_Packet_t;

typedef struct rc_master_info_struct
{
	RC_Master_Rx_Packet_t RC_Master_RxPacket;
	volatile bool init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
}rc_master_info_t;

typedef struct rc_master_struct{
	rc_master_info_t *info;
	void					(*init)(struct  rc_master_struct *self);
	void					(*update)(struct  rc_master_struct *self, uint8_t *rxBuf);	
	void				    (*check)(struct  rc_master_struct *self);		
	void					(*heart_beat)(struct  rc_master_struct *self);
	dev_work_state_t		work_state;
}rc_master_t;

extern motor_t motor[MOTOR_CNT];
extern master_info_t 	master_info;
extern master_t master_sensor;
extern rc_master_info_t 	rc_master_info;
extern rc_master_t rc_master_sensor;

/* Exported functions --------------------------------------------------------*/


#endif
