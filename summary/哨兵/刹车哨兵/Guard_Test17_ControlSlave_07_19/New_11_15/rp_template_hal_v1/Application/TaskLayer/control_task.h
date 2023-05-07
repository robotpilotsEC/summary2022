/**
 * @file        control_task.h
 * @author      RobotPilots
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */
#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported variables ------------------------------------------------------------*/
extern int16_t NormalPwm[FRICTION_MOTOR_CNT];
/* Exported macro ------------------------------------------------------------*/
#define		LEADER_LEN_PACKED		8		//���ݰ����ȣ����Զ���
#define		RC_SLAVE_LEN_PACKED		4		//���ݰ����ȣ����Զ���

/* Exported types ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Leader */
typedef __packed struct
{
  uint8_t online : 1;//ң������λ
	uint8_t mode : 2;//����ģʽλ
	uint8_t attack_colour :1;//ʶ����ɫλ
	uint8_t stop_fire :1;  //��̨��ֹͣ����λ	
	uint8_t fric_open : 1;//Ħ���ֿ���
	uint8_t fire_open :1;//����̨���̿���λ
	uint8_t dial_lock_state :1; //���̽���λ
}leader_mode_t;

typedef __packed struct 
{
	float bullet_speed;
	bool 	shoot_update;		// ������ݸ���
	uint16_t cooling_heat;
	leader_mode_t ctrl_mode;
}Leader_Tx_Packet_t;

typedef struct master_info_struct
{
	Leader_Tx_Packet_t TxPacket;
	volatile bool init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
}leader_info_t;

typedef struct master_struct{
	leader_info_t *info;
	drv_can_t			        *driver;	
	void					(*init)(struct  master_struct *self);
	void					(*update)(struct  master_struct *self, uint8_t *rxBuf);	
	void				    (*check)(struct  master_struct *self);		
	void					(*heart_beat)(struct  master_struct *self);
	dev_work_state_t		work_state;
}leader_t;

extern leader_t	leader_sensor;
extern leader_info_t	leader_info;

/* RC_Slave */
typedef __packed struct 
{
	int16_t rc_slave_ch0;
	int16_t rc_slave_ch1;
}RC_Slave_Tx_Packet_t;

typedef struct rc_master_info_struct
{
	RC_Slave_Tx_Packet_t RC_Slave_TxPacket;
	volatile bool init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
}rc_leader_info_t;

typedef struct rc_master_struct{
	rc_leader_info_t *info;
	drv_can_t			        *driver;	
	void					(*init)(struct  rc_master_struct *self);
	void					(*update)(struct  rc_master_struct *self, uint8_t *rxBuf);	
	void				    (*check)(struct  rc_master_struct *self);		
	void					(*heart_beat)(struct  rc_master_struct *self);
	dev_work_state_t		work_state;
}rc_leader_t;

extern rc_leader_t	rc_leader_sensor;
extern rc_leader_info_t	rc_leader_info;

/* Exported functions --------------------------------------------------------*/

#endif
