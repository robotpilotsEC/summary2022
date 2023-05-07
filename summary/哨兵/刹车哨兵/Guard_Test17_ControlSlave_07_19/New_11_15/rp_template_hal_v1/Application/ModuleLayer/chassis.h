#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
#define LEFT -1
#define RIGHT 1

#define INITIAL_SPEED 1000
#define NORMALl_SPEED 3000
#define MID_SPEED 5000
#define HIGH_SPEED 7000

#define R_DRIVING_GEAR 0.07f
/* Exported types ------------------------------------------------------------*/
typedef enum {
	RUN_NORMAL, // ����Ѳ��
	RUN_COORDINATE, // ����ģʽ
	RUN_MIXED,     //���ģʽ
} chassis_mode_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	uint8_t mode;
	float		out;
} chassis_motor_pid_t;

//typedef struct {
//	pid_ctrl_t	angle;
//	float 		out;
//} chassis_z_pid_t;

typedef struct {
	chassis_motor_pid_t		(*motor)[CHASSIS_MOTOR_CNT];
} chassis_ctrl_t;

typedef struct {
	motor_t	*chas_motor[CHASSIS_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;

typedef struct {
	remote_mode_t		remote_mode;
	chassis_mode_t		local_mode;
}chassis_info_t;

typedef struct chassis{
	chassis_ctrl_t	*controller;
	chassis_dev_t	*dev;
	chassis_info_t	*info;
	bool init_mileage;
	bool brake_check;
	bool brake_start;
	bool brake_finished;
	int32_t rail_mileage;
	int8_t direction;
	int8_t run_dir;
	bool      spare_runnig;  //�����ܹ죬ɲ������ʧ��������
	bool			test_open;
	bool      check_road;      //��̨�ֿ���״̬���鿴��·
	bool      escape;           //������·
	bool      ctrl_sentry;       //�ӹ��ڱ�
	bool      test_sentry;      //�����ڱ�
	int8_t    control_dir;           //-1��1��-1����
	bool      hurt_data_update;   //�˺�����
	chassis_safety_status_t safety_status;

	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}chassis_t;

extern chassis_t chassis;

/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/

#endif
