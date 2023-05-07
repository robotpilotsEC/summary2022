#ifndef __IMU_H
#define __IMU_H

#include "rp_config.h"
#include "rp_math.h"
#include "ALGO.h"
#include "bmi.h"


#define YAW           (imu_sensor.info->yaw)
#define PITCH         (imu_sensor.info->pitch)
#define ROLL          (imu_sensor.info->roll)
#define YAW_V         (imu_sensor.info->yaw_v)
#define YAW_V_WORLD   (imu_sensor.info->rate_yaw_w)

#define PITCH_V       (imu_sensor.info->rate_pitch)//(imu_sensor.info->pitch_v)
#define ROLL_V        (imu_sensor.info->roll_v)

typedef enum{
	pitch_longtime,
	yaw_longtime,
	IMU_CNT,
}imu_cnt_t;


/*     IMU    */
typedef struct{
	float imu_center;
	float imu_angle_sum;
	float imu_angle_pre;
}flag_longtime_t;

typedef struct imu_sensor_info_struct {
	float yaw;
	float pitch;
	float roll;
	
	float yaw_last;
	float pitch_last;
	float roll_last;
	
	float yaw_v;
	float pitch_v;
	float roll_v;
		
	float yaw_v_last;
	float pitch_v_last;
	float roll_v_last;
	
	
	short rate_yaw;
	short rate_pitch;
	short rate_pitch_last;
	
	short rate_roll;
		
	short rate_yaw_w;
	short rate_yaw_last;
	
	float rate_yaw_offset;
	float rate_pitch_offset;
	
	flag_longtime_t Longtime_imu_Array[IMU_CNT];
	
	uint8_t		init_flag;
	uint8_t   offline_cnt;
	uint8_t   offline_max_cnt;
} imu_sensor_info_t;



typedef struct imu_sensor_struct {
	imu_sensor_info_t	*info;
	drv_iic_t			*driver;
	void				(*init)(struct imu_sensor_struct *self);
	void				(*update)(struct imu_sensor_struct *self);
	void				(*check)(struct imu_sensor_struct *self);	
	void				(*heart_beat)(struct imu_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;	
} imu_sensor_t;

extern imu_sensor_t imu_sensor;

void Init_Imu(void);
char imu_offline_check(void);
void Imu_Data(void);

#endif


