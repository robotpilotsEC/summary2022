/**
 * @file        imu_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Imu Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "soft_iic_potocal.h"
#include "imu.h"
#include "S_function.h"



extern float halfT;// = 0.0005f 1ms T = halft*2000
extern kalman_pid_t kal_pid[KAL_CNT];
extern float Kp;
//全局
short gyrox, gyroy, gyroz;//陀螺仪
short accx, accy, accz;//加速度


void imu_sensor_update(imu_sensor_t *imu_sen)
{
	float T = halfT * 20;
	float rol_angle,pit_angle;
  imu_sensor_info_t *imu_info = imu_sen->info;
	
  BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
		
	imu_info->pitch_last = imu_info->pitch;
	imu_info->yaw_last   = imu_info->yaw;
	imu_info->roll_last  = imu_info->roll;

  BMI_Get_EulerAngle(&imu_info->pitch, &imu_info->roll, &imu_info->yaw, &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
 
	if(imu_info->pitch_last != 0)
	{
		if(imu_info->pitch == NULL)
		{
			imu_info->pitch = imu_info->pitch_last;
		}
	}

	imu_info->pitch = JUDGE_NULL(imu_info->pitch_last, imu_info->pitch);
	imu_info->roll = JUDGE_NULL(imu_info->roll_last, imu_info->roll);
	imu_info->yaw = JUDGE_NULL(imu_info->yaw_last, imu_info->yaw);


	if(abs(imu_info->pitch_last - imu_info->pitch) > 4090)
		imu_info->pitch = imu_info->pitch;
	else
		imu_info->pitch = 0.5f*imu_info->pitch_last + 0.5f*imu_info->pitch;
	
	if(abs(imu_info->yaw_last - imu_info->yaw) > 4090)
		imu_info->yaw = imu_info->yaw;
	else		
		imu_info->yaw   = 0.5f*imu_info->yaw_last   + 0.5f*imu_info->yaw;
	
	if(abs(imu_info->roll_last - imu_info->roll) > 4090)
		imu_info->roll = imu_info->roll;
	else
		imu_info->roll  = 0.5f*imu_info->roll_last  + 0.5f*imu_info->roll;
	
	imu_info->pitch_v_last = imu_info->pitch_v;
	imu_info->yaw_v_last   = imu_info->yaw_v;
	imu_info->roll_v_last  = imu_info->roll_v;
  
	imu_info->pitch_v = 0.7f*(Half_Turn(imu_info->pitch - imu_info->pitch_last,8191))/T + 0.3f*imu_info->pitch_v_last;
	imu_info->yaw_v   = 0.7f*(Half_Turn(imu_info->yaw - imu_info->yaw_last,8191))/T   + 0.3f*imu_info->yaw_v_last;
	imu_info->roll_v  = 0.7f*(Half_Turn(imu_info->roll  - imu_info->roll_last,8191))/T  + 0.3f*imu_info->roll_v_last;

//  imu_info->rate_pitch = imu_info->rate_pitch_last;
	
	imu_info->rate_pitch_last = imu_info->rate_pitch;
	
	
	
	imu_info->rate_roll  = gyrox;
	imu_info->rate_pitch = 0.7f * gyroy + 0.3f * imu_info->rate_pitch_last;
	imu_info->rate_yaw   = gyroz;

  pit_angle = (imu_info->rate_pitch / 22.7555f - 180) / 180 * PI;
  rol_angle = (imu_info->rate_roll / 22.7555f - 180) / 180 * PI;

  imu_info->rate_yaw_last = imu_info->rate_yaw_w;
	imu_info->rate_yaw_w    = arm_cos_f32(pit_angle) * imu_info->rate_yaw - arm_sin_f32(pit_angle) * imu_info->rate_roll;	

//	imu_info->rate_yaw_w = 0.7f * (gyroz * arm_cos_f32(rol_angle) + gyrox * arm_sin_f32(rol_angle) + \
//	                               gyroz * arm_cos_f32(pit_angle) + gyroy * arm_sin_f32(pit_angle)) + \
//											   0.3f * imu_info->rate_yaw_last;
//	
//	rat_y = (arm_cos_f32(pit_angle) * imu_info->rate_yaw - arm_sin_f32(pit_angle) * imu_info->rate_roll);
//	imu_info->rate_yaw_w = 0.7f * rat_y + 0.3f * imu_info->rate_yaw_last;
		
	
	imu_sen->check(imu_sen);
}


void imu_sensor_init(imu_sensor_t *imu_sen)
{
	int8_t rslt;
	uint32_t tickstart = HAL_GetTick();
	imu_sensor_info_t *imu_info = imu_sen->info;
	
	imu_sen->errno = NONE_ERR;
//  __set_PRIMASK(1);//关总中断
	rslt = BMI_Init();
	

	while(rslt)
	{
        // 如果初始化失败则重新初始化
        imu_sen->errno = DEV_INIT_ERR;
        rslt = BMI_Init();
	}
    //imu_init_errno = rslt;
//	__set_PRIMASK(0);//开总中断
		
	for(uint16_t i=0; i<250; i++) {
		BMI_Get_GRO(&imu_info->rate_pitch, &imu_info->rate_roll, &imu_info->rate_yaw);
		imu_info->rate_pitch_offset += imu_info->rate_pitch;
		imu_info->rate_yaw_offset += imu_info->rate_yaw;
	}
    /**
        @note
        如果上电的时候云台运动，会导致计算出来的静态偏差数值出错。如果每次上电的时候，静态偏差均
        差别不大的话，可以直接给定一个固定值。或者，另外对计算出来的偏差值做判断等。
    */
	imu_info->rate_pitch_offset /= 250.f;
	imu_info->rate_yaw_offset /= 250.f;

	if(imu_sen->id != DEV_ID_IMU)
		imu_sen->errno = DEV_ID_ERR;
}


