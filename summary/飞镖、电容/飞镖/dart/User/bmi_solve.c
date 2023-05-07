#include "bmi_solve.h"
#include "bmi.h"
#include "motor.h"

#define abs(x) 					((x)>0? (x):(-(x)))

bmi_info_t  bmi_info[] = {
    [BMI_PT] = {
        .pitch = 0,
        .roll = 0,
        .yaw = 0,
        .yaw_sum = 0,
    },
    [BMI_CHAS] = {
        .pitch = 0,
        .roll = 0,
        .yaw = 0,
        .yaw_sum = 0,
    },
};

void IMU_yaw_sum(bmi_info_t *bmi_info)
{
	float err;
//    static float fric_close_mix = 0.0001266;
//    static float fric_s30_mix = 0.00027;
//    static float fric_s15_mix = 0.00011;
	
	/* 未初始化 */
	if( !bmi_info->init_flag )
	{
		bmi_info->init_flag = 1;
		bmi_info->yaw_prev = bmi_info->yaw;
		bmi_info->yaw_sum = 0;
	}
    
//	  偏差修正
//    if(chassis_motor[FRIC_LEFT].info->speed > -200){
//        err = bmi_info->yaw - bmi_info->yaw_prev + fric_close_mix;
//    }else if(chassis_motor[FRIC_LEFT].info->speed > -4800){
//        err = bmi_info->yaw - bmi_info->yaw_prev + fric_s15_mix;
//    }else err = bmi_info->yaw - bmi_info->yaw_prev + fric_s30_mix;
	
	/* 过零点 */
	if(abs(err) > 180)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			bmi_info->yaw_sum += -360 + err;
		/* 8191↑ -> 0 */
		else
			bmi_info->yaw_sum += 360 + err;
	}
	/* 未过零点 */
	else
	{
		bmi_info->yaw_sum += err;
	}
	
	bmi_info->yaw_prev = bmi_info->yaw;		
}

