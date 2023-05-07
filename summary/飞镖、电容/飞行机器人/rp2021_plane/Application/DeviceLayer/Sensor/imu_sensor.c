/**
 * @file        imu_sensor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       IMU' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"

#include "imu_potocol.h"

#include "arm_math.h"
#include "rp_math.h"
//DEBUG:
#include "drv_io.h"
extern void imu_sensor_update(imu_sensor_t *imu_sen);
extern void imu_sensor_init(imu_sensor_t *imu_sen);

/* Private macro -------------------------------------------------------------*/
#define IMU_OFFLINE_MAX_CNT 0xff
/* Private function prototypes -----------------------------------------------*/
static void imu_sensor_check(imu_sensor_t *imu_sen);
static void imu_sensor_heart_beat(imu_sensor_t *imu_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
drv_iic_t 	imu_sensor_driver = {
	.type = DRV_IIC,
};

imu_sensor_info_t 	imu_sensor_info = {
	.offline_max_cnt = IMU_OFFLINE_MAX_CNT,
};

imu_sensor_t 	imu_sensor = {
	.info = &imu_sensor_info,
	.driver = &imu_sensor_driver,
	.init = imu_sensor_init,
	.update = imu_sensor_update,
	.check = imu_sensor_check,
	.heart_beat = imu_sensor_heart_beat,
	.work_state = DEV_ONLINE,
	.id = DEV_ID_IMU,	
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*
    rate_yaw 逆时针为负
    yaw 逆时针增大（正）
    rate_pitch 逆时针为正
    pitch 逆时针减小（负）
    rate_roll 逆时针为负
    roll 逆时针减小（负）
*/
float roll_cal, pitch_cal;
float pitch_cos, pitch_sin;
float roll_cos, roll_sin;
float imu_rateyaw, imu_ratepitch, imu_roll;
static void imu_sensor_check(imu_sensor_t *imu_sen)
{
    static uint16_t test_cnt = 0;
	imu_sensor_info_t *imu_info = imu_sen->info;
	// 加入滤波算法
    
    imu_info->rate_pitch -= imu_info->rate_pitch_offset;
    imu_info->rate_yaw -= imu_info->rate_yaw_offset;
    
//    roll数据不稳定    
    imu_roll = imu_info->roll;
    if((imu_roll > -5.f) && (imu_roll < 5.f)) {
        test_cnt++;
        if(test_cnt > 500)
            LED_BLUE_TOGGLE();
    } else {
        imu_info->roll = imu_roll > 0.f? (imu_roll-180.f) : (imu_roll+180.f);
    }

    roll_cal = imu_info->roll * 0.017452f;
    roll_cos = cos(roll_cal);
    roll_sin = sin(roll_sin);
    pitch_cal = imu_info->pitch * 0.017452f;
    pitch_cos = cos(pitch_cal);
    pitch_sin = sin(pitch_cal);
    
    // 假设roll轴转动很小，忽略其影响。只考虑pitch
    imu_rateyaw = imu_info->rate_yaw*pitch_cos + imu_info->rate_roll*pitch_sin;
    imu_ratepitch = (-imu_info->rate_pitch)*roll_cos + imu_info->rate_yaw*roll_sin;

    //imu_rateyaw = imu_info->rate_yaw*roll_cos*pitch_cos + imu_info->rate_pitch*roll_sin*pitch_cos + imu_info->rate_roll*pitch_sin;
    //imu_ratepitch = (-imu_info->rate_pitch)*roll_cos + imu_info->rate_yaw*roll_sin;
    imu_info->rate_yaw = imu_rateyaw;
    imu_info->rate_pitch = imu_ratepitch;
	
    imu_info->offline_cnt = 0;
}

static void imu_sensor_heart_beat(imu_sensor_t *imu_sen)
{
	imu_sen->work_state = DEV_ONLINE;
}
