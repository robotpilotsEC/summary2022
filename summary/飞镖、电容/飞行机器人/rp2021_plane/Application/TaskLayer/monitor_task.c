/**
 * @file        monitor_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"

#include "drv_io.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//dev_work_state_t dev_work_state_list[DEV_ID_CNT]; 
//uint8_t js_yaw_work_state, js_pit_work_state;
static void device_heart_beat(void)
{
	rc_sensor.heart_beat(&rc_sensor);
	imu_sensor.heart_beat(&imu_sensor);
	judge_sensor.heart_beat(&judge_sensor);
	vision_sensor.heart_beat(&vision_sensor);
	gimbal_motor[YAW].heart_beat(&gimbal_motor[YAW]);
	gimbal_motor[PIT].heart_beat(&gimbal_motor[PIT]);
	turnplate_motor.heart_beat(&turnplate_motor);
    
//    dev_work_state_list[DEV_ID_RC] = rc_sensor.work_state;
//    dev_work_state_list[DEV_ID_GIMBAL_YAW] = gimbal_motor[YAW].work_state;
//    dev_work_state_list[DEV_ID_GIMBAL_PIT] = gimbal_motor[PIT].work_state;
//    
//    js_yaw_work_state = gimbal_motor[YAW].work_state;
//    js_pit_work_state = gimbal_motor[PIT].work_state;
}

static void system_led_flash(void)
{
	static uint16_t led_blue_flash = 0;
	
	led_blue_flash++;
	if(led_blue_flash > 500) 
	{
		led_blue_flash = 0;
		LED_RED_TOGGLE();
	}
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	数据消息处理任务
 */
extern osThreadId MonitorTaskHandle;
uint32_t MonitorTaskStackRemain = 0;	//分配256，剩余117 - 20/11/24
//uint32_t imu_start, imu_end, imu_gap;
void StartMonitorTask(void const * argument)
{
	LASER_ON();
	//LED_RED_ON();
    //imu_start = imu_end = xTaskGetTickCount();
	for(;;)
	{
        //imu_start = xTaskGetTickCount();
        //imu_gap = imu_start - imu_end;
		system_led_flash();
        device_heart_beat();
		imu_sensor.update(&imu_sensor);
		MonitorTaskStackRemain = uxTaskGetStackHighWaterMark(MonitorTaskHandle);
        //imu_end = xTaskGetTickCount();
		osDelay(1);
	}
}

void MonitorTask(void)
{
    system_led_flash();
    device_heart_beat();
    imu_sensor.update(&imu_sensor);
}
