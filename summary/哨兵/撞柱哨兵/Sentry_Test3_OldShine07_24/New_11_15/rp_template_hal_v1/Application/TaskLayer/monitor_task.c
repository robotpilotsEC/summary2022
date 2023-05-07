/**
 * @file        monitor_task.c
 * @author      RobotPilots
 * @Version     V1.0.1
 * @brief       Monitor&Test Center
 * @update      
 *              v1.0(9-November-2020)
 *              v1.0.1(13-November-2021)
 *                  1.添加任务通知机制，event_notify()
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"
#include "vision_task.h"
#include "drv_io.h"
#include "drv_can.h"
#include "drv_haltick.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"
#include "bmi.h"
#include "drv_tim.h"
#include "dial.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t send_control_time = 0;
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void imu_Kp_Update(void)
{
	static uint16_t imu_1s_cnt = 0;
	if(Kp == 10)
	{
		imu_1s_cnt++;
	}
	if(imu_1s_cnt == 3000)  //3s到了
	{
		Kp = 0.1;   //初始化Kp等于10，数据稳定后改成0.1
		imu_1s_cnt = 3001;
	}
}

static void device_heart_beat(void)
{
	rc_sensor.heart_beat(&rc_sensor);
	imu_sensor.heart_beat(&imu_sensor);
	path_sensor.heart_beat(&path_sensor);
	judge_sensor.heart_beat(&judge_sensor);
	vision_sensor.heart_beat(&vision_sensor);
	leader_sensor.heart_beat(&leader_sensor);
	rc_leader_sensor.heart_beat(&rc_leader_sensor);
	motor[DIAL].heart_beat(&motor[DIAL]);
	motor[GIMB_PITCH].heart_beat(&motor[GIMB_PITCH]);
	motor[GIMB_YAW].heart_beat(&motor[GIMB_YAW]);
	motor[CHASSIS].heart_beat(&motor[CHASSIS]);
}

static void esc_standard(void)
{
	//电调校准
	if(ESC_Standard(&standard_flg) == true)
	{
		dial.fric_unlock = true;
	}
}

static void system_led_flash(void)
{
	static uint16_t led_blue_flash = 0;
	
	led_blue_flash++;
	if(led_blue_flash > 500) 
	{
		led_blue_flash = 0;
		LED_BLUE_TOGGLE();
	}
}

extern osThreadId SendTaskHandle;
uint32_t event = 0;
static void event_notify(void)
{
    uint32_t can_send_event = 0;
    
    if(CAN_MailboxReadyForTx(&hcan1Mailbox)) {
        SET_EVENT(event, EVENT_SEND_CAN1_MAILBOX);
    }
    
    if(CAN_MailboxReadyForTx(&hcan2Mailbox)) {
        SET_EVENT(event, EVENT_SEND_CAN2_MAILBOX);
    }
    
    can_send_event = GET_EVENT(event, EVENT_SEND_CAN1_MAILBOX|EVENT_SEND_CAN2_MAILBOX);
    if(can_send_event != 0) {
        send_control_time = micros();
        xTaskNotify((TaskHandle_t)SendTaskHandle,
                    (uint32_t)can_send_event,
                    (eNotifyAction)eSetBits);
        
        CLEAR_EVENT(event, can_send_event);
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统监控任务
 */
void StartMonitorTask(void const * argument)
{
	//LED_RED_ON();
	QueueInit();

	for(;;)
	{
        taskENTER_CRITICAL();
		imu_Kp_Update();
		imu_sensor.update(&imu_sensor);     //陀螺仪数据更新
		path_sensor.update(&path_sensor);   //编码器电机数据更新
		path_sensor.check(&path_sensor);   //编码器电机数据更新		
        taskEXIT_CRITICAL();
        
		system_led_flash();
		device_heart_beat();
		esc_standard();       //电调校准
		QueueUpdate();       //云台数据队列更新
        event_notify();
        
		osDelay(1);
	}
}
