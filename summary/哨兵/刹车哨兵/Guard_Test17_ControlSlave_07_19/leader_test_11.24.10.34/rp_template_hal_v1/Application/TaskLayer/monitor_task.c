/**
 * @file        monitor_task.c
 * @author      RobotPilots
 * @Version     V1.0.1
 * @brief       Monitor&Test Center
 * @update      
 *              v1.0(9-November-2020)
 *              v1.0.1(13-November-2021)
 *                  1.�������֪ͨ���ƣ�event_notify()
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"
#include "drv_io.h"
#include "drv_can.h"
#include "drv_haltick.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"
#include "moving_filter.h"
#include "vision_task.h"
#include "bmi.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t send_control_time = 0;
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void imu_Kp_Update(void)
{
	static uint16_t imu_1s_cnt = 0;
	if(Kp == 10)
	{
		imu_1s_cnt++;
	}
	if(imu_1s_cnt == 3000)  //3s����
	{
		Kp = 0.1;   //��ʼ��Kp����10�������ȶ���ĳ�0.1
		imu_1s_cnt = 3001;
	}
}

static void device_heart_beat(void)
{
	rc_sensor.heart_beat(&rc_sensor);
	imu_sensor.heart_beat(&imu_sensor);
	vision_sensor.heart_beat(&vision_sensor);
	master_sensor.heart_beat(&master_sensor);
	rc_master_sensor.heart_beat(&rc_master_sensor);
	motor[DIAL].heart_beat(&motor[DIAL]);
	motor[FRICTION_L].heart_beat(&motor[FRICTION_L]);
	motor[FRICTION_R].heart_beat(&motor[FRICTION_R]);
	motor[GIMB_PITCH].heart_beat(&motor[GIMB_PITCH]);
	motor[GIMB_YAW].heart_beat(&motor[GIMB_YAW]);
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
 *	@brief	ϵͳ�������
 */
void StartMonitorTask(void const * argument)
{
	//LED_RED_ON();
	QueueInit();
	for(;;)
	{
        taskENTER_CRITICAL();
		imu_Kp_Update();
		imu_sensor.update(&imu_sensor);     //���������ݸ���
        taskEXIT_CRITICAL();
		
		system_led_flash();
		device_heart_beat();
		QueueUpdate();       //��̨���ݶ��и���
        event_notify();
        
		osDelay(1);
	}
}
