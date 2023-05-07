/* Includes ------------------------------------------------------------------*/
#include "vision_task.h"
#include "device.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "rp_math.h"
#include "kalman.h"
#include "moving_filter.h"

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Vision_SentData[LEN_VISION_TX_PACKET];//发送给视觉的数组
//uint8_t Vision_TxBuffer[VISION_TX_BUFFER_LEN];
/* Exported variables --------------------------------------------------------*/
//定义两个装载云台电机历史角度的队列
Queue_t pitch_history_queue;
Queue_t yaw_history_queue;
//线性跟随系数Kp
float vis_pitch_corret_kp = 1;
float vis_yaw_corret_kp = 1;

Vision_RxData_t vision_rx = {
	.Cmd = &vision_sensor_info.RxPacket.FrameHeader.cmd_id,
	.RxData = &vision_sensor_info.RxPacket.RxData,
	.State = &vision_sensor_info.State,
};

vision_t vision;
/* Private functions ---------------------------------------------------------*/
void QueueInit(void)
{
	Queue_Init(&pitch_history_queue);
	Queue_Init(&yaw_history_queue);
	
	Queue_Clear(&pitch_history_queue);
	Queue_Clear(&yaw_history_queue);
}

void QueueUpdate(void)
{
	Queue_Update(&pitch_history_queue,(float)motor[GIMB_PITCH].info->angle);
	Queue_Update(&yaw_history_queue,(float)motor[GIMB_YAW].info->angle);
}


static void Vision_Init(void)
{
	//视觉数据初始化
	vision.rx_data = &vision_rx;
	vision.identify_ok = false;
	vision.target_update = false;
	
	vision.pitch_angle_raw = 0;
	vision.yaw_angle_raw = 0;

	vision.pitch_angle_err = 0;
	vision.yaw_angle_err = 0;

	vision.pitch_angle_err_corrected = 0;
	vision.yaw_angle_err_corrected = 0;
	
	vision.pitch_angle_offset = 0;
	vision.yaw_angle_offset = 0;
}

static void ZeroCrossing_Handle(void)
{
	float pitch_err_buf;
	float yaw_err_buf;
	
	pitch_err_buf = vision.pitch_angle_err;
	yaw_err_buf = vision.yaw_angle_err;
	
	//pitch
	if(abs(pitch_err_buf) > 4095)
	{
		if(pitch_err_buf >= 0)
		{
			vision.pitch_angle_err -= 8191 + pitch_err_buf;
		}
		else if(pitch_err_buf < 0)
		{
			vision.pitch_angle_err += 8191 + pitch_err_buf;
		}
	}
	//yaw
	if(abs(yaw_err_buf) > 4095)
	{
		if(yaw_err_buf >= 0)
		{
			vision.yaw_angle_err -= 8191 + yaw_err_buf;
		}
		else if(yaw_err_buf < 0)
		{
			vision.yaw_angle_err += 8191 + yaw_err_buf;
		}
	}
}

/**
 *	@brief	跟随误差矫正
 */
void Vision_Correct(void)
{
	vision.pitch_angle_err_corrected = vis_pitch_corret_kp * vision.pitch_angle_err;
	vision.yaw_angle_err_corrected = vis_yaw_corret_kp * vision.yaw_angle_err;
}


static void Vision_Normal(void)
{
	static uint16_t active_cnt = 0;  //识别到计数
	static uint16_t lost_cnt = 0;    //识别丢失计数
	
	if(vision_rx.State->rx_data_update)  //视觉数据更新
	{
		vision.pitch_angle_raw = vision.rx_data->RxData->pitch_angle / 360.f * 8191; //更新目标绝对角度
		vision.yaw_angle_raw = vision.rx_data->RxData->yaw_angle / 360.f * 8191;

		if(vision_rx.RxData->identify_target)  //目标识别
		{
			active_cnt++;
			if(active_cnt >= ACTIVE_MAX_CNT)
			{
				vision.identify_ok = true;   
				vision.target_update = true;
				active_cnt = 0;
				lost_cnt = 0;
			}
			//计算误差
			vision.pitch_angle_err = vision.pitch_angle_raw - motor[GIMB_PITCH].info->angle;	
			vision.yaw_angle_err = vision.yaw_angle_raw - motor[GIMB_YAW].info->angle;
			//过零点处理
			ZeroCrossing_Handle();

		}
		else   //目标丢失
		{
			lost_cnt++;
			if(lost_cnt >= LOST_MAX_CNT)
			{
				vision.identify_ok = false;	
				active_cnt = 0;
				lost_cnt = 0;
			}
		}
		//距离更新
		vision.distance_filtered = KalmanFilter(&vision_distance_get, vision.rx_data->RxData->distance);
		vision_rx.State->rx_data_update = false;
	}
	else   //视觉数据还没有更新
	{
		
	}
}


static void Vision_Reset(void)
{
	//视觉数据初始化
	vision_rx.RxData->anti_gyro = 0;
	vision_rx.RxData->distance = 0;
	vision_rx.RxData->identify_target = 0;
	vision_rx.RxData->pitch_angle = 0;
	vision_rx.RxData->yaw_angle = 0;
	
	vision.identify_ok = false;
	vision.target_update = false;
	
	
	vision.pitch_angle_raw = 0;
	vision.yaw_angle_raw = 0;

//	vision.pitch_angle_corrected = 0;
//	vision.yaw_angle_corrected = 0;
	
	vision.pitch_angle_offset = 0;
	vision.yaw_angle_offset = 0;
	
}	

static void Sent_to_Vision_Version2_1(void)
{	 
  static uint8_t Sent_cnt = 0;//发送间隔
	
//	vision_sensor_info.TxPacket.TxData.yaw_measure = imu_sensor.info->yaw;	   //陀螺仪角度
//	vision_sensor_info.TxPacket.TxData.pitch_measure = imu_sensor.info->roll;
	vision_sensor_info.TxPacket.TxData.yaw_measure = motor[GIMB_YAW].info->angle * 1.0 / 8191.f * 360;   //电机角度  0~8191
	vision_sensor_info.TxPacket.TxData.pitch_measure = motor[GIMB_PITCH].info->angle * 1.0 / 8191.f * 360; 

	vision_sensor_info.TxPacket.TxData.attack_color = master_info.RxPacket.ctrl_mode.attack_colour + 1;   //识别颜色
	vision_sensor_info.TxPacket.TxData.time_stamp = HAL_GetTick();               //时间戳
	vision_sensor_info.TxPacket.TxData.anti_gyro = 0;         //反陀螺标志，视觉处理固定发送0
	vision_sensor_info.TxPacket.TxData.bullet_speed = 30;	             //射速
	
	vision_sensor_info.TxPacket.FrameHeader.sof = VISION_SOF;	     //帧头
	vision_sensor_info.TxPacket.FrameHeader.cmd_id = CMD_AIM_SENTRY;   //命令码
	
  Sent_cnt++;
		
	memcpy(Vision_SentData,&vision_sensor_info.TxPacket,LEN_VISION_TX_PACKET);   //将待发数据打包
	
	Append_CRC8_Check_Sum(Vision_SentData,LEN_FRAME_HEADER);         //添加校验码
	Append_CRC16_Check_Sum(Vision_SentData,LEN_VISION_TX_PACKET);

	if(Sent_cnt >= 3)
	{
		vision_sensor_driver.tx_byte(Vision_SentData,LEN_VISION_TX_PACKET);    //发送
		Sent_cnt = 0;
	}
}




void StartVisionTask(void const * argument)
{
	Vision_Init();
	for(;;)
	{
//		if((sys.state == SYS_STATE_NORMAL) && (vision_sensor.work_state == DEV_ONLINE) && (flag.gimbal.reset_ok))
//		{	
//		//测试用
//		if((sys.state == SYS_STATE_NORMAL) && (vision_sensor.work_state == DEV_ONLINE))
//		{	
		Sent_to_Vision_Version2_1();		
		
		if(vision_sensor.work_state == DEV_ONLINE)
		{	
			Vision_Normal();
		}
		else
		{
			Vision_Reset();
		}

//    }
    osDelay(2);
  }
}

