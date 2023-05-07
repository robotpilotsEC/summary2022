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
uint8_t Vision_SentData[LEN_VISION_TX_PACKET];//���͸��Ӿ�������
//uint8_t Vision_TxBuffer[VISION_TX_BUFFER_LEN];
/* Exported variables --------------------------------------------------------*/
//��������װ����̨�����ʷ�ǶȵĶ���
Queue_t pitch_history_queue;
Queue_t yaw_history_queue;
//���Ը���ϵ��Kp
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
	//�Ӿ����ݳ�ʼ��
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
 *	@brief	����������
 */
void Vision_Correct(void)
{
	vision.pitch_angle_err_corrected = vis_pitch_corret_kp * vision.pitch_angle_err;
	vision.yaw_angle_err_corrected = vis_yaw_corret_kp * vision.yaw_angle_err;
}


static void Vision_Normal(void)
{
	static uint16_t active_cnt = 0;  //ʶ�𵽼���
	static uint16_t lost_cnt = 0;    //ʶ��ʧ����
	
	if(vision_rx.State->rx_data_update)  //�Ӿ����ݸ���
	{
		vision.pitch_angle_raw = vision.rx_data->RxData->pitch_angle / 360.f * 8191; //����Ŀ����ԽǶ�
		vision.yaw_angle_raw = vision.rx_data->RxData->yaw_angle / 360.f * 8191;

		if(vision_rx.RxData->identify_target)  //Ŀ��ʶ��
		{
			active_cnt++;
			if(active_cnt >= ACTIVE_MAX_CNT)
			{
				vision.identify_ok = true;   
				vision.target_update = true;
				active_cnt = 0;
				lost_cnt = 0;
			}
			//�������
			vision.pitch_angle_err = vision.pitch_angle_raw - motor[GIMB_PITCH].info->angle;	
			vision.yaw_angle_err = vision.yaw_angle_raw - motor[GIMB_YAW].info->angle;
			//����㴦��
			ZeroCrossing_Handle();

		}
		else   //Ŀ�궪ʧ
		{
			lost_cnt++;
			if(lost_cnt >= LOST_MAX_CNT)
			{
				vision.identify_ok = false;	
				active_cnt = 0;
				lost_cnt = 0;
			}
		}
		//�������
		vision.distance_filtered = KalmanFilter(&vision_distance_get, vision.rx_data->RxData->distance);
		vision_rx.State->rx_data_update = false;
	}
	else   //�Ӿ����ݻ�û�и���
	{
		
	}
}


static void Vision_Reset(void)
{
	//�Ӿ����ݳ�ʼ��
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
  static uint8_t Sent_cnt = 0;//���ͼ��
	
//	vision_sensor_info.TxPacket.TxData.yaw_measure = imu_sensor.info->yaw;	   //�����ǽǶ�
//	vision_sensor_info.TxPacket.TxData.pitch_measure = imu_sensor.info->roll;
	vision_sensor_info.TxPacket.TxData.yaw_measure = motor[GIMB_YAW].info->angle * 1.0 / 8191.f * 360;   //����Ƕ�  0~8191
	vision_sensor_info.TxPacket.TxData.pitch_measure = motor[GIMB_PITCH].info->angle * 1.0 / 8191.f * 360; 

	vision_sensor_info.TxPacket.TxData.attack_color = master_info.RxPacket.ctrl_mode.attack_colour + 1;   //ʶ����ɫ
	vision_sensor_info.TxPacket.TxData.time_stamp = HAL_GetTick();               //ʱ���
	vision_sensor_info.TxPacket.TxData.anti_gyro = 0;         //�����ݱ�־���Ӿ�����̶�����0
	vision_sensor_info.TxPacket.TxData.bullet_speed = 30;	             //����
	
	vision_sensor_info.TxPacket.FrameHeader.sof = VISION_SOF;	     //֡ͷ
	vision_sensor_info.TxPacket.FrameHeader.cmd_id = CMD_AIM_SENTRY;   //������
	
  Sent_cnt++;
		
	memcpy(Vision_SentData,&vision_sensor_info.TxPacket,LEN_VISION_TX_PACKET);   //���������ݴ��
	
	Append_CRC8_Check_Sum(Vision_SentData,LEN_FRAME_HEADER);         //���У����
	Append_CRC16_Check_Sum(Vision_SentData,LEN_VISION_TX_PACKET);

	if(Sent_cnt >= 3)
	{
		vision_sensor_driver.tx_byte(Vision_SentData,LEN_VISION_TX_PACKET);    //����
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
//		//������
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

