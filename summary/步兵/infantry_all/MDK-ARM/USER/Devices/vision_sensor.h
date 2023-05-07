#ifndef __VISION_SENSOR_H
#define __VISION_SENSOR_H
/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "uart_drv.h"



#define VISION_ONLINE  vision_sensor.work_state == DEV_ONLINE
#define VISION_OFFLINE vision_sensor.work_state == DEV_OFFLINE

/* Exported macro ------------------------------------------------------------*/
//��ʼ�ֽڣ�Э��̶�β0xA5
#define 	VISION_SOF					(0xA5)

/* Exported types ------------------------------------------------------------*/
/**
 *	���->�Ӿ�
 */
typedef enum {
	CMD_AIM_OFF 		    = 0x00,	// ����������
	CMD_AIM_AUTO		    = 0x01,	// ��������
	CMD_AIM_SMALL_BUFF	= 0x02,	// ʶ��С��
	CMD_AIM_BIG_BUFF	  = 0x03,	// ʶ����
	CMD_AIM_ANTOP	   	  = 0x04,	// �����ڱ�
	CMD_AIM_ANDF		    = 0x05	// �������
} Vision_Cmd_Id_t;


/* ���ݳ��� */
typedef enum {
	/* Std */
	LEN_VISION_RX_PACKET	= 16,	// ���հ���������
	LEN_VISION_TX_PACKET	= 15,	// ���Ͱ���������

	LEN_RX_DATA 			    = LEN_VISION_RX_PACKET - 5,	// �������ݶγ���
	LEN_TX_DATA 			    = LEN_VISION_TX_PACKET - 5,	// �������ݶγ���
	
	LEN_FRAME_HEADER 	  	= 3,	// ֡ͷ����
	LEN_FRAME_TAILER 		  = 2,	// ֡βCRC16
} Vision_Data_Length_t;

/* ֡ͷ�ֽ�ƫ�� */
typedef enum {
	sof			  = 0,
	Cmd_ID		= 1,
	Crc8		  = 2,
	Data		  = LEN_FRAME_HEADER,
	TX_CRC16	= LEN_FRAME_HEADER + LEN_TX_DATA,
} Vision_Frame_Header_Offset_t;

/* ֡ͷ��ʽ */
typedef __packed struct
{
	uint8_t  			    sof;		// ͬ��ͷ
	Vision_Cmd_Id_t  	cmd_id;	// ������
	uint8_t  			    crc8;		// CRC8У����
} Vision_Frame_Header_t;

/* ֡β��ʽ */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16У����
} Vision_Frame_Tailer_t;
/* �������ݶθ�ʽ */





typedef __packed struct 
{
	float 	pitch_angle;
	float 	yaw_angle;	 

	char    is_find_Target;
	char    is_find_Dafu;	

	char    zm_shoot_enable;	   
	
	char    data[LEN_VISION_RX_PACKET];
	float   jiesuan[(LEN_VISION_RX_PACKET-5)/4];
} Vision_Rx_Data_t;      //3float,6char

/* ���հ���ʽ */
typedef __packed struct 
{
	Vision_Frame_Header_t FrameHeader;	// ֡ͷ
	Vision_Rx_Data_t	    RxData;		    // ����
	Vision_Frame_Tailer_t FrameTailer;	// ֡β	
} Vision_Rx_Packet_t;

/* �������ݶθ�ʽ */
typedef __packed struct
{
	float   yaw;
	float   pitch;
	uint8_t fric_speed;		// ���ٵ�λ(���ݵȼ�����)
	uint8_t my_color;		// ���Լ�����ɫ

} Vision_Tx_Data_t;

/* ���Ͱ���ʽ */
typedef __packed struct
{
	Vision_Frame_Header_t FrameHeader;	// ֡ͷ
	Vision_Tx_Data_t	    TxData;		    // ����
	Vision_Frame_Tailer_t FrameTailer;	// ֡β		
} Vision_Tx_Packet_t;
/**
 *	@brief	�Ӿ�ģʽ
 */
typedef enum
{
	VISION_MODE_MANUAL		  = 0,	// �ֶ�ģʽ
	VISION_MODE_AUTO		    = 1,	// ����ģʽ
	VISION_MODE_BIG_BUFF	  = 2,	// ����ģʽ
	VISION_MODE_SMALL_BUFF	= 3,	// ��С��ģʽ
} Vision_Mode_t;
/* ������ʶ���� */
typedef struct
{
	uint8_t 		  my_color;			 // ��0/1��ʾ��ɫ
	Vision_Mode_t	mode;				   // �Ӿ�ģʽ
	uint8_t  		  rx_data_valid; // �������ݵ���ȷ��
	uint16_t 		  rx_err_cnt;		 // �������ݵĴ���ͳ��
	uint32_t		  rx_cnt;				 // �������ݰ���ͳ��
	bool		      rx_data_update;// ���������Ƿ����
	uint32_t 		  rx_time_prev;	 // �������ݵ�ǰһʱ��
	uint32_t 		  rx_time_now;	 // �������ݵĵ�ǰʱ��
	uint16_t 		  rx_time_fps;	 // ֡��
	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;	
} Vision_State_t;

/* �Ӿ�ͨ�����ݰ���ʽ */
typedef struct {
	Vision_Rx_Packet_t RxPacket;
	Vision_Tx_Packet_t TxPacket;
	Vision_State_t     State;
} vision_info_t;

typedef struct vision_sensor_struct {
	vision_info_t	    *info;
	drv_uart_t		    *driver;
	void				     (*init)(struct vision_sensor_struct *self);
	void				     (*update)(struct vision_sensor_struct *self, uint8_t *rxBuf);
	void				     (*check)(struct vision_sensor_struct *self);	
	void				     (*heart_beat)(struct vision_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			  errno;
	dev_id_t			    id;
} vision_sensor_t;

extern vision_info_t    vision_sensor_info;
extern vision_sensor_t	vision_sensor;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/



#endif

