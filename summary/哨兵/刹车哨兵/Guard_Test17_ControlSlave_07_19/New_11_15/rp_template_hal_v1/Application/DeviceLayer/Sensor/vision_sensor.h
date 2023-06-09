#ifndef __VISION_SENSOR_H
#define __VISION_SENSOR_H
/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "drv_uart.h"
/* Exported macro ------------------------------------------------------------*/
//起始字节，协议固定尾0xA5
#define 	VISION_SOF					(0xA5)
//长度根据协议定义
#define		VISION_LEN_HEADER		3			//帧头长
#define 	VISION_LEN_DATA 		15    //数据段长度，可自定义
#define     VISION_LEN_TAIL			2			//频尾CRC16
#define		VISION_LEN_PACKED		20		//数据包长度，可自定义

/* Exported types ------------------------------------------------------------*/
/**
 *	电控->视觉
 */
typedef enum {
	CMD_AIM_OFF 		= 0x00,	// 不启动自瞄
	CMD_AIM_AUTO		= 0x01,	// 启动自瞄
	CMD_AIM_SMALL_BUFF	= 0x02,	// 识别小符
	CMD_AIM_BIG_BUFF	= 0x03,	// 识别大符
	CMD_AIM_SENTRY		= 0x04,	// 击打哨兵
	CMD_AIM_BASE		= 0x05	// 吊射基地
} Vision_Cmd_Id_t;
/* 帧头字节偏移 */
typedef enum {
	SOF			= 0,
	CMD_ID		= 1,
	CRC8		= 2,
	DATA		= 3,
	TX_CRC16	= 20,
} Vision_Frame_Header_Offset_t;

/* 数据长度 */
typedef enum {
	/* Std */
	LEN_FRAME_HEADER 		= 3,	// 帧头长度
	LEN_RX_DATA 			= 15,	// 接收数据段长度
	LEN_TX_DATA 			= 15,	// 发送数据段长度
	LEN_FRAME_TAILER 		= 2,	// 帧尾CRC16
	LEN_VISION_RX_PACKET	= 20,	// 接收包整包长度
	LEN_VISION_TX_PACKET	= 20,	// 发送包整包长度
} Vision_Data_Length_t;

/* 帧头格式 */
typedef __packed struct
{
	uint8_t  			sof;		// 同步头
	Vision_Cmd_Id_t  	cmd_id;		// 命令码
	uint8_t  			crc8;		// CRC8校验码
} Vision_Frame_Header_t;

/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16校验码
} Vision_Frame_Tailer_t;
/* 接收数据段格式 */
typedef __packed struct 
{
	float 	pitch_angle;	// pitch偏差角度/像素点	单位：角度/像素点
	float 	yaw_angle;		// yaw偏差角度/像素点	单位：角度/像素点
	float 	distance;			// 距离				单位：mm
	uint8_t identify_target;// 是否识别到目标	单位：0/1
	uint8_t anti_gyro;	// 是否识别到小陀螺	单位：0/1	
	uint8_t robot_number;	// 识别到的车辆数字
} Vision_Rx_Data_t;
/* 接收包格式 */
typedef __packed struct 
{
	Vision_Frame_Header_t FrameHeader;	// 帧头
	Vision_Rx_Data_t	  RxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾	
} Vision_Rx_Packet_t;

/* 发送数据段格式 */
typedef __packed struct
{
	uint8_t bullet_speed; //
	uint8_t attack_color;		// 颜色
	float 	yaw_measure;		// yaw电机角度
	float 	pitch_measure;	// pitch电机角度
	uint8_t anti_gyro;
	uint32_t time_stamp;   //时间戳，ms
} Vision_Tx_Data_t;
/* 发送包格式 */
typedef __packed struct
{
	Vision_Frame_Header_t FrameHeader;	// 帧头
	Vision_Tx_Data_t	  TxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾		
} Vision_Tx_Packet_t;
/**
 *	@brief	视觉模式
 */
typedef enum
{
	VISION_MODE_MANUAL		= 0,	// 手动模式
	VISION_MODE_AUTO		= 1,	// 自瞄模式
	VISION_MODE_BIG_BUFF	= 2,	// 打大符模式
	VISION_MODE_SMALL_BUFF	= 3,	// 打小符模式
} Vision_Mode_t;
/* 辅助标识变量 */
typedef struct
{
	uint8_t 		my_color;			// 用0/1表示颜色
	Vision_Mode_t	mode;				// 视觉模式
	uint8_t  		rx_data_valid;		// 接收数据的正确性
	uint16_t 		rx_err_cnt;			// 接收数据的错误统计
	uint32_t		rx_cnt;				// 接收数据包的统计
	bool		    rx_data_update;		// 接收数据是否更新
	uint32_t 		rx_time_prev;		// 接收数据的前一时刻
	uint32_t 		rx_time_now;		// 接收数据的当前时刻
	uint16_t 		rx_time_fps;		// 帧率
	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;	
} Vision_State_t;

/* 视觉通信数据包格式 */
typedef struct {
	Vision_Rx_Packet_t RxPacket;
	Vision_Tx_Packet_t TxPacket;
	Vision_State_t     State;
} vision_info_t;

typedef struct vision_sensor_struct {
	vision_info_t	*info;
	drv_uart_t			*driver;
	void				(*init)(struct vision_sensor_struct *self);
	void				(*update)(struct vision_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct vision_sensor_struct *self);	
	void				(*heart_beat)(struct vision_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
} vision_sensor_t;

extern vision_info_t    vision_sensor_info;
extern vision_sensor_t	vision_sensor;
extern drv_uart_t       vision_sensor_driver;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/



























#endif
