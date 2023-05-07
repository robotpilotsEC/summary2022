#ifndef __VISION_SENSOR_H
#define __VISION_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 *	@brief	视觉标志位
 */
typedef enum
{
	VIS_IF_DATA_VALID  	= 0, // 视觉数据有效性判断
	VIS_IF_DATA_UPDATE 	= 1, // 视觉数据更新标志位
	VIS_IF_LOCK_TARGET 	= 2, // 识别到敌方装甲板
} vision_flag_t;

/**
 *	电控->视觉
 */
typedef enum {
	CMD_AIM_OFF 	= 0x00,	// 不启动自瞄(实际是视觉那边跑程序但不发送数据)
	CMD_AIM_ARMY	= 0x01,	// 识别装甲板
    CMD_AIM_OUTPOST	= 0x02,	// 识别前哨站绿灯
} vision_cmd_id_t;

/* 帧头格式 */
typedef __packed struct
{
	uint8_t  			sof;		// 同步头
	vision_cmd_id_t  	cmd_id;		// 命令码
	uint8_t  			crc8;		// CRC8校验码
} vision_frame_header_t;

/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16校验码
} vision_frame_tailer_t;

/* 接收数据段格式 */
typedef __packed struct 
{
    float   pitch;        // pitch电机角度
    float   yaw;        // yaw电机角度
    uint8_t lock_target;    // 是否识别到目标	单位：0/1
//    float   distance;       // 距离				单位：mm
//    uint8_t lose_target;    // 是否掉帧		    单位：0/1
//    uint8_t res0;           // 保留0
//    uint8_t res1;           // 保留1
//    uint8_t res2;           // 保留2
//    uint8_t res3;           // 保留3
} vision_rx_data_t;



/* 发送数据段格式 */
typedef __packed struct
{
    float yaw;
    float pitch;
	uint8_t my_color;		// 我自己的颜色
    //float test2;
} vision_tx_data_t;

/* 接收包格式 */
typedef __packed struct 
{
	vision_frame_header_t frame_header;	// 帧头
	vision_rx_data_t	  rx_data;		// 数据
	vision_frame_tailer_t frame_tailer;	// 帧尾	
} vision_rx_packet_t;

/* 发送包格式 */
typedef __packed struct
{
	vision_frame_header_t frame_header;	// 帧头
	vision_tx_data_t	  tx_data;		// 数据
	vision_frame_tailer_t frame_tailer;	// 帧尾		
} vision_tx_packet_t;

/* 接收包数据长度 */
typedef struct
{
    uint8_t             rx_data_length;
    uint8_t             rx_packet_length;
} vision_rx_len_t;

/* 发送包数据长度 */
typedef struct
{
    uint8_t             tx_data_length;
    uint8_t             tx_packet_length;
    uint8_t             tx_crc16_offset;
} vision_tx_len_t;

/* 接收状态信息 */
typedef struct
{
	uint8_t  			rx_data_valid;		// 接收数据的正确性
	uint16_t 			rx_err_cnt;			// 接收数据的错误统计
	uint32_t			rx_cnt;				// 接收数据包的统计
	uint8_t 			rx_data_update;		// 接收数据是否更新
	uint32_t 			rx_time_prev;		// 接收数据的前一时刻
	uint32_t 			rx_time_now;		// 接收数据的当前时刻
	uint16_t 			rx_time_ping;		// 帧率    
} vision_rx_state_t;

/* 视觉通信数据包格式 */
typedef struct {
	vision_rx_packet_t 	rx_packet;
    vision_rx_len_t     rx_len;
	vision_tx_packet_t 	tx_packet;
    vision_tx_len_t     tx_len;
	vision_rx_state_t   rx_state;
    
	uint16_t			offline_cnt;
	uint16_t			offline_max_cnt;	
} vision_sensor_info_t;

typedef struct vision_sensor_struct {
	vision_sensor_info_t    *info;
	drv_uart_t			    *driver;
	void				    (*init)(struct vision_sensor_struct *self);
	void				    (*update)(struct vision_sensor_struct *self, uint8_t *rxBuf);
	void				    (*check)(struct vision_sensor_struct *self);	
	void				    (*heart_beat)(struct vision_sensor_struct *self);
    void                    (*send_to_pc)(struct vision_sensor_struct *self, vision_cmd_id_t cmd);
    void                    (*set_color)(struct vision_sensor_struct *self, color_t color);
    bool                    (*get_flag_status)(struct vision_sensor_struct *self, vision_flag_t);
    float                   (*yaw_pixel_err)(struct vision_sensor_struct *self);
    float                   (*pit_pixel_err)(struct vision_sensor_struct *self);
	dev_work_state_t	    work_state;
	dev_errno_t			    errno;
	dev_id_t			    id;	
} vision_sensor_t;

extern vision_sensor_info_t	vision_sensor_info;
extern vision_sensor_t 	    vision_sensor;
/* Exported functions --------------------------------------------------------*/

#endif
