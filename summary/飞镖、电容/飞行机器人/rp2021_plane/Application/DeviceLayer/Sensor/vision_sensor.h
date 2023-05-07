#ifndef __VISION_SENSOR_H
#define __VISION_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 *	@brief	�Ӿ���־λ
 */
typedef enum
{
	VIS_IF_DATA_VALID  	= 0, // �Ӿ�������Ч���ж�
	VIS_IF_DATA_UPDATE 	= 1, // �Ӿ����ݸ��±�־λ
	VIS_IF_LOCK_TARGET 	= 2, // ʶ�𵽵з�װ�װ�
} vision_flag_t;

/**
 *	���->�Ӿ�
 */
typedef enum {
	CMD_AIM_OFF 	= 0x00,	// ����������(ʵ�����Ӿ��Ǳ��ܳ��򵫲���������)
	CMD_AIM_ARMY	= 0x01,	// ʶ��װ�װ�
    CMD_AIM_OUTPOST	= 0x02,	// ʶ��ǰ��վ�̵�
} vision_cmd_id_t;

/* ֡ͷ��ʽ */
typedef __packed struct
{
	uint8_t  			sof;		// ͬ��ͷ
	vision_cmd_id_t  	cmd_id;		// ������
	uint8_t  			crc8;		// CRC8У����
} vision_frame_header_t;

/* ֡β��ʽ */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16У����
} vision_frame_tailer_t;

/* �������ݶθ�ʽ */
typedef __packed struct 
{
    float   pitch;        // pitch����Ƕ�
    float   yaw;        // yaw����Ƕ�
    uint8_t lock_target;    // �Ƿ�ʶ��Ŀ��	��λ��0/1
//    float   distance;       // ����				��λ��mm
//    uint8_t lose_target;    // �Ƿ��֡		    ��λ��0/1
//    uint8_t res0;           // ����0
//    uint8_t res1;           // ����1
//    uint8_t res2;           // ����2
//    uint8_t res3;           // ����3
} vision_rx_data_t;



/* �������ݶθ�ʽ */
typedef __packed struct
{
    float yaw;
    float pitch;
	uint8_t my_color;		// ���Լ�����ɫ
    //float test2;
} vision_tx_data_t;

/* ���հ���ʽ */
typedef __packed struct 
{
	vision_frame_header_t frame_header;	// ֡ͷ
	vision_rx_data_t	  rx_data;		// ����
	vision_frame_tailer_t frame_tailer;	// ֡β	
} vision_rx_packet_t;

/* ���Ͱ���ʽ */
typedef __packed struct
{
	vision_frame_header_t frame_header;	// ֡ͷ
	vision_tx_data_t	  tx_data;		// ����
	vision_frame_tailer_t frame_tailer;	// ֡β		
} vision_tx_packet_t;

/* ���հ����ݳ��� */
typedef struct
{
    uint8_t             rx_data_length;
    uint8_t             rx_packet_length;
} vision_rx_len_t;

/* ���Ͱ����ݳ��� */
typedef struct
{
    uint8_t             tx_data_length;
    uint8_t             tx_packet_length;
    uint8_t             tx_crc16_offset;
} vision_tx_len_t;

/* ����״̬��Ϣ */
typedef struct
{
	uint8_t  			rx_data_valid;		// �������ݵ���ȷ��
	uint16_t 			rx_err_cnt;			// �������ݵĴ���ͳ��
	uint32_t			rx_cnt;				// �������ݰ���ͳ��
	uint8_t 			rx_data_update;		// ���������Ƿ����
	uint32_t 			rx_time_prev;		// �������ݵ�ǰһʱ��
	uint32_t 			rx_time_now;		// �������ݵĵ�ǰʱ��
	uint16_t 			rx_time_ping;		// ֡��    
} vision_rx_state_t;

/* �Ӿ�ͨ�����ݰ���ʽ */
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
