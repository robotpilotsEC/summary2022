/**
 * @file        vision_potocol.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Vision Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision_potocol.h"

#include "vision_sensor.h"
#include "crc.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
#define VISION_FRAME_HEADER	0xA5

/* Private function prototypes -----------------------------------------------*/
/* ���ݳ��� */
enum vision_frame_length_t {
	/* Std */
	VISION_LEN_FRAME_HEADER = 3,	// ֡ͷ����
	VISION_LEN_FRAME_TAILER = 2,	// ֡βCRC16
};

/* ֡ͷ�ֽ�ƫ�� */
enum vision_frame_offset_t {
	VISION_SOF		= 0,
	VISION_CMD_ID	= 1,
	VISION_CRC8		= 2,
	VISION_DATA		= 3,
};

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define VISION_TX_BUFFER_LEN    50
uint8_t Vision_Tx_Buffer[VISION_TX_BUFFER_LEN];

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void vision_sensor_init(vision_sensor_t *vision_sen)
{
    vision_sensor_info_t *sen_info = vision_sen->info;
    
	sen_info->offline_cnt = sen_info->offline_max_cnt+1;
	vision_sen->work_state = DEV_OFFLINE;
	
	if(vision_sen->id == DEV_ID_VISION) {
		vision_sen->errno = NONE_ERR;
	}
	else {
		vision_sen->errno = DEV_ID_ERR;
	}
    
    sen_info->rx_len.rx_data_length = sizeof(vision_rx_data_t);
    sen_info->rx_len.rx_packet_length = sizeof(vision_rx_packet_t);
    
    sen_info->tx_len.tx_data_length = sizeof(vision_tx_data_t);
    sen_info->tx_len.tx_packet_length = sizeof(vision_tx_packet_t);
    sen_info->tx_len.tx_crc16_offset = sen_info->tx_len.tx_packet_length - VISION_LEN_FRAME_TAILER;
}

void vision_sensor_update(vision_sensor_t *vision_sen, uint8_t *rxBuf)
{
	uint8_t ret = false;
	vision_sensor_info_t *sen_info = vision_sen->info;
	sen_info->rx_state.rx_cnt++;
	
	if(rxBuf[VISION_SOF] == VISION_FRAME_HEADER)
	{
		ret = Verify_CRC8_Check_Sum(rxBuf, VISION_LEN_FRAME_HEADER);
		/* ֡ͷCRC8У�� */
		if(ret == true)
		{
			ret = Verify_CRC16_Check_Sum(rxBuf, sen_info->rx_len.rx_packet_length);
			/* ֡βCRC16У�� */
			if(ret == true)
			{
				// ������ȷ�򿽱����հ�
				memcpy(&sen_info->rx_packet, rxBuf, sen_info->rx_len.rx_packet_length);
				sen_info->rx_state.rx_data_update = true; // �Ӿ����ݸ���
				
				// ����֡��
				sen_info->rx_state.rx_time_now = HAL_GetTick();
				sen_info->rx_state.rx_time_ping = sen_info->rx_state.rx_time_now - sen_info->rx_state.rx_time_prev;
				sen_info->rx_state.rx_time_prev = sen_info->rx_state.rx_time_now;
			}
		}
	}

	sen_info->rx_state.rx_data_valid = ret;
	
	// ���յ����ݱ�ʾ����
	sen_info->offline_cnt = 0;
}

void vision_sensor_tx(vision_sensor_t *vision_sen, vision_cmd_id_t cmd)
{
    drv_uart_t *vision_drv = vision_sen->driver;
    vision_sensor_info_t *sen_info = vision_sen->info;
    
    /* ֡ͷ��ʽ */
    sen_info->tx_packet.frame_header.sof = VISION_FRAME_HEADER;
    sen_info->tx_packet.frame_header.cmd_id = cmd;
    /* д��֡ͷ */
    memcpy( Vision_Tx_Buffer, &sen_info->tx_packet.frame_header, VISION_LEN_FRAME_HEADER );
    /* д��֡ͷCRC8 */
	Append_CRC8_Check_Sum( Vision_Tx_Buffer, VISION_LEN_FRAME_HEADER);
	sen_info->tx_packet.frame_header.crc8 = Vision_Tx_Buffer[VISION_CRC8];
    /* ���ݶ���� */
	memcpy( &Vision_Tx_Buffer[VISION_DATA], &sen_info->tx_packet.tx_data, sen_info->tx_len.tx_data_length );
	/* д��֡βCRC16 */
	Append_CRC16_Check_Sum( Vision_Tx_Buffer, sen_info->tx_len.tx_packet_length);
	sen_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Vision_Tx_Buffer[sen_info->tx_len.tx_crc16_offset] << 8) + (Vision_Tx_Buffer[sen_info->tx_len.tx_crc16_offset]);
    /* ����DMA���� */
    vision_drv->tx_buff(vision_drv, Vision_Tx_Buffer, sen_info->tx_len.tx_packet_length);
    /* �������ݰ����� */
	memset(Vision_Tx_Buffer, 0, VISION_TX_BUFFER_LEN);
}

/**
 *	@brief	�ڴ���1�н����Ӿ�����Э��
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
}
