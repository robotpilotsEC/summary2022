/**
 * @file        vision_potocol.c
 * @author      Sentry@2021
 * @Version     V1.0
 * @date        19-February-2021
 * @brief       .
 */

/* Includes ------------------------------------------------------------------*/
#include  "vision_protocol.h"
#include  "vision_sensor.h"
#include "rp_math.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void vision_init(vision_sensor_t *vision)
{
    // ��ʼ��Ϊ����״̬
    vision->info->State.offline_cnt = vision->info->State.offline_max_cnt + 1;
    vision->work_state = DEV_OFFLINE;

    if(vision->id == DEV_ID_VISION)
        vision->errno = NONE_ERR;
    else
        vision->errno = DEV_ID_ERR;
}
/**
 *	@brief	�Ӿ����ݽ���Э��
 */
void vision_update(vision_sensor_t *vision_sen, uint8_t *rxBuf)
{
    vision_info_t *vision_info = vision_sen->info;

    uint8_t res = false;
    vision_info->State.rx_cnt++;
    /* ֡���ֽ��Ƿ�Ϊ0xA5 */
    if(rxBuf[SOF] == VISION_FRAME_HEADER)
    {
        res = Verify_CRC8_Check_Sum( rxBuf, LEN_FRAME_HEADER );
        /* ֡ͷCRC8У��*/
        if(res == true)
        {
            res = Verify_CRC16_Check_Sum( rxBuf, LEN_VISION_RX_PACKET );
            /* ֡βCRC16У�� */
            if(res == true)
            {
                /* ������ȷ�򿽱����հ� */
                memcpy(&vision_info->RxPacket, rxBuf, LEN_VISION_RX_PACKET);
                /* ֡�ʼ��� */
                vision_info->State.rx_time_now = HAL_GetTick();
                vision_info->State.rx_time_fps = vision_info->State.rx_time_now - vision_info->State.rx_time_prev;
                vision_info->State.rx_time_prev = vision_info->State.rx_time_now;
				
                vision_info->State.rx_data_update = true;	// �Ӿ����ݸ���				
                vision_info->State.offline_cnt=0;
            }
        }
    }
    /* ������Ч���ж� */
    if(res == true) {
        vision_info->State.rx_data_valid = true;
    } else if(res == false) {
        vision_info->State.rx_data_valid = false;
        vision_info->State.rx_err_cnt++;
    }
}

/**
 *	@brief	�ڴ���2�н���ң������Э��
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
    // �����Ӿ�����
    vision_sensor.update(&vision_sensor, rxBuf);
    vision_sensor.check(&vision_sensor);
}
