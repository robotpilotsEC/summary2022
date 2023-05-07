/**
 * @file        vision_sensor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Device Vision.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision_sensor.h"

#include "drv_uart.h"
#include "vision_potocol.h"

extern void vision_sensor_init(vision_sensor_t *vision_sen);
extern void vision_sensor_update(vision_sensor_t *vision_sen, uint8_t *rxBuf);
extern void vision_sensor_tx(vision_sensor_t *vision_sen, vision_cmd_id_t cmd);
    
/* Private macro -------------------------------------------------------------*/
#define VISION_OFFLINE_MAX_CNT	100
// 1920*1080
#define VISION_MID_YAW  320
#define VISION_MID_PIT  240

/* Private function prototypes -----------------------------------------------*/
static void vision_sensor_check(vision_sensor_t *vision_sen);
static void vision_sensor_heart_beat(vision_sensor_t *vision_sen);
static void vision_sensor_set_color(vision_sensor_t *vision_sen, color_t color);
static bool vision_sensor_get_flag_status(vision_sensor_t *vision_sen, vision_flag_t flag);
static float vision_sensor_get_yaw_pixel_err(vision_sensor_t *vision_sen);
static float vision_sensor_get_pit_pixel_err(vision_sensor_t *vision_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 视觉信息
vision_sensor_info_t 	vision_sensor_info = {
	.offline_max_cnt = VISION_OFFLINE_MAX_CNT,
};

// 视觉驱动
drv_uart_t      vision_sensor_driver = {
    .type = DRV_UART1,
    .tx_byte = USART_TxByte,
    .tx_buff = USART_TxBuff,
    .tx_buff_dma = USART_TxBuff_DMA
};

// 视觉传感器
vision_sensor_t	vision_sensor = {
	.info = &vision_sensor_info,
    .driver = &vision_sensor_driver,
	.init = vision_sensor_init,
	.update = vision_sensor_update,
	.check = vision_sensor_check,
	.heart_beat = vision_sensor_heart_beat,
    .send_to_pc = vision_sensor_tx,
    .set_color = vision_sensor_set_color,
    .get_flag_status = vision_sensor_get_flag_status,
    .yaw_pixel_err = vision_sensor_get_yaw_pixel_err,
    .pit_pixel_err = vision_sensor_get_pit_pixel_err,
	.work_state = DEV_OFFLINE,
	.errno = NONE_ERR,
	.id = DEV_ID_VISION,
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	裁判系统数据检查
 */
static void vision_sensor_check(vision_sensor_t *vision_sen)
{
	vision_sensor_info_t *sen_info = vision_sen->info;
	
	if(sen_info->rx_state.rx_data_valid == false)
		sen_info->rx_state.rx_err_cnt++;
}

/**
 *	@brief	裁判系统心跳包
 */
static void vision_sensor_heart_beat(vision_sensor_t *vision_sen)
{
	vision_sensor_info_t *sen_info = vision_sen->info;

	sen_info->offline_cnt++;
	if(sen_info->offline_cnt > sen_info->offline_max_cnt) {
		sen_info->offline_cnt = sen_info->offline_max_cnt;
		vision_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(vision_sen->work_state == DEV_OFFLINE)
			vision_sen->work_state = DEV_ONLINE;
	}
}

static void vision_sensor_set_color(vision_sensor_t *vision_sen, color_t color)
{
    vision_sen->info->tx_packet.tx_data.my_color = color;
}

static bool vision_sensor_get_flag_status(vision_sensor_t *vision_sen, vision_flag_t flag)
{
    bool res = false;
    
    switch(flag)
    {
        case VIS_IF_DATA_VALID:
            res = vision_sen->info->rx_state.rx_data_valid;
            break;
        
        case VIS_IF_DATA_UPDATE:
            res = vision_sen->info->rx_state.rx_data_update;
            vision_sen->info->rx_state.rx_data_update = false;  // 读取之后自动清零
            break;
        
        case VIS_IF_LOCK_TARGET:
            res = vision_sen->info->rx_packet.rx_data.lock_target;
            break;
        
        default:
            break;
    }
    
    return res;
}

static float vision_sensor_get_yaw_pixel_err(vision_sensor_t *vision_sen)
{
//    return vision_sen->info->rx_packet.rx_data.yaw_err - VISION_MID_YAW;
}

static float vision_sensor_get_pit_pixel_err(vision_sensor_t *vision_sen)
{
//    return vision_sen->info->rx_packet.rx_data.pit_err - VISION_MID_PIT;
}

/* Exported functions --------------------------------------------------------*/
