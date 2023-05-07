/**
 * @file        judge_sensor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        5-November-2020
 * @brief       Device Judge.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "judge_sensor.h"

#include "device.h"
#include "drv_uart.h"
#include "judge_potocol.h"

extern void judge_sensor_init(judge_sensor_t *judge_sen);
extern void judge_sensor_update(judge_sensor_t *judge_sen, uint8_t *rxBuf);
extern void judge_aerial_to_sentry(judge_sensor_t *judge_sen);
extern void judge_aerial_to_radar(judge_sensor_t *judge_sen);
extern void judge_config_line(graphic_data_struct_t *ui,
                              enum graphic_operate_type op_type,
                              enum graphic_layer layer,
                              enum graphic_color color,
                              uint16_t width,
                              uint16_t start_x,
                              uint16_t start_y,
                              uint16_t end_x,
                              uint16_t end_y);
extern void judge_config_rectangle(graphic_data_struct_t *ui,
                                   enum graphic_operate_type op_type,
                                   enum graphic_layer layer,
                                   enum graphic_color color,
                                   uint16_t width,
                                   uint16_t start_x,
                                   uint16_t start_y,
                                   uint16_t end_x,
                                   uint16_t end_y);
extern void judge_config_circle(graphic_data_struct_t *ui,
                                enum graphic_operate_type op_type,
                                enum graphic_layer layer,
                                enum graphic_color color,
                                uint16_t width,
                                uint16_t start_x,
                                uint16_t start_y,
                                uint16_t radius);
extern void judge_config_oval(graphic_data_struct_t *ui,
                              enum graphic_operate_type op_type,
                              enum graphic_layer layer,
                              enum graphic_color color,
                              uint16_t width,
                              uint16_t start_x,
                              uint16_t start_y,
                              uint16_t a,
                              uint16_t b);
extern void judge_config_arc(graphic_data_struct_t *ui,
                             enum graphic_operate_type op_type,
                             enum graphic_layer layer,
                             enum graphic_color color,
                             uint16_t start_angle,
                             uint16_t end_angle,
                             uint16_t width,
                             uint16_t start_x,
                             uint16_t start_y,
                             uint16_t a,
                             uint16_t b);
extern void judge_config_float(graphic_data_struct_t *ui,
                               enum graphic_operate_type op_type,
                               enum graphic_layer layer,
                               enum graphic_color color,
                               uint16_t font_size,
                               uint16_t dec_place,
                               uint16_t width,
                               uint16_t start_x,
                               uint16_t start_y,
                               float val);           
extern void judge_config_int(graphic_data_struct_t *ui,
                             enum graphic_operate_type op_type,
                             enum graphic_layer layer,
                             enum graphic_color color,
                             uint16_t font_size,
                             uint16_t width,
                             uint16_t start_x,
                             uint16_t start_y,
                             int32_t val);  
extern void judge_config_str(ext_client_custom_character_t *tx_data,
                             enum graphic_operate_type op_type,
                             enum graphic_layer layer,
                             enum graphic_color color,
                             uint16_t font_size,
                             uint16_t str_len,
                             uint16_t width,
                             uint16_t start_x,
                             uint16_t start_y,
                             const uint8_t *str);         
extern void judge_draw_1_fig(judge_sensor_t *judge_sen, ext_client_custom_graphic_single_t *tx_data);
extern void judge_draw_2_figs(struct judge_sensor_struct *judge_sen, 
                              ext_client_custom_graphic_single_t *tx_data_1,
                              ext_client_custom_graphic_single_t *tx_data_2);
extern void judge_draw_5_figs(struct judge_sensor_struct *judge_sen, 
                              ext_client_custom_graphic_single_t *tx_data_1, 
                              ext_client_custom_graphic_single_t *tx_data_2,
                              ext_client_custom_graphic_single_t *tx_data_3,   
                              ext_client_custom_graphic_single_t *tx_data_4,
                              ext_client_custom_graphic_single_t *tx_data_5);
extern void judge_draw_7_figs(struct judge_sensor_struct *judge_sen, 
                              ext_client_custom_graphic_single_t *tx_data_1, 
                              ext_client_custom_graphic_single_t *tx_data_2,
                              ext_client_custom_graphic_single_t *tx_data_3,   
                              ext_client_custom_graphic_single_t *tx_data_4,
                              ext_client_custom_graphic_single_t *tx_data_5,
                              ext_client_custom_graphic_single_t *tx_data_6,
                              ext_client_custom_graphic_single_t *tx_data_7);                             
extern void judge_draw_str(judge_sensor_t *judge_sen, ext_client_custom_character_t *tx_data);
extern void judge_modify_float(graphic_data_struct_t *ui, float val);
extern void judge_modify_int(graphic_data_struct_t *ui, int32_t val);
extern void judge_modify_str(ext_client_custom_character_t *tx_data, uint16_t str_len, const uint8_t *str);
                             
/* Private macro -------------------------------------------------------------*/
#define JUDGE_OFFLINE_MAX_CNT	100

/* Private function prototypes -----------------------------------------------*/
static void judge_sensor_check(judge_sensor_t *judge_sen);
static void judge_sensor_heart_beat(judge_sensor_t *judge_sen);
// Extended
static bool if_data_valid(judge_sensor_t *judge_sen);
static color_t get_my_color(judge_sensor_t *judge_sen);    
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 裁判系统信息
judge_info_t 	judge_info = {
	.offline_max_cnt = JUDGE_OFFLINE_MAX_CNT,
};

// 裁判系统驱动
drv_uart_t      judge_driver = {
    .type = DRV_UART4,
    .tx_byte = USART_TxByte,
    .tx_buff = USART_TxBuff,
    .tx_buff_dma = USART_TxBuff_DMA
};

// 裁判系统传感器
judge_sensor_t	judge_sensor = {
	.info = &judge_info,
    .driver = &judge_driver,
	.init = judge_sensor_init,
	.update = judge_sensor_update,
	.check = judge_sensor_check,
	.heart_beat = judge_sensor_heart_beat,
	.work_state = DEV_OFFLINE,
	.errno = NONE_ERR,
	.id = DEV_ID_JUDGE,
    .data_valid = if_data_valid,
    .my_color = get_my_color,
    .aerial_to_sentry = judge_aerial_to_sentry,
    .aerial_to_radar = judge_aerial_to_radar,
    .config_line = judge_config_line,
    .config_rectangle = judge_config_rectangle,
    .config_circle = judge_config_circle,
    .config_oval = judge_config_oval,
    .config_arc = judge_config_arc,
    .config_float = judge_config_float,
    .config_int = judge_config_int,
    .config_str = judge_config_str,
    .draw_1_fig = judge_draw_1_fig,
    .draw_2_figs = judge_draw_2_figs,
    .draw_5_figs = judge_draw_5_figs,
    .draw_7_figs = judge_draw_7_figs,
    .draw_str = judge_draw_str,
    .modify_float = judge_modify_float,
    .modify_int = judge_modify_int,
    .modify_str = judge_modify_str
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	裁判系统数据检查
 */
static void judge_sensor_check(judge_sensor_t *judge_sen)
{
	judge_info_t *judge_info = judge_sen->info;
	
	if(judge_info->shoot_update) {
	
	}
	
	if(judge_info->power_heat_update) {
	
	}
	
	if(judge_info->hurt_data_update) {
	
	}
	
}

/**
 *	@brief	裁判系统心跳包
 */
static void judge_sensor_heart_beat(judge_sensor_t *judge_sen)
{
	judge_info_t *judge_info = judge_sen->info;

	judge_info->offline_cnt++;
	if(judge_info->offline_cnt > judge_info->offline_max_cnt) {
		judge_info->offline_cnt = judge_info->offline_max_cnt;
		judge_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(judge_sen->work_state == DEV_OFFLINE)
			judge_sen->work_state = DEV_ONLINE;
	}
}

// Extended Functions
static bool if_data_valid(judge_sensor_t *judge_sen)
{
    return judge_sen->info->data_valid;
}

static color_t get_my_color(judge_sensor_t *judge_sen)
{
    if(judge_sen->info->game_robot_status.robot_id == ROBOT_ID_R6)
        return RED;
    else
        return BLUE;
}

/* Exported functions --------------------------------------------------------*/
