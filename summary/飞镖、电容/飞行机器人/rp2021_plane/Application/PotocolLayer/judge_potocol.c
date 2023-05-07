/**
 * @file        judge_potocol.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        4-November-2020
 * @brief       Referee Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "judge_potocol.h"

#include "judge_sensor.h"
#include "crc.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
#define JUDGE_FRAME_HEADER	0xA5

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* ֡ͷ�ֽ�ƫ�� */
enum judge_frame_offset_t {
	JUDGE_SOF		= 0,
	JUDGE_DATA_LEN	= 1,
	JUDGE_SEQ		= 3,
	JUDGE_CRC8		= 4,
    JUDGE_CMD_ID    = 5,
    JUDGE_DATA      = 7
};

enum judge_cmd_id_t {
	ID_GAME_STATUS 					= 0x0001,	// ����״̬			
	ID_GAME_RESULT 					= 0x0002,	// ������� 		
	ID_GAME_ROBOT_HP 				= 0x0003,	// ������Ѫ������  	
//	ID_DART_STATUS					= 0x0004,	// ���ڷ���״̬
	ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005,	// �˹�������ս���ӳ���ͷ���״̬
	
	ID_EVENT_DATA 					= 0x0101,	// �����¼�����		
	ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// ����վ������ʶ	
	//ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// ���󲹸�վ��������
	ID_REFEREE_WARNING 				= 0x0104,	// ���о�����Ϣ
	ID_DART_REMAINING_TIME			= 0x0105,	// ���ڷ���ڵ���ʱ
	
	ID_GAME_ROBOT_STATUS 			= 0x0201,	// ����������״̬
	ID_POWER_HEAT_DATA 				= 0x0202,	// ʵʱ������������
	ID_GAME_ROBOT_POS				= 0x0203,	// ������λ��
	ID_BUFF							= 0x0204,	// ����������
	ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// ���л���������״̬
	ID_ROBOT_HURT					= 0x0206,	// �������˺�״̬
	ID_SHOOT_DATA					= 0x0207,	// ʵʱ�����Ϣ
	ID_BULLET_REMAINING				= 0x0208,	// �ӵ�ʣ�෢����
	ID_RFID_STATUS					= 0x0209,	// ������RFID״̬
	ID_DART_CLIENT_CMD              = 0x020A,   // ���ڻ����˿ͻ���ָ��
    
	ID_COMMUNICATION				= 0x0301,	// �����˼佻������(���ͷ���������)
    ID_CUSTOMED_CTRL                = 0x0302,   // �Զ���������������ݽӿ�
    ID_GIMBALER_COMMAND             = 0x0303,   // �ͻ���С��ͼ��������
    ID_ROBOT_COMMAND                = 0x0304,   // ���̡������Ϣ
    ID_CLIENT_MAP_COMMAND           = 0x0305    // �ͻ���С��ͼ������Ϣ
};

enum judge_data_length_t {
	/* Std */
	LEN_FRAME_HEAD 	= 5,	// ֡ͷ����
	LEN_CMD_ID 		= 2,	// �����볤��
	LEN_FRAME_TAIL 	= 2,	// ֡βCRC16
    LEN_DATA_HEADER = sizeof(com_aerial_client_ui_t)
//	/* Ext */
//	// 0x000x
//	LEN_GAME_STATUS 				= 11,
//	LEN_GAME_RESULT 				= 1,
//	LEN_GAME_ROBOT_HP 				= 32,
////	LEN_DART_STATUS					= 3,
//	LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS= 11,
//	
//	// 0x010x
//	LEN_EVENT_DATA					= 4,
//	LEN_SUPPLY_PROJECTILE_ACTION	= 4,
//	//LEN_SUPPLY_PROJECTILE_BOOKING	= 3,
//	LEN_REFEREE_WARNING				= 2,
//	LEN_DART_REMAINING_TIME			= 1,
//	
//	// 0x020x
//	LEN_GAME_ROBOT_STATUS			= 27,
//	LEN_POWER_HEAT_DATA 			= 16,
//	LEN_GAME_ROBOT_POS				= 16,
//	LEN_BUFF		 				= 1,
//	LEN_AERIAL_ROBOT_ENERGY 		= 1,
//	LEN_ROBOT_HURT					= 1,
//	LEN_SHOOT_DATA					= 7,
//	LEN_BULLET_REMAINING	 		= 6,
//	LEN_RFID_STATUS					= 4,
//    LEN_DART_CLIENT_CMD             = 6, 
};

/* Private variables ---------------------------------------------------------*/
// 0x000x
uint8_t LEN_GAME_STATUS;
uint8_t LEN_GAME_RESULT;
uint8_t LEN_GAME_ROBOT_HP;
uint8_t LEN_DART_STATUS;
uint8_t LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS;	
// 0x010x
uint8_t LEN_EVENT_DATA;
uint8_t LEN_SUPPLY_PROJECTILE_ACTION;
uint8_t LEN_SUPPLY_PROJECTILE_BOOKING;
uint8_t LEN_REFEREE_WARNING;
uint8_t LEN_DART_REMAINING_TIME;
// 0x020x
uint8_t LEN_GAME_ROBOT_STATUS;
uint8_t LEN_POWER_HEAT_DATA;
uint8_t LEN_GAME_ROBOT_POS;
uint8_t LEN_BUFF;
uint8_t LEN_AERIAL_ROBOT_ENERGY;
uint8_t LEN_ROBOT_HURT;
uint8_t LEN_SHOOT_DATA;
uint8_t LEN_BULLET_REMAINING;
uint8_t LEN_RFID_STATUS;
uint8_t LEN_DART_CLIENT_CMD;

#define JUDGE_TX_BUFFER_LEN 128
uint8_t Judge_Tx_Buffer[JUDGE_TX_BUFFER_LEN];

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void judge_sensor_init(judge_sensor_t *judge_sen)
{
	judge_sen->info->offline_cnt = judge_sen->info->offline_max_cnt+1;
	judge_sen->work_state= DEV_OFFLINE;
	
	if(judge_sen->id == DEV_ID_JUDGE) {
		judge_sen->errno = NONE_ERR;
	}
	else {
		judge_sen->errno = DEV_ID_ERR;
	}
    
    LEN_GAME_STATUS = sizeof(ext_game_status_t);
    LEN_GAME_RESULT = sizeof(ext_game_result_t);
    LEN_GAME_ROBOT_HP = sizeof(ext_game_robot_HP_t);
    LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = sizeof(ext_ICRA_buff_debuff_zone_status_t);
    LEN_EVENT_DATA = sizeof(ext_event_data_t);
    LEN_SUPPLY_PROJECTILE_ACTION = sizeof(ext_supply_projectile_action_t);
    LEN_REFEREE_WARNING = sizeof(ext_referee_warning_t);
    LEN_DART_REMAINING_TIME = sizeof(ext_dart_remaining_time_t);
    LEN_GAME_ROBOT_STATUS = sizeof(ext_game_robot_status_t);
    LEN_POWER_HEAT_DATA = sizeof(ext_power_heat_data_t);
    LEN_GAME_ROBOT_POS = sizeof(ext_game_robot_pos_t);
    LEN_BUFF = sizeof(ext_buff_t);
    LEN_AERIAL_ROBOT_ENERGY = sizeof(ext_aerial_robot_energy_t);
    LEN_ROBOT_HURT = sizeof(ext_robot_hurt_t);
    LEN_SHOOT_DATA = sizeof(ext_shoot_data_t);
    LEN_BULLET_REMAINING = sizeof(ext_bullet_remaining_t);
    LEN_RFID_STATUS = sizeof(ext_rfid_status_t);
    LEN_DART_CLIENT_CMD = sizeof(ext_dart_client_cmd_t);
}

void judge_sensor_update(judge_sensor_t *judge_sen, uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;	
	judge_info_t *judge_info = judge_sen->info;
	
	if( rxBuf == NULL )
	{
		judge_info->data_valid = false;
		return;
	}
	
	memcpy(&judge_info->rx_frame_header, rxBuf, LEN_FRAME_HEAD);
	
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[JUDGE_SOF] == JUDGE_FRAME_HEADER) 
	{
		/* ֡ͷCRC8У�� */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* ͳ��һ֡�������ݳ��ȣ�����CRC16У�� */
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->rx_frame_header.data_length + LEN_FRAME_TAIL;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[JUDGE_CMD_ID+1] << 8 | rxBuf[JUDGE_CMD_ID]);
				
				switch(cmd_id)
				{
					case ID_GAME_STATUS: {
							memcpy(&judge_info->game_status, (rxBuf+JUDGE_DATA), LEN_GAME_STATUS);
						}break;
					
					case ID_GAME_RESULT: {
							memcpy(&judge_info->game_result, (rxBuf+JUDGE_DATA), LEN_GAME_RESULT);
						}break;
					
					case ID_GAME_ROBOT_HP: {
							memcpy(&judge_info->game_robot_HP, (rxBuf+JUDGE_DATA), LEN_GAME_ROBOT_HP);
						}break;
					
					case ID_EVENT_DATA: {
							memcpy(&judge_info->event_data, (rxBuf+JUDGE_DATA), LEN_EVENT_DATA);
						}break;
					
					case ID_SUPPLY_PROJECTILE_ACTION: {
							memcpy(&judge_info->supply_projectile_action, (rxBuf+JUDGE_DATA), LEN_SUPPLY_PROJECTILE_ACTION);
							judge_info->supply_data_update = true;	// ����վ���ݸ���
						}break;
					
					case ID_REFEREE_WARNING: {
							memcpy(&judge_info->referee_warning, (rxBuf+JUDGE_DATA), LEN_REFEREE_WARNING);
						}break;
					
					case ID_DART_REMAINING_TIME: {
							memcpy(&judge_info->dart_remaining_time, (rxBuf+JUDGE_DATA), LEN_DART_REMAINING_TIME);
						}break;
						
					case ID_GAME_ROBOT_STATUS: {
							memcpy(&judge_info->game_robot_status, (rxBuf+JUDGE_DATA), LEN_GAME_ROBOT_STATUS);
						}break;
					
					case ID_POWER_HEAT_DATA: {
							memcpy(&judge_info->power_heat_data, (rxBuf+JUDGE_DATA), LEN_POWER_HEAT_DATA);
							judge_info->power_heat_update = true;
						}break;
					
					case ID_GAME_ROBOT_POS: {
							memcpy(&judge_info->game_robot_pos, (rxBuf+JUDGE_DATA), LEN_GAME_ROBOT_POS);
						}break;
					
					case ID_BUFF: {
							memcpy(&judge_info->buff, (rxBuf+JUDGE_DATA), LEN_BUFF);
						}break;
					
					case ID_AERIAL_ROBOT_ENERGY: {
							memcpy(&judge_info->aerial_robot_energy, (rxBuf+JUDGE_DATA), LEN_AERIAL_ROBOT_ENERGY);
						}break;
					
					case ID_ROBOT_HURT: {
							memcpy(&judge_info->robot_hurt, (rxBuf+JUDGE_DATA), LEN_ROBOT_HURT);
							judge_info->hurt_data_update = true;	// �˺����ݸ���
						}break;
					
					case ID_SHOOT_DATA: {
							memcpy(&judge_info->shoot_data, (rxBuf+JUDGE_DATA), LEN_SHOOT_DATA);
							judge_info->shoot_update = true;
							//TODO:���㷢����
						}break;
					
					case ID_BULLET_REMAINING: {
							memcpy(&judge_info->bullet_remaining, (rxBuf+JUDGE_DATA), LEN_BULLET_REMAINING);
						}break;
					
					case ID_RFID_STATUS: {
							memcpy(&judge_info->rfid_status, (rxBuf+JUDGE_DATA), LEN_RFID_STATUS);
						}break;
						
                    case ID_DART_CLIENT_CMD: {
                            memcpy(&judge_info->dart_client_cmd, (rxBuf+JUDGE_DATA), LEN_DART_CLIENT_CMD);
                        }break;
                        
					case ID_COMMUNICATION: {
							//JUDGE_ReadFromCom();
						}break;
				}
			}
		}

		/* ֡βCRC16��һ�ֽ��Ƿ�Ϊ0xA5 */
		if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
		{
			/* ���һ�����ݰ������˶�֡���ݾ��ٴζ�ȡ */
			judge_sensor_update( judge_sen, &rxBuf[frame_length] );
		}
	}
	
	judge_info->data_valid = res;
	if(judge_info->data_valid != true)
		judge_info->err_cnt++;
	
	// ���յ����ݱ�ʾ����
	judge_info->offline_cnt = 0;
}

void judge_aerial_to_sentry(judge_sensor_t *judge_sen)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = sizeof(com_aerial_to_sentry_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + sizeof(judge_info->aerial_to_sentry) + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + sizeof(judge_info->aerial_to_sentry);

    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
    judge_info->aerial_to_sentry.data_cmd_id = 0x0201;
    judge_info->aerial_to_sentry.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_sentry.receiver_ID = ROBOT_ID_R7;
    else
        judge_info->aerial_to_sentry.receiver_ID = ROBOT_ID_B7;
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_sentry, tx_data_length);
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);
}

void judge_aerial_to_radar(judge_sensor_t *judge_sen)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = sizeof(com_aerial_to_radar_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + sizeof(judge_info->aerial_to_radar) + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + sizeof(judge_info->aerial_to_radar);

    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
//    judge_info->aerial_to_radar.data_cmd_id = 0x0206 or 0x0207;
    judge_info->aerial_to_radar.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_radar.receiver_ID = ROBOT_ID_R9;
    else
        judge_info->aerial_to_radar.receiver_ID = ROBOT_ID_B9;
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_radar, tx_data_length);
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);
}

/**
 *  @berif
 *      config ui as graphic_line
 */
void judge_config_line(graphic_data_struct_t *ui,
                       enum graphic_operate_type op_type,
                       enum graphic_layer layer,
                       enum graphic_color color,
                       uint16_t width,
                       uint16_t start_x,
                       uint16_t start_y,
                       uint16_t end_x,
                       uint16_t end_y)
{
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_LINE;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = 0;    // any value
    ui->end_angle = 0;      // any value
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = 0;         // any value
    ui->end_x = end_x;      
    ui->end_y = end_y;      
}

/**
 *  @berif
 *      config ui as graphic_rectangle
 */
void judge_config_rectangle(graphic_data_struct_t *ui,
                            enum graphic_operate_type op_type,
                            enum graphic_layer layer,
                            enum graphic_color color,
                            uint16_t width,
                            uint16_t start_x,
                            uint16_t start_y,
                            uint16_t end_x,
                            uint16_t end_y)
{
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_RECTANGLE;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = 0;    // any value
    ui->end_angle = 0;      // any value
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = 0;         // any value
    ui->end_x = end_x;      
    ui->end_y = end_y;      
}

/**
 *  @berif
 *      config ui as graphic_circle
 */
void judge_config_circle(graphic_data_struct_t *ui,
                         enum graphic_operate_type op_type,
                         enum graphic_layer layer,
                         enum graphic_color color,
                         uint16_t width,
                         uint16_t start_x,
                         uint16_t start_y,
                         uint16_t radius)
{
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_CIRCLE;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = 0;    // any value
    ui->end_angle = 0;      // any value
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = radius;
    ui->end_x = 0;          // any value
    ui->end_y = 0;          // any value
}

/**
 *  @berif
 *      config ui as graphic_oval
 */
void judge_config_oval(graphic_data_struct_t *ui,
                       enum graphic_operate_type op_type,
                       enum graphic_layer layer,
                       enum graphic_color color,
                       uint16_t width,
                       uint16_t start_x,
                       uint16_t start_y,
                       uint16_t a,
                       uint16_t b)
{
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_OVAL;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = 0;    // any value
    ui->end_angle = 0;      // any value
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = 0;         // any value
    ui->end_x = a;          // len of x 
    ui->end_y = b;          // len of y
}

/**
 *  @berif
 *      config ui as graphic_arc
 */
void judge_config_arc(graphic_data_struct_t *ui,
                      enum graphic_operate_type op_type,
                      enum graphic_layer layer,
                      enum graphic_color color,
                      uint16_t start_angle,
                      uint16_t end_angle,
                      uint16_t width,
                      uint16_t start_x,
                      uint16_t start_y,
                      uint16_t a,
                      uint16_t b)
{
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_ARC;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = start_angle; 
    ui->end_angle = end_angle;      
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = 0;         // any value
    ui->end_x = a;          // len of x 
    ui->end_y = b;          // len of y
}

/**
 *  @berif
 *      config ui as graphic_float
 */
void judge_config_float(graphic_data_struct_t *ui,
                        enum graphic_operate_type op_type,
                        enum graphic_layer layer,
                        enum graphic_color color,
                        uint16_t font_size,
                        uint16_t dec_place,
                        uint16_t width,
                        uint16_t start_x,
                        uint16_t start_y,
                        float val)
{
    int32_t int_val = (int32_t)val * 1000;// float * 1000 �� int32_t���δ���
    
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_FLOAT;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = font_size; 
    ui->end_angle = dec_place;      
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
//    ui->radius = (int_val >> 21) & 0x3ff;   // 10bits
//    ui->end_x = (int_val >> 11) & 0x7ff;    // 11bits
//    ui->end_y = int_val & 0x7ff;            // 11bits    
    ui->radius = int_val & 0x3ff;           // 10bits
    ui->end_x = (int_val >> 11) & 0x7ff;    // 11bits
    ui->end_y = (int_val >> 21) & 0x7ff;    // 11bits    
}

/**
 *  @berif
 *      config ui as graphic_int
 */
void judge_config_int(graphic_data_struct_t *ui,
                      enum graphic_operate_type op_type,
                      enum graphic_layer layer,
                      enum graphic_color color,
                      uint16_t font_size,
                      uint16_t width,
                      uint16_t start_x,
                      uint16_t start_y,
                      int32_t val)
{
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_INT;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = font_size; 
    ui->end_angle = 0;      // any value
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = val & 0x3ff;           // 10bits
    ui->end_x = (val >> 11) & 0x7ff;    // 11bits
    ui->end_y = (val >> 21) & 0x7ff;    // 11bits     
}

/**
 *  @berif
 *      config ui as graphic_char
 */
void judge_config_str(ext_client_custom_character_t *tx_data,
                      enum graphic_operate_type op_type,
                      enum graphic_layer layer,
                      enum graphic_color color,
                      uint16_t font_size,
                      uint16_t str_len,
                      uint16_t width,
                      uint16_t start_x,
                      uint16_t start_y,
                      const uint8_t *str)
{
    graphic_data_struct_t *ui = &tx_data->grapic_data_struct;
    /* ͼ������ */
    ui->operate_type = op_type;
    ui->graphic_type = GRAPHIC_CHAR;
    ui->layer = layer;
    ui->color = color;
    ui->start_angle = font_size; 
    ui->end_angle = str_len;
    ui->width = width;
    ui->start_x = start_x;
    ui->start_y = start_y;
    ui->radius = 0;   // any value
    ui->end_x = 0;    // any value
    ui->end_y = 0;    // any value
    memcpy(tx_data->data, str, str_len);
}

/**
 *  @berif
 *      draw 1 figure
 */
static const uint8_t single_graphic_len = sizeof(ext_client_custom_graphic_single_t);
void judge_draw_1_fig(judge_sensor_t *judge_sen, ext_client_custom_graphic_single_t *tx_data)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = LEN_DATA_HEADER + sizeof(ext_client_custom_graphic_single_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + tx_data_length + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + tx_data_length;

    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
    judge_info->aerial_to_client.data_cmd_id = GRAPHIC_DRAW_1_FIG;
    judge_info->aerial_to_client.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_R6;
    else
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_B6;
    // tx_data header
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_client, LEN_DATA_HEADER);
    // tx_data seg
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER], tx_data, single_graphic_len);
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);        
}

/**
 *  @berif
 *      draw 2 figures
 */
void judge_draw_2_figs(judge_sensor_t *judge_sen, 
                       ext_client_custom_graphic_single_t *tx_data_1,
                       ext_client_custom_graphic_single_t *tx_data_2)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = LEN_DATA_HEADER + sizeof(ext_client_custom_graphic_double_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + tx_data_length + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + tx_data_length;

    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
    judge_info->aerial_to_client.data_cmd_id = GRAPHIC_DRAW_2_FIGS;
    judge_info->aerial_to_client.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_R6;
    else
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_B6;
    // tx_data header
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_client, LEN_DATA_HEADER);
    // tx_data seg
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER], tx_data_1, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER + single_graphic_len], tx_data_2, single_graphic_len);
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);        
}

/**
 *  @berif
 *      draw 5 figures
 */
void judge_draw_5_figs(judge_sensor_t *judge_sen, 
                       ext_client_custom_graphic_single_t *tx_data_1, 
                       ext_client_custom_graphic_single_t *tx_data_2,
                       ext_client_custom_graphic_single_t *tx_data_3,   
                       ext_client_custom_graphic_single_t *tx_data_4,
                       ext_client_custom_graphic_single_t *tx_data_5)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = LEN_DATA_HEADER + sizeof(ext_client_custom_graphic_five_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + tx_data_length + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + tx_data_length;

    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
    judge_info->aerial_to_client.data_cmd_id = GRAPHIC_DRAW_5_FIGS;
    judge_info->aerial_to_client.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_R6;
    else
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_B6;
    // tx_data header
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_client, LEN_DATA_HEADER);
    // tx_data seg
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER], tx_data_1, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len], tx_data_2, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*2], tx_data_3, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*3], tx_data_4, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*4], tx_data_5, single_graphic_len);
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);        
}

/**
 *  @berif
 *      draw 7 figures
 */
void judge_draw_7_figs(judge_sensor_t *judge_sen, 
                       ext_client_custom_graphic_single_t *tx_data_1, 
                       ext_client_custom_graphic_single_t *tx_data_2,
                       ext_client_custom_graphic_single_t *tx_data_3,   
                       ext_client_custom_graphic_single_t *tx_data_4,
                       ext_client_custom_graphic_single_t *tx_data_5,
                       ext_client_custom_graphic_single_t *tx_data_6,
                       ext_client_custom_graphic_single_t *tx_data_7)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = LEN_DATA_HEADER + sizeof(ext_client_custom_graphic_seven_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + tx_data_length + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + tx_data_length;
    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
    judge_info->aerial_to_client.data_cmd_id = GRAPHIC_DRAW_5_FIGS;
    judge_info->aerial_to_client.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_R6;
    else
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_B6;
    /* data_header */
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_client, LEN_DATA_HEADER);
    /* data_seg */
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER], tx_data_1, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len], tx_data_2, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*2], tx_data_3, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*3], tx_data_4, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*4], tx_data_5, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*5], tx_data_6, single_graphic_len);
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER+single_graphic_len*6], tx_data_7, single_graphic_len);
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);        
}

/**
 *  @berif
 *      draw 1 string
 */
void judge_draw_str(judge_sensor_t *judge_sen, ext_client_custom_character_t *tx_data)
{
    drv_uart_t *judge_drv = judge_sen->driver;
    judge_info_t *judge_info = judge_sen->info;
    static const uint8_t tx_data_length = LEN_DATA_HEADER + sizeof(ext_client_custom_character_t);
    static const uint8_t tx_packet_length = JUDGE_DATA + tx_data_length + 2;
    static const uint8_t tx_crc16_offset = JUDGE_DATA + tx_data_length;

    /* ֡ͷ��ʽ */
    judge_info->tx_packet.frame_header.sof = 0xA5;
    judge_info->tx_packet.frame_header.seq = 0;
    judge_info->tx_packet.frame_header.data_length = tx_data_length;
    /* д��֡ͷ */
    memcpy(Judge_Tx_Buffer, &judge_info->tx_packet.frame_header, sizeof(std_frame_header_t));
    /* д��֡ͷCRC8 */
    Append_CRC8_Check_Sum(Judge_Tx_Buffer, sizeof(std_frame_header_t));
    judge_info->tx_packet.frame_header.crc8 = Judge_Tx_Buffer[JUDGE_CRC8];
    /* ������ */
    judge_info->tx_packet.cmd_id = ID_COMMUNICATION;
    /* д�������� */
    memcpy(&Judge_Tx_Buffer[JUDGE_CMD_ID], (uint8_t*)&judge_info->tx_packet.cmd_id, 2);
    /* ���ݶ���� */
    judge_info->tx_packet.tx_data = &Judge_Tx_Buffer[JUDGE_DATA];
    judge_info->aerial_to_client.data_cmd_id = GRAPHIC_DRAW_CHAR;
    judge_info->aerial_to_client.send_ID = judge_info->game_robot_status.robot_id;
    if(judge_sen->my_color(judge_sen) == RED)
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_R6;
    else
        judge_info->aerial_to_client.receiver_ID = CLIENT_ID_B6;
    /* data_header */
    memcpy(judge_info->tx_packet.tx_data, &judge_info->aerial_to_client, LEN_DATA_HEADER);
    /* data_seg */
    memcpy(&judge_info->tx_packet.tx_data[LEN_DATA_HEADER], tx_data, sizeof(ext_client_custom_character_t));
    /* д��֡βCRC16 */
    Append_CRC16_Check_Sum(Judge_Tx_Buffer, tx_packet_length);
	judge_info->tx_packet.frame_tailer.crc16 = ((uint16_t)Judge_Tx_Buffer[tx_crc16_offset] << 8) + (Judge_Tx_Buffer[tx_crc16_offset]);
    /* ����DMA���� */
    judge_drv->tx_buff(judge_drv, Judge_Tx_Buffer, tx_packet_length);
    /* �������ݰ����� */
	memset(Judge_Tx_Buffer, 0, JUDGE_TX_BUFFER_LEN);
}

/**
 *  @berif
 *      modify the value of float
 */
void judge_modify_float(graphic_data_struct_t *ui, float val)
{
    int32_t int_val = (int32_t)val * 1000;// float * 1000 �� int32_t���δ���
    /* ͼ������ */
    ui->operate_type = GRAPHIC_MODIFY;
    ui->radius = int_val & 0x3ff;           // 10bits
    ui->end_x = (int_val >> 11) & 0x7ff;    // 11bits
    ui->end_y = (int_val >> 21) & 0x7ff;    // 11bits    
}

/**
 *  @berif
 *      modify the value of int
 */
void judge_modify_int(graphic_data_struct_t *ui, int32_t val)
{
    /* ͼ������ */
    ui->operate_type = GRAPHIC_MODIFY;
    ui->radius = val & 0x3ff;           // 10bits
    ui->end_x = (val >> 11) & 0x7ff;    // 11bits
    ui->end_y = (val >> 21) & 0x7ff;    // 11bits  
}

/**
 *  @berif
 *      modify the vlaue of string
 */
void judge_modify_str(ext_client_custom_character_t *tx_data, uint16_t str_len, const uint8_t *str)
{
    graphic_data_struct_t *ui = &tx_data->grapic_data_struct;
    /* ͼ������ */
    ui->operate_type = GRAPHIC_MODIFY;
    ui->end_angle = str_len;
    memcpy(tx_data->data, str, str_len);
}

/**
 *	@brief	�ڴ���3�н�������ϵͳ����Э��
 */
void USART4_rxDataHandler(uint8_t *rxBuf)
{	
	judge_sensor.update(&judge_sensor, rxBuf);
	judge_sensor.check(&judge_sensor);
}
