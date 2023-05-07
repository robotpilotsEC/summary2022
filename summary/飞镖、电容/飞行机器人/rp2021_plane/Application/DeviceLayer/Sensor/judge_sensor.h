#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
// �����˼佻�����ݶγ���
#define INTERACT_DATA_LEN	113
/* Exported types ------------------------------------------------------------*/
/* ֡ͷ��ʽ */
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;


/* ֡β��ʽ */
typedef __packed struct 
{
	uint16_t crc16;     // CRC16У����
} std_frame_tailer_t;


/* ID: 0x0001	Byte: 	11	����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;			// ��������
	uint8_t game_progress : 4;		// �����׶�
	uint16_t stage_remain_time;		// ��ǰ�׶�ʣ��ʱ��(��λ:s)
    uint64_t SyncTimeStamp;         // �����˽��յ���ָ��ľ�ȷUnixʱ�䣬�����ض��յ���Ч��NTP��������ʱ����Ч
} ext_game_status_t; 


/* ID: 0x0002	Byte:	1	����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003	Byte:	32	������Ѫ���������� */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��(δ�ϳ�������Ѫ��Ϊ0)
	uint16_t red_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t red_3_robot_HP;	// ��3����������Ѫ��
	uint16_t red_4_robot_HP;	// ��4����������Ѫ��
	uint16_t red_5_robot_HP;	// ��5����������Ѫ��
	uint16_t red_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t red_outpost_HP;	// �췽ǰ��վѪ��
	uint16_t red_base_HP;		// �췽����Ѫ��
	uint16_t blue_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��
	uint16_t blue_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t blue_3_robot_HP;	// ��3����������Ѫ��
	uint16_t blue_4_robot_HP;	// ��4����������Ѫ��
	uint16_t blue_5_robot_HP;	// ��5����������Ѫ��
	uint16_t blue_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t blue_outpost_HP;	// ����ǰ��վѪ��
	uint16_t blue_base_HP;		// ��������Ѫ��	
} ext_game_robot_HP_t; 


///* ID: 0x0004 	Byte:	3	���ڷ���״̬ */
//typedef __packed struct
//{
//	uint8_t dart_belong;
//	uint16_t stage_remaining_time;
//} ext_dart_status_t;


/* ID: 0x0005 	Byte:	11	�˹�������ս���ӳ���ͷ���״̬ */
typedef __packed struct
{
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3;
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
    
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    ����վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


///* ID: 0X0103  Byte:  3    ���󲹸�վ�����ӵ����� */
//typedef __packed struct 
//{ 
//	uint8_t supply_projectile_id;
//	uint8_t supply_robot_id;
//	uint8_t supply_num;  
//} ext_supply_projectile_booking_t; 


/* ID: 0X0104  Byte:  2	   ���о�����Ϣ */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0105  Byte:  1	   ���ڷ���ڵ���ʱ */
typedef __packed struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 27    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id;   					// ������ID��������У�鷢��
	uint8_t robot_level;  					// 1һ����2������3����
	uint16_t remain_HP;  					// ������ʣ��Ѫ��
	uint16_t max_HP; 						// ��������Ѫ��
	uint16_t shooter_id1_17mm_cooling_rate;  	// ������ 1��17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit;    // ������ 1��17mm ǹ����������
	uint16_t shooter_id1_17mm_speed_limit;	    // ������ 1��17mm ǹ�������ٶ�
	uint16_t shooter_id2_17mm_cooling_rate;  	// ������ 2��17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id2_17mm_cooling_limit;    // ������ 2��17mm ǹ����������
	uint16_t shooter_id2_17mm_speed_limit;	    // ������ 2��17mm ǹ�������ٶ�
    uint16_t shooter_id1_42mm_cooling_rate;  	// ������ 1��42mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_42mm_cooling_limit;    // ������ 1��42mm ǹ����������
	uint16_t shooter_id1_42mm_speed_limit;	    // ������ 1��42mm ǹ�������ٶ�
    uint16_t max_chassis_power;				// �����������̹���(W)
	uint8_t mains_power_gimbal_output : 1;  // gimbal�����	
	uint8_t mains_power_chassis_output : 1; // chassis�����
	uint8_t mains_power_shooter_output : 1; // shooter�����
} ext_game_robot_status_t; 


/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   		// ���������ѹ����λ��mV
	uint16_t chassis_current;		// ���������������λ��mA
	float chassis_power;   			// ˲ʱ���ʣ���λ��W
	uint16_t chassis_power_buffer;	// ���̹��ʻ��壬��λ��60J������������(���¸��ݹ���������250J)
	uint16_t shooter_id1_17mm_cooling_heat;	// 1��17mm ǹ��ʵʱ����
	uint16_t shooter_id2_17mm_cooling_heat; // 2��17mm ǹ��ʵʱ����
	uint16_t shooter_id1_42mm_cooling_heat; // 1��42mm ǹ��ʵʱ����
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; 


/* ID: 0x0205  Byte:  1    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t attack_time;    // �ɹ���ʱ��(s) 30s -> 0s
} ext_aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// װ���˺�ʱ����װ��ID
	uint8_t hurt_type : 4; 	// 0x0װ���˺� 0x1ģ����� 0x2������ 0x3������ 0x4������ 0x5ײ��
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// �ӵ�����(1-17mm, 2-42mm)
    uint8_t shooter_id;     // �������ID
	uint8_t bullet_freq;  	// �ӵ���Ƶ(Hz)
	float bullet_speed;		// �ӵ�����(m/s)
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  6    �ӵ�ʣ�෢�������� */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
    uint16_t buller_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;


/* ID: 0x020A  Byte:  6 	���ڻ����˿ͻ���ָ�� */
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/**
 *	-----------------------------------
 *	# �����˼佻������
 *	-----------------------------------
 */

/* 
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail ��9���ֽ��Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

    ������ ID:
	1 Ӣ��(��)��
	2 ����(��)��
	3/4/5 ����(��)��
	6 ����(��)��
	7 �ڱ�(��)��
    9 �״�(��)��
    
	101 Ӣ��(��)��
	102 ����(��)��
	103/104/105 ����(��)��
	106 ����(��)��
	107 �ڱ�(��)��
    109 �״�(��)��
    
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 ���̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105 ���������ֿͻ���(��)��
	0x0106 ���в����ֿͻ���((��)�� 

	0x0165 Ӣ�۲����ֿͻ���(��)��
	0x0166 ���̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169 ���������ֿͻ���(��)��
	0x016A ���в����ֿͻ���(��)�� 
*/
/* ������ID */
enum ext_robot_id {
    ROBOT_ID_R1 = 0x1,
    ROBOT_ID_R2 = 0x2,
    ROBOT_ID_R3 = 0x3,
    ROBOT_ID_R4 = 0x4,
    ROBOT_ID_R5 = 0x5,
    ROBOT_ID_R6 = 0x6,
    ROBOT_ID_R7 = 0x7,
    ROBOT_ID_R9 = 0x9,
    ROBOT_ID_B1 = 0x65,
    ROBOT_ID_B2 = 0x66,
    ROBOT_ID_B3 = 0x67,
    ROBOT_ID_B4 = 0x68,
    ROBOT_ID_B5 = 0x69,
    ROBOT_ID_B6 = 0x6A,
    ROBOT_ID_B7 = 0x6B,
    ROBOT_ID_B9 = 0x6D
};

/* �ͻ���ID */
enum ext_client_id {
    CLIENT_ID_R1 = 0x101,
    CLIENT_ID_R2 = 0x102,
    CLIENT_ID_R3 = 0x103,
    CLIENT_ID_R4 = 0x104,
    CLIENT_ID_R5 = 0x105,
    CLIENT_ID_R6 = 0x106,
    CLIENT_ID_B1 = 0x165,
    CLIENT_ID_B2 = 0x166,
    CLIENT_ID_B3 = 0x167,
    CLIENT_ID_B4 = 0x168,
    CLIENT_ID_B5 = 0x169,
    CLIENT_ID_B6 = 0x16A
};

/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 		��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 		��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
/*
			�������ÿ���ֽ���	�������ÿ���ֽ���
	����		3720Bytes/s			3720Bytes/s
*/
typedef __packed struct 
{ 
	uint8_t data[INTERACT_DATA_LEN]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;


typedef __packed struct
{
    std_frame_header_t              frame_header;
    uint16_t                        cmd_id;
    uint8_t*                        tx_data;
    std_frame_tailer_t              frame_tailer;
} com_tx_packet_t;


typedef __packed struct
{
    /* data header */
    uint16_t data_cmd_id;   // 0x201
	uint16_t send_ID;
	uint16_t receiver_ID;
    /* data seg */
    enum cmd_t {
        stop_fire = 0x1,    // ֹͣ����
        back_scan = 0x2,    // ���ɨ��
        escape = 0x4,       // ��������
        check_road = 0x8,   // �鿴��·(������¹���)
        ctrl_sentry = 0x10,  // �ӹ��ڱ�
        test_sentry = 0x20  // �����ڱ�
    } cmd;
    int8_t ctrl_dir;        // �����ڱ��ķ���
} com_aerial_to_sentry_t;

typedef __packed struct
{
    /* data header */
    uint16_t data_cmd_id;   // 0x206��ʱ����������ͣ��0x207��ʱ������
	uint16_t send_ID;
	uint16_t receiver_ID;
} com_aerial_to_radar_t;

/**
 *	-----------------------------------
 *	# �����˿ͻ���UI
 *	-----------------------------------
 */
enum graphic_cmd_id
{
    GRAPHIC_DELETE_FIG = 0x100,
    GRAPHIC_DRAW_1_FIG = 0x101,
    GRAPHIC_DRAW_2_FIGS = 0x102,
    GRAPHIC_DRAW_5_FIGS = 0x103,
    GRAPHIC_DRAW_7_FIGS = 0x104,
    GRAPHIC_DRAW_CHAR = 0x110
};


enum graphic_operate_type
{
    GRAPHIC_NULL = 0,
    GRAPHIC_ADD = 1,
    GRAPHIC_MODIFY = 2,
    GRAPHIC_DELETE = 3
};


enum graphic_figure_type
{
    GRAPHIC_LINE = 0,
    GRAPHIC_RECTANGLE = 1,
    GRAPHIC_CIRCLE = 2,
    GRAPHIC_OVAL = 3,
    GRAPHIC_ARC = 4,
    GRAPHIC_INT = 6,    // BUG:�����ֲ�����
    GRAPHIC_FLOAT = 5,  // BUG:�����ֲ�����
    GRAPHIC_CHAR = 7
};


enum graphic_layer
{
    GRAPHIC_LAYER_0 = 0,
    GRAPHIC_LAYER_1 = 1,
    GRAPHIC_LAYER_2 = 2,
    GRAPHIC_LAYER_3 = 3,
    GRAPHIC_LAYER_4 = 4,
    GRAPHIC_LAYER_5 = 5,
    GRAPHIC_LAYER_6 = 6,
    GRAPHIC_LAYER_7 = 7,
    GRAPHIC_LAYER_8 = 8,
    GRAPHIC_LAYER_9 = 9,
};


enum graphic_color
{
	GRAPHIC_RED_BLUE  = 0,  // ������ɫ	
	GRAPHIC_YELLOW    = 1,
	GRAPHIC_GREEN     = 2,
	GRAPHIC_ORANGE    = 3,
	GRAPHIC_FUCHSIA   = 4,	// �Ϻ�ɫ
	GRAPHIC_PINK      = 5,
	GRAPHIC_CYAN_BLUE = 6,	// ��ɫ
	GRAPHIC_BLACK     = 7,
	GRAPHIC_WHITE     = 8    
};


typedef __packed struct
{
    uint8_t operate_type; 
    uint8_t layer; 
} ext_client_custom_graphic_delete_t;


typedef __packed struct
{ 
    uint8_t graphic_name[3]; 
    uint32_t operate_type:3; 
    uint32_t graphic_type:3; 
    uint32_t layer:4; 
    uint32_t color:4; 
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10; 
    uint32_t start_x:11; 
    uint32_t start_y:11; 
    uint32_t radius:10; 
    uint32_t end_x:11; 
    uint32_t end_y:11; 
} graphic_data_struct_t;


typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;


typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;


typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;


typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;


typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

typedef __packed struct
{
    /* data header */
    uint16_t data_cmd_id;   // 0x201
	uint16_t send_ID;
	uint16_t receiver_ID;
    /* data seg */
} com_aerial_client_ui_t;

typedef struct judge_info_struct {
    /* rx */
	std_frame_header_t				rx_frame_header;			// ֡ͷ��Ϣ
	ext_game_status_t 				game_status;				// 0x0001
	ext_game_result_t 				game_result;				// 0x0002
	ext_game_robot_HP_t 			game_robot_HP;				// 0x0003
//	ext_dart_status_t				dart_status;				// 0x0004
	//ext_ICRA_buff_debuff_zone_status_t	
	
	ext_event_data_t				event_data;					// 0x0101
	ext_supply_projectile_action_t	supply_projectile_action;	// 0x0102
	//ext_supply_projectile_booking_t supply_projectile_booking;// 0x0103
	ext_referee_warning_t			referee_warning;			// 0x0104
	ext_dart_remaining_time_t		dart_remaining_time;		// 0x0105
	
	ext_game_robot_status_t			game_robot_status;			// 0x0201
	ext_power_heat_data_t			power_heat_data;			// 0x0202
	ext_game_robot_pos_t			game_robot_pos;				// 0x0203
	ext_buff_t						buff;						// 0x0204
	ext_aerial_robot_energy_t		aerial_robot_energy;		// 0x0205
	ext_robot_hurt_t				robot_hurt;					// 0x0206
	ext_shoot_data_t				shoot_data;					// 0x0207
	ext_bullet_remaining_t			bullet_remaining;			// 0x0208	
	ext_rfid_status_t				rfid_status;				// 0x0209	
	ext_dart_client_cmd_t           dart_client_cmd;            // 0x020A
    /* tx */
    com_tx_packet_t                 tx_packet;
    com_aerial_to_sentry_t          aerial_to_sentry;
    com_aerial_to_radar_t           aerial_to_radar;
    com_aerial_client_ui_t          aerial_to_client;
    ext_client_custom_character_t   aerial_ui_str_yaw;   // "yaw:"
    ext_client_custom_character_t   aerial_ui_str_pit;   // "pit:"
    ext_client_custom_graphic_single_t aerial_ui_int_yaw; // ����yawƫ��
    ext_client_custom_graphic_single_t aerial_ui_int_pit; // ����pitƫ��
    ext_client_custom_graphic_single_t aerial_ui_int_30s;  // 30s����ʱ
    ext_client_custom_graphic_single_t aerial_ui_line_1;    // ������׼��
    ext_client_custom_graphic_single_t aerial_ui_line_2;    // ������׼��
    /* other */
	bool	 	data_valid;	// ������Ч��
	bool		err_cnt;
	bool		power_heat_update;
	bool		shoot_update;
	bool		hurt_data_update;
	bool 		dart_data_update;
	bool		supply_data_update;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} judge_info_t;

typedef struct judge_sensor_struct {
	judge_info_t		*info;
	drv_uart_t			*driver;
	void				(*init)(struct judge_sensor_struct *self);
	void				(*update)(struct judge_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct judge_sensor_struct *self);	
	void				(*heart_beat)(struct judge_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
    bool                (*data_valid)(struct judge_sensor_struct *self);
    color_t             (*my_color)(struct judge_sensor_struct *self);
    void                (*aerial_to_sentry)(struct judge_sensor_struct *self);
    void                (*aerial_to_radar)(struct judge_sensor_struct *self);
    void                (*config_line)(graphic_data_struct_t *ui,
                                       enum graphic_operate_type op_type,
                                       enum graphic_layer layer,
                                       enum graphic_color color,
                                       uint16_t width,
                                       uint16_t start_x,
                                       uint16_t start_y,
                                       uint16_t end_x,
                                       uint16_t end_y);
    void                (*config_rectangle)(graphic_data_struct_t *ui,
                                            enum graphic_operate_type op_type,
                                            enum graphic_layer layer,
                                            enum graphic_color color,
                                            uint16_t width,
                                            uint16_t start_x,
                                            uint16_t start_y,
                                            uint16_t end_x,
                                            uint16_t end_y);
    void                (*config_circle)(graphic_data_struct_t *ui,
                                         enum graphic_operate_type op_type,
                                         enum graphic_layer layer,
                                         enum graphic_color color,
                                         uint16_t width,
                                         uint16_t start_x,
                                         uint16_t start_y,
                                         uint16_t radius);
    void                (*config_oval)(graphic_data_struct_t *ui,
                                       enum graphic_operate_type op_type,
                                       enum graphic_layer layer,
                                       enum graphic_color color,
                                       uint16_t width,
                                       uint16_t start_x,
                                       uint16_t start_y,
                                       uint16_t a,
                                       uint16_t b);
    void                (*config_arc)(graphic_data_struct_t *ui,
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
    void                (*config_float)(graphic_data_struct_t *ui,
                                        enum graphic_operate_type op_type,
                                        enum graphic_layer layer,
                                        enum graphic_color color,
                                        uint16_t font_size,
                                        uint16_t dec_place,
                                        uint16_t width,
                                        uint16_t start_x,
                                        uint16_t start_y,
                                        float val);
    void                (*config_int)(graphic_data_struct_t *ui,
                                      enum graphic_operate_type op_type,
                                      enum graphic_layer layer,
                                      enum graphic_color color,
                                      uint16_t font_size,
                                      uint16_t width,
                                      uint16_t start_x,
                                      uint16_t start_y,
                                      int32_t val);
    void                (*config_str)(ext_client_custom_character_t *tx_data,
                                      enum graphic_operate_type op_type,
                                      enum graphic_layer layer,
                                      enum graphic_color color,
                                      uint16_t font_size,
                                      uint16_t str_len,
                                      uint16_t width,
                                      uint16_t start_x,
                                      uint16_t start_y,
                                      const uint8_t *str);
    void                (*draw_1_fig)(struct judge_sensor_struct *judge_sen, 
                                      ext_client_custom_graphic_single_t *tx_data);
    void                (*draw_2_figs)(struct judge_sensor_struct *judge_sen, 
                                       ext_client_custom_graphic_single_t *tx_data_1,
                                       ext_client_custom_graphic_single_t *tx_data_2);
    void                (*draw_5_figs)(struct judge_sensor_struct *judge_sen, 
                                       ext_client_custom_graphic_single_t *tx_data_1,
                                       ext_client_custom_graphic_single_t *tx_data_2,
                                       ext_client_custom_graphic_single_t *tx_data_3,
                                       ext_client_custom_graphic_single_t *tx_data_4,
                                       ext_client_custom_graphic_single_t *tx_data_5
                                       );
    void                (*draw_7_figs)(struct judge_sensor_struct *judge_sen, 
                                       ext_client_custom_graphic_single_t *tx_data_1,
                                       ext_client_custom_graphic_single_t *tx_data_2,
                                       ext_client_custom_graphic_single_t *tx_data_3,
                                       ext_client_custom_graphic_single_t *tx_data_4,
                                       ext_client_custom_graphic_single_t *tx_data_5,
                                       ext_client_custom_graphic_single_t *tx_data_6,
                                       ext_client_custom_graphic_single_t *tx_data_7                                       
                                       );
    void                (*draw_str)(struct judge_sensor_struct *judge_sen, 
                                    ext_client_custom_character_t *tx_data);
    void                (*modify_float)(graphic_data_struct_t *ui, float val);
    void                (*modify_int)(graphic_data_struct_t *ui, int32_t val);
    void                (*modify_str)(ext_client_custom_character_t *tx_data, uint16_t str_len, const uint8_t *str);
} judge_sensor_t;

extern judge_info_t judge_info;
extern judge_sensor_t judge_sensor;

/* Exported functions --------------------------------------------------------*/

#endif
