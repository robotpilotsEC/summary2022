#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
// 机器人间交互数据段长度
#define INTERACT_DATA_LEN	113
/* Exported types ------------------------------------------------------------*/
/* 帧头格式 */
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;


/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;     // CRC16校验码
} std_frame_tailer_t;


/* ID: 0x0001	Byte: 	11	比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;			// 比赛类型
	uint8_t game_progress : 4;		// 比赛阶段
	uint16_t stage_remain_time;		// 当前阶段剩余时间(单位:s)
    uint64_t SyncTimeStamp;         // 机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效
} ext_game_status_t; 


/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003	Byte:	32	机器人血量数据数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
	uint16_t red_2_robot_HP;	// 红2工程机器人血量
	uint16_t red_3_robot_HP;	// 红3步兵机器人血量
	uint16_t red_4_robot_HP;	// 红4步兵机器人血量
	uint16_t red_5_robot_HP;	// 红5步兵机器人血量
	uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
	uint16_t red_outpost_HP;	// 红方前哨站血量
	uint16_t red_base_HP;		// 红方基地血量
	uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
	uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
	uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
	uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
	uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
	uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
	uint16_t blue_outpost_HP;	// 蓝方前哨站血量
	uint16_t blue_base_HP;		// 蓝方基地血量	
} ext_game_robot_HP_t; 


///* ID: 0x0004 	Byte:	3	飞镖发射状态 */
//typedef __packed struct
//{
//	uint8_t dart_belong;
//	uint16_t stage_remaining_time;
//} ext_dart_status_t;


/* ID: 0x0005 	Byte:	11	人工智能挑战赛加成与惩罚区状态 */
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


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


///* ID: 0X0103  Byte:  3    请求补给站补弹子弹数据 */
//typedef __packed struct 
//{ 
//	uint8_t supply_projectile_id;
//	uint8_t supply_robot_id;
//	uint8_t supply_num;  
//} ext_supply_projectile_booking_t; 


/* ID: 0X0104  Byte:  2	   裁判警告信息 */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0105  Byte:  1	   飞镖发射口倒计时 */
typedef __packed struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 27    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;   					// 机器人ID，可用来校验发送
	uint8_t robot_level;  					// 1一级，2二级，3三级
	uint16_t remain_HP;  					// 机器人剩余血量
	uint16_t max_HP; 						// 机器人满血量
	uint16_t shooter_id1_17mm_cooling_rate;  	// 机器人 1号17mm 枪口每秒冷却值
	uint16_t shooter_id1_17mm_cooling_limit;    // 机器人 1号17mm 枪口热量上限
	uint16_t shooter_id1_17mm_speed_limit;	    // 机器人 1号17mm 枪口上限速度
	uint16_t shooter_id2_17mm_cooling_rate;  	// 机器人 2号17mm 枪口每秒冷却值
	uint16_t shooter_id2_17mm_cooling_limit;    // 机器人 2号17mm 枪口热量上限
	uint16_t shooter_id2_17mm_speed_limit;	    // 机器人 2号17mm 枪口上限速度
    uint16_t shooter_id1_42mm_cooling_rate;  	// 机器人 1号42mm 枪口每秒冷却值
	uint16_t shooter_id1_42mm_cooling_limit;    // 机器人 1号42mm 枪口热量上限
	uint16_t shooter_id1_42mm_speed_limit;	    // 机器人 1号42mm 枪口上限速度
    uint16_t max_chassis_power;				// 机器人最大底盘功率(W)
	uint8_t mains_power_gimbal_output : 1;  // gimbal口输出	
	uint8_t mains_power_chassis_output : 1; // chassis口输出
	uint8_t mains_power_shooter_output : 1; // shooter口输出
} ext_game_robot_status_t; 


/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   		// 底盘输出电压，单位：mV
	uint16_t chassis_current;		// 底盘输出电流，单位：mA
	float chassis_power;   			// 瞬时功率，单位：W
	uint16_t chassis_power_buffer;	// 底盘功率缓冲，单位：60J焦耳缓冲能量(飞坡根据规则增加至250J)
	uint16_t shooter_id1_17mm_cooling_heat;	// 1号17mm 枪口实时热量
	uint16_t shooter_id2_17mm_cooling_heat; // 2号17mm 枪口实时热量
	uint16_t shooter_id1_42mm_cooling_heat; // 1号42mm 枪口实时热量
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; 


/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t attack_time;    // 可攻击时间(s) 30s -> 0s
} ext_aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// 装甲伤害时代表装甲ID
	uint8_t hurt_type : 4; 	// 0x0装甲伤害 0x1模块掉线 0x2超射速 0x3超热量 0x4超功率 0x5撞击
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// 子弹类型(1-17mm, 2-42mm)
    uint8_t shooter_id;     // 发射机构ID
	uint8_t bullet_freq;  	// 子弹射频(Hz)
	float bullet_speed;		// 子弹射速(m/s)
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  6    子弹剩余发射数数据 */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
    uint16_t buller_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4 	机器人RFID状态 */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;


/* ID: 0x020A  Byte:  6 	飞镖机器人客户端指令 */
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/**
 *	-----------------------------------
 *	# 机器人间交互数据
 *	-----------------------------------
 */

/* 
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接收者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 共9个字节以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

    机器人 ID:
	1 英雄(红)；
	2 工程(红)；
	3/4/5 步兵(红)；
	6 空中(红)；
	7 哨兵(红)；
    9 雷达(红)；
    
	101 英雄(蓝)；
	102 工程(蓝)；
	103/104/105 步兵(蓝)；
	106 空中(蓝)；
	107 哨兵(蓝)；
    109 雷达(蓝)；
    
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105 步兵操作手客户端(红)；
	0x0106 空中操作手客户端((红)； 

	0x0165 英雄操作手客户端(蓝)；
	0x0166 工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169 步兵操作手客户端(蓝)；
	0x016A 空中操作手客户端(蓝)。 
*/
/* 机器人ID */
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

/* 客户端ID */
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

/* 交互数据接收信息：0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 		需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 		需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
/*
			最大上行每秒字节量	最大下行每秒字节量
	步兵		3720Bytes/s			3720Bytes/s
*/
typedef __packed struct 
{ 
	uint8_t data[INTERACT_DATA_LEN]; //数据段,n需要小于113
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
        stop_fire = 0x1,    // 停止开火
        back_scan = 0x2,    // 向后扫描
        escape = 0x4,       // 光速逃跑
        check_road = 0x8,   // 查看公路(对面飞坡过来)
        ctrl_sentry = 0x10,  // 接管哨兵
        test_sentry = 0x20  // 测试哨兵
    } cmd;
    int8_t ctrl_dir;        // 控制哨兵的方向
} com_aerial_to_sentry_t;

typedef __packed struct
{
    /* data header */
    uint16_t data_cmd_id;   // 0x206计时器启动、暂停，0x207计时器重置
	uint16_t send_ID;
	uint16_t receiver_ID;
} com_aerial_to_radar_t;

/**
 *	-----------------------------------
 *	# 机器人客户端UI
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
    GRAPHIC_INT = 6,    // BUG:规则手册有误
    GRAPHIC_FLOAT = 5,  // BUG:规则手册有误
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
	GRAPHIC_RED_BLUE  = 0,  // 红蓝主色	
	GRAPHIC_YELLOW    = 1,
	GRAPHIC_GREEN     = 2,
	GRAPHIC_ORANGE    = 3,
	GRAPHIC_FUCHSIA   = 4,	// 紫红色
	GRAPHIC_PINK      = 5,
	GRAPHIC_CYAN_BLUE = 6,	// 青色
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
	std_frame_header_t				rx_frame_header;			// 帧头信息
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
    ext_client_custom_graphic_single_t aerial_ui_int_yaw; // 自瞄yaw偏移
    ext_client_custom_graphic_single_t aerial_ui_int_pit; // 自瞄pit偏移
    ext_client_custom_graphic_single_t aerial_ui_int_30s;  // 30s倒计时
    ext_client_custom_graphic_single_t aerial_ui_line_1;    // 辅助瞄准线
    ext_client_custom_graphic_single_t aerial_ui_line_2;    // 辅助瞄准线
    /* other */
	bool	 	data_valid;	// 数据有效性
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
