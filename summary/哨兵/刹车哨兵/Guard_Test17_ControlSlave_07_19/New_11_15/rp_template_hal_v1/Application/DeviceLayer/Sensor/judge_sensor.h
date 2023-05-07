#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H
/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "drv_uart.h"
/* Exported macro ------------------------------------------------------------*/
// �����˼佻�����ݶγ���
#define INTERACT_DATA_LEN	10

/* Exported types ------------------------------------------------------------*/
/* �Զ���֡ͷ Byte: 5 */
typedef __packed struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} std_frame_header_t;


/* ID: 0x0001	Byte: 	11	����״̬���� */
typedef __packed struct
{
    uint8_t game_type : 4;			// ��������
    uint8_t game_progress : 4;		// �����׶�
    uint16_t stage_remain_time;		// ��ǰ�׶�ʣ��ʱ��(��λ:s)
    uint64_t SyncTimeStamp;       //�����˽��յ���ָ��ľ�ȷ Unix ʱ�䣬�����ض��յ���Ч�� NTP ��������ʱ����Ч
} ext_game_status_t;


/* ID: 0x0002	Byte:	1	����������� */
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;


/* ID: 0x0003	Byte:	36	������Ѫ���������� */
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


/* ID: 0x0004 	Byte:	3	���ڷ���״̬ */
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;


/* ID: 0x0005 	Byte:	3	�˹�������ս���ӳ���ͷ���״̬ */
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
    uint32_t others_1 : 10;
    uint32_t outpost  : 1;
    uint32_t others_2 : 21;
//	uint32_t event_type;
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


/* ID: 0X0201  Byte: 18    ������״̬���� */
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;


/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct
{
    uint16_t chassis_volt;   		// ���������ѹ����λ��mV
    uint16_t chassis_current;		// ���������������λ��mA
    float chassis_power;   			// ˲ʱ���ʣ���λ��W
    uint16_t chassis_power_buffer;	// ���̹��ʻ���
    uint16_t  shooter_id1_17mm_cooling_heat;		// 17mm ǹ������
    uint16_t shooter_id2_17mm_cooling_heat;	// ����17mm ǹ������
    uint16_t shooter_id1_42mm_cooling_heat;	//  42mm ǹ������
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


/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct
{
    uint8_t attack_time;
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
    uint8_t shooter_id;
    uint8_t bullet_freq;  	// �ӵ���Ƶ(Hz)
    float bullet_speed;		// �ӵ�����(m/s)
} ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    �ӵ�ʣ�෢�������� */
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* ID: 0x301  Byte:  12 	��̨�ֽ��� */
typedef __packed struct
{
    uint16_t cmd_id;
    uint16_t send_id;
    uint16_t receive_id;
	
	enum cmd_t{
		stop_fire = 0x1,      //ֹͣ���𣨲��̣�
		back_scan = 0x2,     //���ɨ��
		escape = 0x4,         //������·
		check_road = 0x8,      //��·�鿴
		control_aerial = 0x10,   //�ӹ��ڱ�
		test_sentry = 0x20,       //�����ڱ�
	}cmd;
	int8_t control_dir; //-1��1����������
} ext_aerial_data_t;

/* ID: 0x0303  Byte:  15 	 */
typedef __packed struct
{
    float target_x;
    float target_y;
    float target_z;
    uint8_t commd_keyboard;
    uint16_t target_ID;
} ext_robot_command_t;

typedef struct
{
    uint16_t frame_length;	// ��֡����(����ʱʹ��)
    uint16_t cmd_id;		// ������(����ʱʹ��)
    uint16_t err_cnt;		// ��֡��(����ʱʹ��)
    bool	 data_valid;	// ������Ч��
    uint16_t self_client_id;// �����߻����˶�Ӧ�Ŀͻ���ID
    bool	power_heat_update;	// �����������ݸ���
    bool	shoot_update;		// ������ݸ���
    bool	 hurt_data_update;	// �˺����ݸ���
    bool	dart_data_update;	// �������ݸ���
    bool	supply_data_update;	// ����վ���ݸ���
	bool    command_data_update;//С��ͼ���ݸ���
	bool	communication_data_update;//��̨�����ݸ���
    std_frame_header_t				FrameHeader;				// ֡ͷ��Ϣ
    ext_game_status_t 				GameStatus;					// 0x0001
    ext_game_result_t 				GameResult;					// 0x0002
    ext_game_robot_HP_t 			GameRobotHP;		 		// 0x0003
    ext_dart_status_t				DartStatus;					// 0x0004
    //ext_ICRA_buff_debuff_zone_status_t

    ext_event_data_t				EventData;					// 0x0101
    ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
    //ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
    ext_referee_warning_t			RefereeWarning;				// 0x0104
    ext_dart_remaining_time_t		DartRemainingTime;			// 0x0105

    ext_game_robot_status_t			GameRobotStatus;			// 0x0201
    ext_power_heat_data_t			PowerHeatData;				// 0x0202
    ext_game_robot_pos_t			GameRobotPos;				// 0x0203
    ext_buff_t						Buff;						// 0x0204
    ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
    ext_robot_hurt_t				RobotHurt;					// 0x0206
    ext_shoot_data_t				ShootData;					// 0x0207
    ext_bullet_remaining_t			BulletRemaining;			// 0x0208
    ext_rfid_status_t				RfidStatus;					// 0x0209
	
    ext_aerial_data_t                AerialData;                  //0x301	

    ext_robot_command_t             command;
    int16_t		offline_cnt;
    int16_t		offline_max_cnt;

} judge_info_t;

typedef struct judge_sensor_struct {
    judge_info_t	*info;
    drv_uart_t			*driver;
    void				(*init)(struct judge_sensor_struct *self);
    void				(*update)( uint8_t *rxBuf);
    void				(*check)(struct judge_sensor_struct *self);
    void				(*heart_beat)(struct judge_sensor_struct *self);
    dev_work_state_t	work_state;
    dev_errno_t			errno;
    dev_id_t			id;
} judge_sensor_t;

extern judge_sensor_t	judge_sensor;
extern judge_info_t 	judge_sensor_info;

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif
