#ifndef __RC_H
#define __RC_H

#include "rp_config.h"
#include "pid.h"
#include "motor.h"
#include "bmi_solve.h"

//������غ궨��

/* Exported macro ------------------------------------------------------------*/

/* ----------------------- RC Switch Definition-------------------------------*/

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

#define    SW1    0
#define    SW2    1

/*ң���������ж�ʱ��*/
#define RC_S_TIM   50    //ms


/* Exported types ------------------------------------------------------------*/
typedef struct rc_sensor_info_struct {
	int16_t 	ch0;
	int16_t 	ch1;
	int16_t 	ch2;
	int16_t 	ch3;
	uint8_t  	s1;
	uint8_t  	s2;
	int16_t		mouse_vx;
	int16_t 	mouse_vy;
	int16_t 	mouse_vz;
	uint8_t 	mouse_btn_l;
	uint8_t 	mouse_btn_r;
	int16_t 	thumbwheel;	
	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;
    
    union {
        uint16_t key_code;
        struct
        {
          uint16_t W : 1;
          uint16_t S : 1;
          uint16_t A : 1;
          uint16_t D : 1;
          uint16_t SHIFT : 1;
          uint16_t CTRL : 1;
          uint16_t Q : 1;
          uint16_t E : 1;
          uint16_t R : 1;
          uint16_t F : 1;
          uint16_t G : 1;
          uint16_t Z : 1;
          uint16_t X : 1;
          uint16_t C : 1;
          uint16_t V : 1;
          uint16_t B : 1;
        } bit;
    } kb;
} rc_sensor_info_t;

typedef struct rc_chx_flag_struct{
    uint8_t max;
    uint8_t min;
    uint8_t mid;
}chx_flag_t;

typedef struct rc_sensor_struct {
	rc_sensor_info_t	*info;
	drv_uart_t			*driver;
	void				(*init)(struct rc_sensor_struct *self);
	void				(*update)(struct rc_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct rc_sensor_struct *self);	
	void				(*heart_beat)(struct rc_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
} rc_sensor_t;

typedef enum{
    IDLE,
    UP_MID,
    MID_DOWN,
    DOWN_MID,
    MID_UP,
} rc_switch_turnvalue_t;

typedef enum{
    TW_IDLE,
    TW_UP,
    TW_DOWN,
    TW_UP_LONG,
    TW_DOWN_LONG,
} rc_thumbwheel_turnvalue_t;

typedef enum{
    POS,    //��λģʽ
    HAND,   //�ֶ�ģʽ
    AUTO,   //�Զ�ģʽ
} rc_mode_t;

typedef struct rc_switch_turn {
    uint8_t last_sw_value;
    rc_switch_turnvalue_t turnvalue;
} rc_switch_turn_t;

typedef struct rc_thumbwheel_turn {
    int32_t last_tb_state;
    rc_thumbwheel_turnvalue_t turnvalue;
} rc_thumbwheel_turn_t;

typedef struct _load_t{
    uint8_t state;              //״̬��
    uint8_t launch_num;        //�������
    uint8_t dart_wheel_cnt;      //�����ּ���
    uint8_t reset_flag;        //��λ��־����ʼ��λ��1��ȫ����λ�����2
    uint8_t shoot_cmd;          //����ָ�װ����ɺ���1����
    int32_t timetick;          //״̬����ʱ
    uint32_t fail_timetick;          //װ����ʱ
    uint32_t fail_flag;             //װ��ʧ�ܱ�־
    const int32_t belt_mid_up;       //���ſ��ڶ���λ��
    const int32_t belt_mid_upmid;    //���ſ�պ�ײ����λ���λ��
    const int32_t belt_mid_mid;      //���ſ��ڹ����֡��ư�ǰ����ײ�ϵ�λ��
    const int32_t belt_mid_down;     //���ſ�������ǰ��λ��
    const int32_t belt_mid_in;       //���ſ�������λ��
    const int32_t belt_side_cur_lim; //�ư�ѹ��ʱ��������
    const int32_t belt_side_up;       //�ư�ͬ�����ڶ���
    const int32_t belt_side_down;       //�ư�ͬ������ѹʵλ��ƫ��
    const int32_t belt_side_in;       //�ư�ͬ������ѹʵλ��
    const int32_t belt_side_mid;       //�ư�ͬ�������ư岻��͹����ָ����λ��
    const int32_t dart_wheel_out[4];  //�����ֲ��赲��������λ��
    const int32_t dart_wheel_in[3];   //����������λ��
    const int32_t push_turn_out;      //�ư岻�赲��������λ��
    const int32_t push_turn_in;       //�ư�����λ��
    const uint16_t servo_lock;         //��������ס�Ķ��pwmռ�ձ�
    const uint16_t servo_unlock;       //�����������Ķ��pwmռ�ձ�
}load_t;

typedef __packed struct
{
    uint8_t dart_launch_opening_status;     //��ǰ���ڷ���ڵ�״̬ 1���ر� 2�����ڿ������߹ر��� 0���Ѿ�����
    uint8_t dart_attack_target;     //���ڵĴ��Ŀ�� 0��ǰ��վ 1�����ء�
    uint16_t target_change_time;    //�л����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ�룬��δ�л�Ĭ��Ϊ 0��
    uint16_t operate_launch_cmd_time;   //���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��, ��ʼֵΪ 0
    uint8_t dart_remaining_time;    //���ڷ���ڵ���ʱ
    uint8_t game_type : 4;          //��ǰ�������ͣ�1��RoboMaster���״�ʦ����2��RoboMaster���״�ʦ��������
    uint8_t game_progress : 4;      //��ǰ�����׶Σ�4����ս�У�
    uint16_t stage_remain_time;     //��ǰ�׶�ʣ��ʱ�䣬��λs
    uint8_t robot_id;               //��������id��8���췽���ڻ����ˣ�108���������ڻ�����
    uint16_t red_outpost_HP;        //�췽ǰ��վѪ��
    uint16_t red_base_HP;           //�췽����Ѫ��
    uint16_t blue_outpost_HP;       //����ǰ��վѪ��
    uint16_t blue_base_HP;          //��������Ѫ��
} referee_data_t;

typedef struct
{
    uint8_t shoot_flag;
    uint8_t state;
    uint16_t timetick;
    uint16_t target_HP;
    uint16_t target;
}flash_sol_t;

typedef struct
{
    uint8_t state;
    uint8_t ctrl_mode;
    uint8_t gate_on_num;
    uint8_t gate_on_flag;
    uint8_t target_switching;
    uint8_t last_gate_state;
    uint16_t target_HP;
    uint32_t gate_on_time;
    uint32_t timetick;
    int32_t pitch_target[2];
    int32_t yaw_target[2];
    int32_t gimbal_target[2];
}auto_mod_t;

typedef struct
{
    uint8_t dart_launch_opening_status;     //��ǰ���ڷ���ڵ�״̬ 1���ر� 2�����ڿ������߹ر��� 0���Ѿ�����
    uint8_t game_type : 4;          //��ǰ�������ͣ�1��RoboMaster���״�ʦ����2��RoboMaster���״�ʦ��������
    uint8_t game_progress : 4;      //��ǰ�����׶Σ�4����ս�У�
}auto_mod_test_t;

extern rc_sensor_info_t rc_sensor_info;
extern rc_sensor_t 		rc_sensor;
extern referee_data_t     referee_data;

/* Exported functions --------------------------------------------------------*/
bool RC_IsChannelReset(void);
void RC_ResetData(rc_sensor_t *rc);

void USART2_rxDataHandler(uint8_t *rxBuf);
void rc_control(void);

void led_program(void);

void rc_ch_state_get(void);
void rc_sw_state_get(void);
void rc_tw_state_get(void);
void mode_solve(void);

#endif

