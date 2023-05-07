#ifndef __RC_H
#define __RC_H

#include "rp_config.h"
#include "pid.h"
#include "motor.h"
#include "bmi_solve.h"

//控制相关宏定义

/* Exported macro ------------------------------------------------------------*/

/* ----------------------- RC Switch Definition-------------------------------*/

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

#define    SW1    0
#define    SW2    1

/*遥控器消抖判断时间*/
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
    POS,    //定位模式
    HAND,   //手动模式
    AUTO,   //自动模式
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
    uint8_t state;              //状态机
    uint8_t launch_num;        //发射次数
    uint8_t dart_wheel_cnt;      //供弹轮计数
    uint8_t reset_flag;        //复位标志，开始复位置1，全部复位完成置2
    uint8_t shoot_cmd;          //发射指令，装弹完成后置1发射
    int32_t timetick;          //状态机计时
    uint32_t fail_timetick;          //装弹计时
    uint32_t fail_flag;             //装弹失败标志
    const int32_t belt_mid_up;       //上膛块在顶端位置
    const int32_t belt_mid_upmid;    //上膛块刚好撞到限位块的位置
    const int32_t belt_mid_mid;      //上膛块在供弹轮、推板前不会撞上的位置
    const int32_t belt_mid_down;     //上膛块在上膛前的位置
    const int32_t belt_mid_in;       //上膛块在上膛位置
    const int32_t belt_side_cur_lim; //推板压紧时的最大电流
    const int32_t belt_side_up;       //推板同步带在顶端
    const int32_t belt_side_down;       //推板同步带在压实位置偏上
    const int32_t belt_side_in;       //推板同步带在压实位置
    const int32_t belt_side_mid;       //推板同步带在推板不会和供弹轮干涉的位置
    const int32_t dart_wheel_out[4];  //供弹轮不阻挡轨道滑块的位置
    const int32_t dart_wheel_in[3];   //供弹轮上膛位置
    const int32_t push_turn_out;      //推板不阻挡轨道滑块的位置
    const int32_t push_turn_in;       //推板上膛位置
    const uint16_t servo_lock;         //撒放器锁住的舵机pwm占空比
    const uint16_t servo_unlock;       //撒放器解锁的舵机pwm占空比
}load_t;

typedef __packed struct
{
    uint8_t dart_launch_opening_status;     //当前飞镖发射口的状态 1：关闭 2：正在开启或者关闭中 0：已经开启
    uint8_t dart_attack_target;     //飞镖的打击目标 0：前哨站 1：基地。
    uint16_t target_change_time;    //切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
    uint16_t operate_launch_cmd_time;   //最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0
    uint8_t dart_remaining_time;    //飞镖发射口倒计时
    uint8_t game_type : 4;          //当前比赛类型，1：RoboMaster机甲大师赛；2：RoboMaster机甲大师单项赛；
    uint8_t game_progress : 4;      //当前比赛阶段，4：对战中；
    uint16_t stage_remain_time;     //当前阶段剩余时间，单位s
    uint8_t robot_id;               //本机器人id，8：红方飞镖机器人；108：蓝方飞镖机器人
    uint16_t red_outpost_HP;        //红方前哨站血量
    uint16_t red_base_HP;           //红方基地血量
    uint16_t blue_outpost_HP;       //蓝方前哨站血量
    uint16_t blue_base_HP;          //蓝方基地血量
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
    uint8_t dart_launch_opening_status;     //当前飞镖发射口的状态 1：关闭 2：正在开启或者关闭中 0：已经开启
    uint8_t game_type : 4;          //当前比赛类型，1：RoboMaster机甲大师赛；2：RoboMaster机甲大师单项赛；
    uint8_t game_progress : 4;      //当前比赛阶段，4：对战中；
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

