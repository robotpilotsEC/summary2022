#include "rc.h"
#include "math.h"
#include "can.h"
#include "tim.h"
#include "motor.h"
#include "bmi.h"
#include "bmi_solve.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include <stdlib.h> 
#include "kalman.h"
#include "flash.h"

#define abs(x) 					((x)>0 ? (x):(-(x)))
#define min(a,b)                ((a)<(b) ? (a):(b))
#define max(a,b)                ((a)>(b) ? (a):(b))

extern void rc_sensor_init(rc_sensor_t *rc_sen);
extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);

rc_mode_t mode;
load_t load = {
    .servo_lock = 1650,         //������������Ӧ��PWMֵ
    .servo_unlock = 1450,       //������������Ӧ��PWMֵ
    .belt_mid_up = -4000,       //����ͬ�������ڶ���
    .belt_mid_upmid = -385300,  //����ͬ��������ʹ���ſ�ǡ��������λ���λ��
    .belt_mid_mid = -1100000,   //����ͬ�������ڲ���ʹ��λ��ײ���ư�����λ��
    .belt_mid_in = -1821500,    //����ͬ��������λ��ѹ������λ��
    .belt_mid_down = -1773000,  //����ͬ��������λ��ѹ������ǰ��λ��
    .belt_side_cur_lim = 5000,  //�෽ͬ�����������ʱ�ĵ�������
    .push_turn_out = -65000,    //�ư�����
    .push_turn_in = -6350,      //�ư�����
    .belt_side_down = -550000,
    .belt_side_up = -12500,     //�෽ͬ�������ڶ���
    .belt_side_in = -645500,    //�෽ͬ����������ѹ������λ��
    .belt_side_mid = -154600,   //�෽ͬ���������ư�������������λ��
    .dart_wheel_out[0] = -32550,//������λ��
    .dart_wheel_out[1] = 19000,
    .dart_wheel_out[2] = 71300,
    .dart_wheel_out[3] = 125000,
    .dart_wheel_in[0] = -11980,
    .dart_wheel_in[1] = 40148,
    .dart_wheel_in[2] = 93700,
};

int16_t current_temp1[4] = {0};
int16_t current_temp2[4] = {0};

//int32_t actualspeed_belt_mid = 0;
//int32_t actualspeed_belt_side = 0;
//int32_t actualspeed_push_turn = 0;
//int32_t actualspeed_dart_wheel = 0;
//int32_t actualspeed_pitch = 0;
//int32_t actualspeed_yaw = 0;
//int32_t actualspeed_gimbal = 0;
//int32_t actualangle_belt_mid = 0;
//int32_t actualangle_belt_side = 0;
//int32_t actualangle_push_turn = 0;
//int32_t actualangle_dart_wheel = 0;
//int32_t actualangle_pitch = 0;
//int32_t actualangle_yaw = 0;
//int32_t actualangle_gimbal = 0;

//int32_t setspeed_belt_mid = 0;
//int32_t setspeed_belt_side = 0;
//int32_t setspeed_push_turn = 0;
//int32_t setspeed_dart_wheel = 0;
//int32_t setspeed_pitch = 0;
//int32_t setspeed_yaw = 0;
//int32_t setspeed_gimbal = 0;
//int32_t setangle_belt_mid = 0;
//int32_t setangle_belt_side = 0;
//int32_t setangle_push_turn = 0;
//int32_t setangle_dart_wheel = 0;
//int32_t setangle_pitch = 0;
//int32_t setangle_yaw = 0;
//int32_t setangle_gimbal = 0;

chx_flag_t ch0_flag;
chx_flag_t ch1_flag;
chx_flag_t ch2_flag;
chx_flag_t ch3_flag;


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t *rc_sen);
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// ң��������
drv_uart_t	rc_sensor_driver = {
	.type = DRV_UART2,
	.tx_byte = NULL,
};

// ң������Ϣ
rc_sensor_info_t 	rc_sensor_info = {
	.offline_max_cnt = 60,
};

// ң����������
rc_sensor_t	rc_sensor = {
	.info = &rc_sensor_info,								//���ݽṹ��
	.init = rc_sensor_init,									//��������ʼ��
	.update = rc_sensor_update,							//���ݸ���
	.check = rc_sensor_check,								//���ݺ������ж�
	.heart_beat = rc_sensor_heart_beat,			//״̬����
	.work_state = DEV_OFFLINE,							//״̬��ѯ
	.id = DEV_ID_RC,
};

rc_switch_turn_t rc_sw_turn[2];
rc_thumbwheel_turn_t rc_tw_turn;
referee_data_t referee_data;
auto_mod_t auto_mod = {
    .pitch_target[0] = 520000,
    .pitch_target[1] = 520000,
    .yaw_target[0] = -2388150,
    .yaw_target[1] = -2388150,
    .gimbal_target[0] = 26700,
    .gimbal_target[1] = 26700,
};
flash_sol_t flash_sol;

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	ң�������ݼ��
 */
static void rc_sensor_check(rc_sensor_t *rc_sen)
{
	rc_sensor_info_t *rc_info = rc_sen->info;
	
	if(abs(rc_info->ch0) > 660 ||
	   abs(rc_info->ch1) > 660 ||
	   abs(rc_info->ch2) > 660 ||
	   abs(rc_info->ch3) > 660)
	{
		rc_sen->errno = DEV_DATA_ERR;
		rc_info->ch0 = 0;
		rc_info->ch1 = 0;
		rc_info->ch2 = 0;
		rc_info->ch3 = 0;		
		rc_info->s1 = 0;
		rc_info->s2 = 0;
		rc_info->thumbwheel = 0;
	}
	else
	{
		rc_sen->errno = NONE_ERR;
	}
}

/**
 *	@brief	ң����������
 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)//��������
{
	rc_sensor_info_t *rc_info = rc_sen->info;

	rc_info->offline_cnt++;
	if(rc_info->offline_cnt > rc_info->offline_max_cnt)//ÿ�εȴ�һ��ʱ����Զ�����
	{
		rc_info->offline_cnt = rc_info->offline_max_cnt;
		rc_sen->work_state = DEV_OFFLINE;
	} 
	else //ÿ�ν��ճɹ�����ռ���
	{
		/* ����->���� */
		if(rc_sen->work_state == DEV_OFFLINE)
			rc_sen->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/
float DeathZoom(float input, float center, float death)
{
	if(abs(input - center) < death)
		return center;
	return input;
}

bool RC_IsChannelReset(void)//ң�ع��в�ѯ������һ�㸴λʱʹ��
{
	if(  (DeathZoom(rc_sensor_info.ch0, 0, 50) == 0) && 
		 (DeathZoom(rc_sensor_info.ch1, 0, 50) == 0) && 
		 (DeathZoom(rc_sensor_info.ch2, 0, 50) == 0) && 
		 (DeathZoom(rc_sensor_info.ch3, 0, 50) == 0)   )	
	{
		return true;
	}
	return false;		
}

void RC_ResetData(rc_sensor_t *rc)//ң��������գ�һ��ʧ��ʱʹ��
{
	// ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬)
	rc->info->ch0 = 0;
	rc->info->ch1 = 0;
	rc->info->ch2 = 0;
	rc->info->ch3 = 0;
	// ���ҿ���ѡ�����óɴ���ֵ״̬
	rc->info->s1 = 0;
	rc->info->s2 = 0;
	// ���
	rc->info->mouse_vx = 0;
	rc->info->mouse_vy = 0;
	rc->info->mouse_vz = 0;
	rc->info->mouse_btn_l = 0;
	rc->info->mouse_btn_r = 0;
	// ����
	rc->info->kb.key_code = 0;
	// ����
	rc->info->thumbwheel = 0;
}

void rc_sensor_init(rc_sensor_t *rc_sen)
{
	// ��ʼ��Ϊ����״̬
	rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
	rc_sen->work_state = DEV_OFFLINE;
	
	if(rc_sen->id == DEV_ID_RC)
		rc_sen->errno = NONE_ERR;
	else
		rc_sen->errno = DEV_ID_ERR;
}

/**
 *	@brief	ң�������ݽ���Э��
 */
void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf)
{
	rc_sensor_info_t *rc_info = rc_sen->info;
	
	rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;
	rc_info->ch0 -= 1024;
	rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF;
	rc_info->ch1 -= 1024;
	rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
	rc_info->ch2 -= 1024;
	rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF;
	rc_info->ch3 -= 1024;

	rc_info->s1 = ((rxBuf[5] >> 4) & 0x000C) >> 2;
	rc_info->s2 = (rxBuf[5] >> 4) & 0x0003;	
	
	rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8);
	rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8);
	rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);
	rc_info->mouse_btn_l = rxBuf[12];
	rc_info->mouse_btn_r = rxBuf[13];
	
	rc_info->kb.key_code = rxBuf[14] | (rxBuf[15] << 8);	
	
	rc_info->thumbwheel = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	rc_info->thumbwheel -= 1024;
	
	rc_info->offline_cnt = 0;
}

/**
 *	@brief	�ڴ���2�н���ң������Э��
 */
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	// ����ң������
	rc_sensor.update(&rc_sensor, rxBuf);
	rc_sensor.check(&rc_sensor);
}


void rc_control(void)
{   
    if(rc_sensor.work_state == DEV_OFFLINE)
    {
        //���ߴ���
        RC_ResetData(&rc_sensor);
        rc_sw_turn[SW1].turnvalue = IDLE;
        rc_sw_turn[SW1].last_sw_value = 0;
        rc_sw_turn[SW2].turnvalue = IDLE;
        rc_sw_turn[SW2].last_sw_value = 0;
    }
    else if(rc_sensor.work_state == DEV_ONLINE)
    {
        rc_ch_state_get();
        rc_sw_state_get();
        rc_tw_state_get();
    }
    mode_solve();

    //���pid
    for(motor_cnt_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
    {
        motor[cnt].pid_a->actual = motor[cnt].info->angle_sum;
        pid_angle_control(&pid_angle[cnt]);
        if(pid_angle[cnt].sw == 1)
        {
            //����ǶȻ�ʹ�ܣ���ѽǶȻ�����������ٶȻ�
            pid_speed[cnt].set = pid_angle[cnt].out;
        }
        pid_speed[cnt].actual = motor[cnt].info->speed;
        pid_speed_control(&pid_speed[cnt]);
    }
    
    //�ַ����ݿ��Ƶ��
    
    current_temp1[BELT_MID] = pid_speed[BELT_MID].out;
    current_temp1[BELT_SIDE] = pid_speed[BELT_SIDE].out;
    current_temp1[PUSH_TURN] = pid_speed[PUSH_TURN].out;
    current_temp1[DART_WHEEL] = pid_speed[DART_WHEEL].out;
    CAN_SendData(&hcan1, 0x200, current_temp1);
    current_temp2[PITCH_M-4] = pid_speed[PITCH_M].out;
    current_temp2[YAW_M-4] = pid_speed[YAW_M].out;
    current_temp2[GIMBAL-4] = pid_speed[GIMBAL].out;
    current_temp2[3] = 0;
    CAN_SendData(&hcan2, 0x1FF, current_temp2);
    
    //jscope
//    actualspeed_belt_mid = motor[BELT_MID].info->speed;
//    actualspeed_belt_side = motor[BELT_SIDE].info->speed;
//    actualspeed_push_turn = motor[PUSH_TURN].info->speed;
//    actualspeed_dart_wheel = motor[DART_WHEEL].info->speed;
//    actualspeed_pitch = motor[PITCH_M].info->speed;
//    actualspeed_yaw = motor[YAW_M].info->speed;
//    actualspeed_gimbal = motor[GIMBAL].info->speed;
//    
//    actualangle_belt_mid = motor[BELT_MID].info->angle_sum;
//    actualangle_belt_side = motor[BELT_SIDE].info->angle_sum;
//    actualangle_push_turn = motor[PUSH_TURN].info->angle_sum;
//    actualangle_dart_wheel = motor[DART_WHEEL].info->angle_sum;
//    actualangle_pitch = motor[PITCH_M].info->angle_sum;
//    actualangle_yaw = motor[YAW_M].info->angle_sum;
//    actualangle_gimbal = motor[GIMBAL].info->angle_sum;
//    
//    setspeed_belt_mid = motor[BELT_MID].info->speed;
//    setspeed_belt_side = motor[BELT_SIDE].info->speed;
//    setspeed_push_turn = motor[PUSH_TURN].info->speed;
//    setspeed_dart_wheel = motor[DART_WHEEL].info->speed;
//    setspeed_pitch = motor[PITCH_M].info->speed;
//    setspeed_yaw = motor[YAW_M].info->speed;
//    setspeed_gimbal = motor[GIMBAL].info->speed;
//    
//    setangle_belt_mid = motor[BELT_MID].info->angle_sum;
//    setangle_belt_side = motor[BELT_SIDE].info->angle_sum;
//    setangle_push_turn = motor[PUSH_TURN].info->angle_sum;
//    setangle_dart_wheel = motor[DART_WHEEL].info->angle_sum;
//    setangle_pitch = motor[PITCH_M].info->angle_sum;
//    setangle_yaw = motor[YAW_M].info->angle_sum;
//    setangle_gimbal = motor[GIMBAL].info->angle_sum;
}

//ͨ��״̬��ȡ
void rc_ch_state_get()
{
    if(rc_sensor_info.ch0 == 0)
    {
        ch0_flag.mid = 1;
    }
    if(ch0_flag.mid == 1)
    {
        if(rc_sensor_info.ch0 > 500)
        {
            ch0_flag.mid = 0;
            ch0_flag.max = 1;
        }
        else if(rc_sensor_info.ch0 < -500)
        {
            ch0_flag.mid = 0;
            ch0_flag.min = 1;
        }
    }
    
    if(rc_sensor_info.ch1 == 0)
    {
        ch1_flag.mid = 1;
    }
    if(ch1_flag.mid == 1)
    {
        if(rc_sensor_info.ch1 > 500)
        {
            ch1_flag.mid = 0;
            ch1_flag.max = 1;
        }
        else if(rc_sensor_info.ch1 < -500)
        {
            ch1_flag.mid = 0;
            ch1_flag.min = 1;
        }
    }
    
    if(rc_sensor_info.ch2 == 0)
    {
        ch2_flag.mid = 1;
    }
    if(ch2_flag.mid == 1)
    {
        if(rc_sensor_info.ch2 > 500)
        {
            ch2_flag.mid = 0;
            ch2_flag.max = 1;
        }
        else if(rc_sensor_info.ch2 < -500)
        {
            ch2_flag.mid = 0;
            ch2_flag.min = 1;
        }
    }
    
    if(rc_sensor_info.ch3 == 0)
    {
        ch3_flag.mid = 1;
    }
    if(ch3_flag.mid == 1)
    {
        if(rc_sensor_info.ch3 > 500)
        {
            ch3_flag.mid = 0;
            ch3_flag.max = 1;
        }
        else if(rc_sensor_info.ch3 < -500)
        {
            ch3_flag.mid = 0;
            ch3_flag.min = 1;
        }
    }
}

//ָ��״̬��ȡ
void rc_tw_state_get()
{
    static uint32_t timetick_up = 0;
    static uint32_t timetick_down = 0;
    
    timetick_up = (rc_sensor_info.thumbwheel < -600) ? ++timetick_up : 0;
    timetick_down = (rc_sensor_info.thumbwheel > 600) ? ++timetick_down : 0;
    
    rc_tw_turn.turnvalue = TW_IDLE;
    //�ж����ϲ�����ʱ
    if(rc_sensor_info.thumbwheel < -600)
    {
        rc_tw_turn.last_tb_state --;
    }
    //�ж����²�����ʱ
    else if(rc_sensor_info.thumbwheel > 600)
    {
        rc_tw_turn.last_tb_state ++;
    }
    //�ж϶̲�����
    else if(abs(rc_sensor_info.thumbwheel) < 400 && rc_tw_turn.last_tb_state != 0)
    {
        if(rc_tw_turn.last_tb_state < -1000)
        {
            rc_tw_turn.turnvalue = TW_UP_LONG;
        }
        else if(rc_tw_turn.last_tb_state < 0)
        {
            rc_tw_turn.turnvalue = TW_UP;
        }
        else if(rc_tw_turn.last_tb_state < 1000)
        {
            rc_tw_turn.turnvalue = TW_DOWN;
        }
        else
        {
            rc_tw_turn.turnvalue = TW_DOWN_LONG;
        }
        rc_tw_turn.last_tb_state = 0;
    }
}

//����״̬��ȡ
void rc_sw_state_get()
{
    rc_sw_turn[SW1].turnvalue = IDLE;
    rc_sw_turn[SW2].turnvalue = IDLE;
    //S1״̬�л�
    if(rc_sensor.info->s1 == RC_SW_MID)
    {
        if(rc_sw_turn[SW1].last_sw_value == RC_SW_DOWN)
        {
            rc_sw_turn[SW1].turnvalue = DOWN_MID;  //�²�����
        }
        else if(rc_sw_turn[SW1].last_sw_value == RC_SW_UP)
        {
            rc_sw_turn[SW1].turnvalue = UP_MID;  //�ϲ�����
        }
        rc_sw_turn[SW1].last_sw_value = RC_SW_MID;
    }
    else if(rc_sensor.info->s1 == RC_SW_UP)
    {
        if(rc_sw_turn[SW1].last_sw_value == RC_SW_MID)
        {
            rc_sw_turn[SW1].turnvalue = MID_UP;  //�в�����
        }
        rc_sw_turn[SW1].last_sw_value = RC_SW_UP;
    }
    else if(rc_sensor.info->s1 == RC_SW_DOWN)
    {
        if(rc_sw_turn[SW1].last_sw_value == RC_SW_MID)
        {
            rc_sw_turn[SW1].turnvalue = MID_DOWN;  //�в�����
        }
        rc_sw_turn[SW1].last_sw_value = RC_SW_DOWN;
    }
    //S2״̬�л�
    if(rc_sensor.info->s2 == RC_SW_MID)
    {
        if(rc_sw_turn[SW2].last_sw_value == RC_SW_DOWN)
        {
            rc_sw_turn[SW2].turnvalue = DOWN_MID;  //�²�����
        }
        else if(rc_sw_turn[SW2].last_sw_value == RC_SW_UP)
        {
            rc_sw_turn[SW2].turnvalue = UP_MID;  //�ϲ�����
        }
        rc_sw_turn[SW2].last_sw_value = RC_SW_MID;
    }
    else if(rc_sensor.info->s2 == RC_SW_UP)
    {
        if(rc_sw_turn[SW2].last_sw_value == RC_SW_MID)
        {
            rc_sw_turn[SW2].turnvalue = MID_UP;  //�в�����
        }
        rc_sw_turn[SW2].last_sw_value = RC_SW_UP;
    }
    else if(rc_sensor.info->s2 == RC_SW_DOWN)
    {
        if(rc_sw_turn[SW2].last_sw_value == RC_SW_MID)
        {
            rc_sw_turn[SW2].turnvalue = MID_DOWN;  //�в�����
        }
        rc_sw_turn[SW2].last_sw_value = RC_SW_DOWN;
    }
}

void auto_load()
{
    if(rc_sensor.work_state == DEV_OFFLINE)
    {
        load.timetick = 0;
        if(load.state == 7 || load.state == 8)
        {
            //��������Ų���7��8ʱ���ߣ���ָ�������ֵ��6000
            motor[BELT_SIDE].pid_s->outmax = 6000;
        }
        load.state = 0;
    }
    else if(rc_sensor.work_state == DEV_ONLINE)
    {
        if(load.state > 0 && load.state < 9)
        {
            load.fail_timetick ++;
            if(load.fail_timetick > 12000)
            {
                //����ʱ����㣬������12��δ������ɣ����ж�Ϊ����ʧ�ܣ����ж��
                //ʧ�ܱ�־λ��1�������ٴ����ţ�����ң�ص�������ϲ�ָ�ֿɻָ�
                load.state = 0;
                load.fail_flag = 1;
                for(motor_cnt_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
                {
                    motor[cnt].pid_a->sw = 0;
                    motor[cnt].pid_s->sw = 0;
                }
            }
        }
        else if(load.state == 0)
        {
            load.fail_timetick = 0;
        }
        
        if(rc_tw_turn.turnvalue == TW_UP)
        {
            //���ϲ�ָ�֣�ʧ�ܱ�־λ��0����������
            load.fail_flag = 0;
            for(uint8_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
            {
                motor[cnt].pid_s->sw = 1;
                motor[cnt].pid_a->sw = 1;
                motor[cnt].pid_a->set = motor[cnt].info->angle_sum;
            }
        }
        
        if(load.state == 0)
        {
            load.timetick = 0;
            motor[BELT_MID].pid_a->outmax = 5000;   //����ͬ���������ת����Ϊ5000
        }
        else if(load.state == 1)
        {
            if(load.reset_flag != 3)
            {
                //�����λδ��ɣ�����������
                load.state = 0;
                return;
            }
            motor[BELT_MID].pid_a->sw = 1;
            motor[BELT_MID].pid_s->sw = 1;
            motor[BELT_MID].pid_a->set = load.belt_mid_mid;
            motor[BELT_SIDE].pid_a->sw = 1;
            motor[BELT_SIDE].pid_s->sw = 1;
            motor[BELT_SIDE].pid_a->set = load.belt_side_up;
            if(load.launch_num <= 1)
            {
                //��һ�����ڣ��������ſ���ٶ��Լ������ſ�����λ�����ײ������·�������
                int32_t temp = motor[BELT_MID].pid_a->actual - load.belt_mid_upmid;
                motor[BELT_MID].pid_a->outmax = max(min(temp>0 ? (temp/5) : (-temp/50+500), 5000), 500);
            }
            load.timetick = (abs(motor[BELT_SIDE].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 2;
                if(load.launch_num < 4)
                {
                    load.launch_num++;
                }
                else
                {
                    load.launch_num = 1;
                }
            }
        }
        else if(load.state == 2)
        {
            motor[DART_WHEEL].pid_a->sw = 1;
            motor[DART_WHEEL].pid_s->sw = 1;
            motor[DART_WHEEL].pid_a->set = load.dart_wheel_out[load.dart_wheel_cnt];
            motor[PUSH_TURN].pid_a->sw = 1;
            motor[PUSH_TURN].pid_s->sw = 1;
            motor[PUSH_TURN].pid_a->set = load.push_turn_out;
            if(load.launch_num <= 1)
            {
                int32_t temp = motor[BELT_MID].pid_a->actual - load.belt_mid_upmid;
                motor[BELT_MID].pid_a->outmax = max(min(temp>0 ? (temp/5) : (-temp/30+1500), 5000), 1500);
            }
            load.timetick = (abs(motor[DART_WHEEL].pid_a->err) < 200 && abs(motor[PUSH_TURN].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 3;
            }
        }
        else if(load.state == 3)
        {
            motor[BELT_MID].pid_a->sw = 1;
            motor[BELT_MID].pid_s->sw = 1;
            motor[BELT_MID].pid_a->set = load.belt_mid_in;
            htim1.Instance->CCR2 = load.servo_unlock;
            if(load.launch_num <= 1)
            {
                int32_t temp = motor[BELT_MID].pid_a->actual - load.belt_mid_upmid;
                motor[BELT_MID].pid_a->outmax = max(min(temp>0 ? (temp/5) : (-temp/30+1500), 5000), 1500);
            }
            load.timetick = (abs(motor[BELT_MID].pid_a->err) < 500) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 4;
            }
        }
        else if(load.state == 4)
        {
            htim1.Instance->CCR2 = load.servo_lock;
            load.timetick++;
            if(load.timetick > 400)
            {
                load.timetick = 0;
                if(load.launch_num == 1)
                {
                    load.state = 6;
                }
                else
                {
                    load.state = 5;
                }
            }
        }
        else if(load.state == 5)
        {
            motor[BELT_MID].pid_a->sw = 1;
            motor[BELT_MID].pid_s->sw = 1;
            motor[BELT_MID].pid_a->outmax = 500;
            motor[BELT_MID].pid_a->set = load.belt_mid_up;
            if(motor[BELT_MID].pid_a->outmax < 5000)
            {
                motor[BELT_MID].pid_a->outmax += 10;
            }
            motor[DART_WHEEL].pid_a->sw = 1;
            motor[DART_WHEEL].pid_s->sw = 1;
            motor[DART_WHEEL].pid_a->set = load.dart_wheel_in[load.dart_wheel_cnt];
            motor[PUSH_TURN].pid_a->sw = 1;
            motor[PUSH_TURN].pid_s->sw = 1;
            motor[PUSH_TURN].pid_a->set = load.push_turn_in;
            load.timetick = (abs(motor[DART_WHEEL].pid_a->err) < 200 && abs(motor[PUSH_TURN].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 7;
                load.dart_wheel_cnt++;
            }
        }
        else if(load.state == 6)
        {
            motor[BELT_MID].pid_a->sw = 1;
            motor[BELT_MID].pid_s->sw = 1;
            motor[BELT_MID].pid_a->outmax = 500;
            motor[BELT_MID].pid_a->set = load.belt_mid_up;
            if(motor[BELT_MID].pid_a->outmax < 5000)
            {
                motor[BELT_MID].pid_a->outmax += 10;
            }
            motor[PUSH_TURN].pid_a->sw = 1;
            motor[PUSH_TURN].pid_s->sw = 1;
            motor[PUSH_TURN].pid_a->set = load.push_turn_in;
            load.timetick = (abs(motor[PUSH_TURN].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 7;
            }
        }
        else if(load.state == 7)
        {
            motor[BELT_MID].pid_a->sw = 1;
            motor[BELT_MID].pid_s->sw = 1;
            if(motor[BELT_MID].pid_a->outmax < 5000)
            {
                motor[BELT_MID].pid_a->outmax += 10;
            }
            motor[BELT_MID].pid_a->set = load.belt_mid_up;
            motor[BELT_SIDE].pid_a->sw = 1;
            motor[BELT_SIDE].pid_s->sw = 1;
            motor[BELT_SIDE].pid_a->set = load.belt_side_in;
            if(motor[BELT_SIDE].info->angle_sum < load.belt_side_down)
            {
                motor[BELT_SIDE].pid_s->outmax = load.belt_side_cur_lim;  //���Ƶ���ֵ
                motor[DART_WHEEL].pid_a->sw = 1;
                motor[DART_WHEEL].pid_s->sw = 1;
                motor[DART_WHEEL].pid_a->set = load.dart_wheel_out[load.dart_wheel_cnt];
                load.timetick = (abs(motor[DART_WHEEL].pid_a->err) < 200 \
                                && abs(motor[BELT_SIDE].pid_s->actual < 20))
                                    ? ++load.timetick : 0;
                if(load.timetick > 400)
                {
                    load.timetick = 0;
                    load.state = 8;
                }
            }
        }
        else if(load.state == 8)
        {
            motor[BELT_SIDE].pid_a->sw = 1;
            motor[BELT_SIDE].pid_s->sw = 1;
            motor[BELT_SIDE].pid_s->outmax = 6000;  //���Ƶ���ֵ
            motor[BELT_SIDE].pid_a->set = load.belt_side_up;
            if(motor[BELT_MID].pid_a->outmax < 5000)
            {
                motor[BELT_MID].pid_a->outmax += 10;
            }
            if(motor[BELT_SIDE].info->angle_sum > load.belt_side_mid)
            {
                motor[PUSH_TURN].pid_a->sw = 1;
                motor[PUSH_TURN].pid_s->sw = 1;
                motor[PUSH_TURN].pid_a->set = load.push_turn_out;
                load.timetick = (abs(motor[BELT_SIDE].pid_a->err) < 200 && abs(motor[PUSH_TURN].pid_a->err) < 200) ? ++load.timetick : 0;
                if(load.timetick > 50)
                {
                    load.timetick = 0;
                    load.state = 9;
                }
            }
        }
        else if(load.state == 9)
        {
            motor[PUSH_TURN].pid_a->sw = 1;
            motor[PUSH_TURN].pid_s->sw = 1;
            motor[PUSH_TURN].pid_a->set = load.push_turn_out;
            load.timetick = (abs(motor[PUSH_TURN].pid_a->err) < 200 && abs(motor[DART_WHEEL].pid_a->err) < 200 && abs(motor[BELT_MID].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50 && load.shoot_cmd == 1)
            {
                load.shoot_cmd = 0;
                htim1.Instance->CCR2 = load.servo_unlock;
                load.timetick = 0;
                flash_sol.shoot_flag = 1;
                if(load.dart_wheel_cnt >= 3)
                {
                    load.state = 10;
                }
                else load.state = 0;
            }
        }
        else if(load.state == 10)
        {
            load.timetick++;
            if(load.timetick > 1500)
            {
                load.timetick = 0;
                load.state = 11;
            }
        }
        else if(load.state == 11)
        {
            motor[PUSH_TURN].pid_a->sw = 1;
            motor[PUSH_TURN].pid_s->sw = 1;
            motor[PUSH_TURN].pid_a->set = load.push_turn_in;
            load.timetick = (abs(motor[PUSH_TURN].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 12;
            }
        }
        else if(load.state == 12)
        {
            motor[BELT_SIDE].pid_a->sw = 1;
            motor[BELT_SIDE].pid_s->sw = 1;
            motor[BELT_SIDE].pid_a->set = load.belt_side_in;
            load.timetick = (abs(motor[BELT_SIDE].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 13;
            }
        }
        else if(load.state == 13)
        {
            load.dart_wheel_cnt = 0;
            motor[DART_WHEEL].pid_a->sw = 1;
            motor[DART_WHEEL].pid_s->sw = 1;
            motor[DART_WHEEL].pid_a->set = load.dart_wheel_out[load.dart_wheel_cnt];
            load.timetick = (abs(motor[DART_WHEEL].pid_a->err) < 200) ? ++load.timetick : 0;
            if(load.timetick > 50)
            {
                load.timetick = 0;
                load.state = 14;
            }
        }
        else if(load.state == 14)
        {
            motor[BELT_SIDE].pid_a->sw = 1;
            motor[BELT_SIDE].pid_s->sw = 1;
            motor[BELT_SIDE].pid_a->set = load.belt_side_up;
            if(motor[BELT_SIDE].info->angle_sum > load.belt_side_mid)
            {
                motor[PUSH_TURN].pid_a->sw = 1;
                motor[PUSH_TURN].pid_s->sw = 1;
                motor[PUSH_TURN].pid_a->set = load.push_turn_out;
                load.timetick = (abs(motor[BELT_SIDE].pid_a->err) < 200 && abs(motor[PUSH_TURN].pid_a->err) < 200) ? ++load.timetick : 0;
                if(load.timetick > 50)
                {
                    load.timetick = 0;
                    load.state = 0;
                }
            }
        }
    }
}

void target_switch()
{
    static uint8_t last_target = 0;
    static uint32_t timetick = 0;
    if(last_target != referee_data.dart_attack_target)
    {
        last_target = referee_data.dart_attack_target;
        auto_mod.target_switching = 1;
        timetick = 0;
    }
    else if(auto_mod.target_switching == 1)
    {
        motor[PITCH_M].pid_a->sw = 1;
        motor[PITCH_M].pid_s->sw = 1;
        motor[PITCH_M].pid_a->set = auto_mod.pitch_target[referee_data.dart_attack_target];
        motor[YAW_M].pid_a->sw = 1;
        motor[YAW_M].pid_s->sw = 1;
        motor[YAW_M].pid_a->set = auto_mod.yaw_target[referee_data.dart_attack_target];
        timetick = (abs(motor[PITCH_M].pid_a->err) < 200 && abs(motor[YAW_M].pid_a->err) < 200) ? ++timetick : 0;
        if(timetick > 50)
        {
            auto_mod.target_switching = 0;
            timetick = 0;
        }
    }
}

void pos_mode()
{
    if(rc_sensor.work_state == DEV_OFFLINE)
    {
        for(motor_cnt_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
        {
            motor[cnt].pid_a->sw = 0;
            motor[cnt].pid_s->sw = 0;
        }
        if(load.reset_flag != 3)load.reset_flag = 0;
    }
    else if(rc_sensor.work_state == DEV_ONLINE)
    {
        if(ch2_flag.max == 1)
        {
            //�������λ��־λ����
            ch2_flag.max = 0;
            load.reset_flag = 1;
            load.launch_num = 0;
            load.dart_wheel_cnt = 0;
            motor[BELT_MID].res->state = 1;
            motor[BELT_SIDE].res->state = 1;
            motor[PITCH_M].res->state = 1;
            motor[YAW_M].res->state = 1;
//            motor[GIMBAL].res->state = 1;
        }
        else if(load.reset_flag == 1)
        {
            //���и���λ����������ư�ͬ������λ�Ƿ����
            motor[BELT_MID].reset(&motor[BELT_MID]);
            motor[BELT_SIDE].reset(&motor[BELT_SIDE]);
            motor[PITCH_M].reset(&motor[PITCH_M]);
            motor[YAW_M].reset(&motor[YAW_M]);
//            motor[GIMBAL].reset(&motor[GIMBAL]);
            if(motor[BELT_SIDE].res->state == 4)
            {
                //�ư�ͬ������λ��ɣ������ư�͹����ָ�λ��־
                load.reset_flag = 2;
                motor[PUSH_TURN].res->state = 1;
                motor[DART_WHEEL].res->state = 1;
            }
        }
        else if(load.reset_flag == 2)
        {
            //���и���λ������������λ�Ƿ����
            motor[BELT_MID].reset(&motor[BELT_MID]);
            motor[PUSH_TURN].reset(&motor[PUSH_TURN]);
            motor[DART_WHEEL].reset(&motor[DART_WHEEL]);
            motor[PITCH_M].reset(&motor[PITCH_M]);
            motor[YAW_M].reset(&motor[YAW_M]);
//            motor[GIMBAL].reset(&motor[GIMBAL]);
            if(motor[BELT_MID].res->state == 4 \
                && motor[PUSH_TURN].res->state == 4 \
                && motor[DART_WHEEL].res->state == 4 \
                && motor[PITCH_M].res->state == 4 \
                && motor[YAW_M].res->state == 4)
//                && motor[GIMBAL].res->state == 4)
            {
                load.reset_flag = 3;
            }
        }
        else if((load.reset_flag == 3 || load.reset_flag == 0) && load.fail_flag == 0)
        {
            //�ƶ�pitch�ᡢyaw��ͼ�����̨
            motor[YAW_M].pid_a->sw = 1;
            motor[YAW_M].pid_s->sw = 1;
            motor[YAW_M].pid_a->set += -rc_sensor_info.ch0 * motor[YAW_M].res->direction / 3;
            motor[PITCH_M].pid_a->sw = 1;
            motor[PITCH_M].pid_s->sw = 1;
            motor[PITCH_M].pid_a->set += rc_sensor_info.ch1 * motor[YAW_M].res->direction / 5;
//            motor[GIMBAL].pid_a->sw = 1;
//            motor[GIMBAL].pid_s->sw = 1;
//            motor[GIMBAL].pid_a->set += (float)rc_sensor_info.ch3 * motor[GIMBAL].res->direction / 150;
        }
    }
}

void hand_mode()
{
    if(rc_sensor.work_state == DEV_OFFLINE)
    {
        for(motor_cnt_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
        {
            motor[cnt].pid_a->sw = 0;
            motor[cnt].pid_s->sw = 0;
        }
    }
    else if(rc_sensor.work_state == DEV_ONLINE)
    {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        if(load.state == 0 && load.fail_flag == 0)
        {
            motor[DART_WHEEL].pid_a->sw = 1;
            motor[DART_WHEEL].pid_s->sw = 1;
            motor[DART_WHEEL].pid_a->set += -rc_sensor_info.ch0 * motor[DART_WHEEL].res->direction / 10;
            motor[BELT_MID].pid_a->sw = 1;
            motor[BELT_MID].pid_s->sw = 1;
            motor[BELT_MID].pid_a->set += rc_sensor_info.ch1 * motor[BELT_MID].res->direction / 2;
            motor[PUSH_TURN].pid_a->sw = 1;
            motor[PUSH_TURN].pid_s->sw = 1;
            motor[PUSH_TURN].pid_a->set += (float)rc_sensor_info.ch2 * motor[PUSH_TURN].res->direction / 20;
            motor[BELT_SIDE].pid_a->sw = 1;
            motor[BELT_SIDE].pid_s->sw = 1;
            motor[BELT_SIDE].pid_a->set += rc_sensor_info.ch3 * motor[BELT_SIDE].res->direction / 4;
            if(rc_sw_turn[SW2].turnvalue == MID_UP)
            {
                rc_sw_turn[SW2].turnvalue = IDLE;
                if(htim1.Instance->CCR2 == load.servo_lock)
                {
                    htim1.Instance->CCR2 = load.servo_unlock;
                }
                else
                {
                    htim1.Instance->CCR2 = load.servo_lock;
                }
            }
            else if(rc_sw_turn[SW2].turnvalue == MID_DOWN)
            {
                rc_sw_turn[SW2].turnvalue = IDLE;
                load.state = 1;
                load.timetick = 0;
            }
        }
        else if(load.state == 9)
        {
            if(rc_sw_turn[SW2].turnvalue == MID_UP)
            {
                load.shoot_cmd = 1;
            }
        }
    }
}

void auto_mode()
{
    if(rc_sensor.work_state == DEV_OFFLINE)
    {
        for(motor_cnt_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
        {
            motor[cnt].pid_a->sw = 0;
            motor[cnt].pid_s->sw = 0;
        }
    }
    else if(rc_sensor.work_state == DEV_ONLINE)
    {
//        target_switch();
        if(referee_data.game_progress != 4 && auto_mod.ctrl_mode == 0)
        {
            auto_mod.state = 0;
            auto_mod.gate_on_num = 0;
            auto_mod.gate_on_time = 0;
            auto_mod.timetick = 0;
            auto_mod.last_gate_state = 0;
        }
        else
        {
            if(auto_mod.last_gate_state != referee_data.dart_launch_opening_status)
            {
                if(auto_mod.last_gate_state == 1 && referee_data.dart_launch_opening_status == 2)
                {
                    //�����ɹر�תΪ���ڿ���
                    auto_mod.gate_on_flag = 1;
                }
                //��¼�����Ŵ���
                auto_mod.last_gate_state = referee_data.dart_launch_opening_status;
                if(auto_mod.last_gate_state == 0)
                {
                    auto_mod.gate_on_num++;
                }
            }
            
            //��¼������ʱ��
            if(referee_data.dart_launch_opening_status == 0 \
                && (auto_mod.gate_on_num == 1 || auto_mod.gate_on_num == 2))
            {
                auto_mod.gate_on_time++;
            }
            else
            {
                auto_mod.gate_on_time = 0;
            }
        }
        
        if(referee_data.dart_attack_target == 0)
        {
            if(referee_data.robot_id == 8)
            {
                auto_mod.target_HP = referee_data.blue_outpost_HP;
            }
            else if(referee_data.robot_id == 108)
            {
                auto_mod.target_HP = referee_data.red_outpost_HP;
            }
        }
        else if(referee_data.dart_attack_target == 1)
        {
            if(referee_data.robot_id == 8)
            {
                auto_mod.target_HP = referee_data.blue_base_HP;
            }
            else if(referee_data.robot_id == 108)
            {
                auto_mod.target_HP = referee_data.red_base_HP;
            }
        }

        if(rc_tw_turn.turnvalue == TW_UP)
        {
            //�л�����ģʽ������ģʽ��ģ�����ģʽ
            rc_tw_turn.turnvalue = TW_IDLE;
            auto_mod.ctrl_mode = !auto_mod.ctrl_mode;
        }
        
        if(auto_mod.ctrl_mode == 0)
        {
            //����ģʽ���Զ����ţ����Ŵ�1�����
//������α�ע�͵��Ĵ����Ǿ�������������bug�ģ����Ǳ���һ��ʼ�����ţ�����Ƥ����Ľϴ�
//            if(auto_mod.state == 0)
//            {
//                if(referee_data.game_progress == 4)
//                {
//                    auto_mod.state = 1;
//                    auto_mod.timetick = 0;
//                    load.state = 1;
//                }
//            }
//            else if(auto_mod.state == 1)
//            {
//                if(    load.state == 9 \
//                    && referee_data.dart_launch_opening_status == 0 \
//                    && auto_mod.gate_on_time > 1000 \
//                    && referee_data.game_progress == 4 \
//                    )
//                {
//                    load.shoot_cmd = 1;
//                    auto_mod.timetick = 0;
//                    if(load.launch_num == 4)
//                    {
//                        auto_mod.state = 3;
//                    }
//                    else
//                    {
//                        auto_mod.state = 2;
//                    }
//                }
//            }
//            else if(auto_mod.state == 2)
//            {
//                //��ʱ1�������
//                if(auto_mod.timetick < 1000)
//                {
//                    auto_mod.timetick++;
//                }
//                else
//                {
//                    auto_mod.state = 1;
//                    auto_mod.timetick = 0;
//                    if(load.state == 0)load.state = 1;
//                }
//            }
            
            
//������δ������ڲ����ɹر�תΪ���ڴ�ʱ�����ţ�����Ƥ����Ľ�С����û�в��Թ�
            if(auto_mod.state == 0)
            {
                if(referee_data.game_progress == 4 && auto_mod.gate_on_flag == 1)
                {
                    auto_mod.gate_on_flag = 0;
                    auto_mod.state = 1;
                    auto_mod.timetick = 0;
                    load.state = 1;
                }
            }
            else if(auto_mod.state == 1)
            {
                if(    load.state == 9 \
                    && referee_data.dart_launch_opening_status == 0 \
                    && auto_mod.gate_on_time > 1000 \
                    && referee_data.game_progress == 4 \
                    )
                {
                    load.shoot_cmd = 1;
                    auto_mod.timetick = 0;
                    if(load.launch_num == 1 || load.launch_num == 3)
                    {
                        auto_mod.state = 2;
                    }
                    else if(load.launch_num == 2)
                    {
                        auto_mod.state = 0;
                    }
                    else
                    {
                        auto_mod.state = 3;
                    }
                }
            }
            else if(auto_mod.state == 2)
            {
                //��ʱ1�������
                if(auto_mod.timetick < 1000)
                {
                    auto_mod.timetick++;
                }
                else
                {
                    auto_mod.state = 1;
                    auto_mod.timetick = 0;
                    if(load.state == 0)load.state = 1;
                }
            }
        }
        else if(auto_mod.ctrl_mode == 1)
        {
            //ģ�����ģʽ������һ�β��������Զ��ٴ����Ų�����
            if(auto_mod.state == 0)
            {
                if(rc_sw_turn[SW2].turnvalue == MID_DOWN)
                {
                    rc_sw_turn[SW2].turnvalue = IDLE;
                    auto_mod.state = 1;
                    auto_mod.timetick = 0;
                    load.state = 1;
                }
            }
            else if(auto_mod.state == 1)
            {
                if(    load.state == 9 \
                    && rc_sw_turn[SW2].turnvalue == MID_UP)
                {
                    load.shoot_cmd = 1;
                    auto_mod.timetick = 0;
                    auto_mod.state = 2;
                }
            }
            else if(auto_mod.state == 2)
            {
                if(auto_mod.timetick < 1000)
                {
                    auto_mod.timetick++;
                }
                else
                {
                    auto_mod.state = 3;
                    auto_mod.timetick = 0;
                    if(load.state == 0)load.state = 1;
                }
            }
            else if(auto_mod.state == 3)
            {
                if(load.state == 9)
                {
                    load.shoot_cmd = 1;
                    auto_mod.timetick = 0;
                    auto_mod.state = 0;
                }
            }
        }
        
        if(load.fail_flag == 0)
        {
            //�ƶ�pitch�ᡢyaw��ͼ�����̨
            motor[YAW_M].pid_a->sw = 1;
            motor[YAW_M].pid_s->sw = 1;
            motor[YAW_M].pid_a->set += -rc_sensor_info.ch0 * motor[YAW_M].res->direction / 3;
            motor[PITCH_M].pid_a->sw = 1;
            motor[PITCH_M].pid_s->sw = 1;
            motor[PITCH_M].pid_a->set += rc_sensor_info.ch1 * motor[YAW_M].res->direction / 5;
//            motor[GIMBAL].pid_a->sw = 1;
//            motor[GIMBAL].pid_s->sw = 1;
//            motor[GIMBAL].pid_a->set += (float)rc_sensor_info.ch3 * motor[GIMBAL].res->direction / 150;            
        }
    }
}

void flash_solve()
{
    if(flash_sol.shoot_flag == 1)
    {
        //��¼��������
        flash_sol.state = 1;
        flash_sol.shoot_flag = 0;
        flash_sol.target = referee_data.dart_attack_target;
        flash_sol.target_HP = auto_mod.target_HP;
        flash_info.frame_header = 0xA5;
        flash_info.launch_num = load.launch_num;
        flash_info.pitch = motor[PITCH_M].pid_a->set;
        flash_info.yaw = motor[YAW_M].pid_a->set;
        flash_info.gimbal = motor[GIMBAL].pid_a->set;
        if(mode == AUTO)
        {
            flash_info.target = referee_data.dart_attack_target;
        }
        else if(mode == HAND)
        {
            flash_info.target = 2;
        }
    }
    
    if(flash_sol.state == 1)
    {
        if(flash_sol.timetick < 5000)
        {
            flash_sol.timetick++;
        }
        else
        {
            //�ж��Ƿ����Ŀ��
            if(mode == AUTO)
            {
                if(    auto_mod.target_HP + 750 <= flash_sol.target_HP \
                    || (referee_data.dart_attack_target == 1 && flash_sol.target == 0))
                {
                    flash_info.hit = 1;
                }
                else
                {
                    flash_info.hit = 0;
                }
            }
            else if(mode == HAND && rc_sensor_info.s2 == RC_SW_MID)
            {
                flash_info.hit = 1;
            }
            else
            {
                flash_info.hit = 0;
            }
            
            flash_sol.timetick = 0;
            flash_sol.state = 0;
            Flash_Write();
        }
    }
    
    if(rc_tw_turn.turnvalue == TW_DOWN)
    {
        //���²�ָ�ָֻ����ϴμ�¼��λ������
        rc_tw_turn.turnvalue = TW_IDLE;
        motor[PITCH_M].pid_a->set = ((flash_info_t*)(SECTOR_11_ADDRESS + (flash_cmd.flash_idle_offset - 1)*sizeof(flash_info_t)))->pitch;
        motor[YAW_M].pid_a->set = ((flash_info_t*)(SECTOR_11_ADDRESS + (flash_cmd.flash_idle_offset - 1)*sizeof(flash_info_t)))->yaw;
        motor[GIMBAL].pid_a->set = ((flash_info_t*)(SECTOR_11_ADDRESS + (flash_cmd.flash_idle_offset - 1)*sizeof(flash_info_t)))->gimbal;
    }
}

void mode_solve()
{
    static uint8_t online_flag = 0;
    static uint8_t mode_flag = 0;
    if(rc_sensor.work_state == DEV_OFFLINE)
    {
        online_flag = 0;
    }
    else if(online_flag == 0)
    {
        online_flag = 1;
    }
    else if(online_flag == 1)
    {
        online_flag = 2;
        for(uint8_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
        {
            motor[cnt].pid_s->sw = 1;
            motor[cnt].pid_a->sw = 1;
            motor[cnt].pid_a->set = motor[cnt].info->angle_sum;
        }
    }
    if(mode == POS)
    {
        pos_mode();
        if(rc_sensor.info->s1 == RC_SW_DOWN)
        {
            mode = HAND;
            mode_flag = 1;
        }
        else if(rc_sensor.info->s1 == RC_SW_UP)
        {
            mode = AUTO;
            mode_flag = 1;
        }
    }
    else if(mode == HAND)
    {
        hand_mode();
        if(rc_sensor.info->s1 == RC_SW_MID)
        {
            mode = POS;
            mode_flag = 1;
        }
        else if(rc_sensor.info->s1 == RC_SW_UP)
        {
            mode = AUTO;
            mode_flag = 1;
        }
    }
    else if(mode == AUTO)
    {
        auto_mode();
        if(rc_sensor.info->s1 == RC_SW_MID)
        {
            mode = POS;
            mode_flag = 1;
        }
        else if(rc_sensor.info->s1 == RC_SW_DOWN)
        {
            mode = HAND;
            mode_flag = 1;
        }
    }
    auto_load();
    flash_solve();
    if(mode_flag == 1)
    {
        //�л�ģʽʱ��λһ��ң�������ݲ����õ��Ŀ��λ��Ϊ��ǰλ��
        mode_flag = 0;
        ch0_flag.max = 0;
        ch0_flag.min = 0;
        ch0_flag.mid = 0;
        ch1_flag.max = 0;
        ch1_flag.min = 0;
        ch1_flag.mid = 0;
        ch2_flag.max = 0;
        ch2_flag.min = 0;
        ch2_flag.mid = 0;
        ch3_flag.max = 0;
        ch3_flag.min = 0;
        ch3_flag.mid = 0;
        rc_sw_turn[0].turnvalue = IDLE;
        rc_sw_turn[1].turnvalue = IDLE;
        rc_tw_turn.turnvalue = TW_IDLE;
        for(uint8_t cnt = BELT_MID; cnt < MOTOR_CNT; cnt++)
        {
            motor[cnt].pid_s->sw = 1;
            motor[cnt].pid_a->sw = 1;
            motor[cnt].pid_a->set = motor[cnt].info->angle_sum;
        }
    }
}

//ң���������̵����������̵���
void led_green_set()
{
    if(rc_sensor.work_state == DEV_ONLINE)
    {
        LED_GREEN_ON();
    }
    else if(rc_sensor.work_state == DEV_OFFLINE)
    {
        LED_GREEN_OFF();
    }
}

//ϵͳ��������ʱ��������˸
void led_blue_set()
{
    static uint16_t timetick = 0;
    if(timetick < 500)
    {
        timetick ++;  //�ú�������1msһ�ε������У�timetickÿms��1
    }
    else
    {
        timetick = 0;
        LED_BLUE_TOGGLE();
    }
}

void led_program()
{
    led_green_set();
    led_blue_set();
    Flash_Read();
    Flash_Erase();
    Flash_Empty_Check();
    Flash_usart_send();
}








