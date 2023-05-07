/**
 * @file        fricwheel.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        3-November-2020
 * @brief       Friction Wheel Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "fricwheel.h"

#include "rp_math.h"
#include "drv_io.h"
#include "judge.h"

/* Private macro -------------------------------------------------------------*/
#define FRIC_RAMP_STEP	2
#define FRICADAPTMAX    4

/* Private function prototypes -----------------------------------------------*/
static bool FRICWHEEL_IfOpen(void);
static bool FRICWHEEL_IfReady(void);
void FRICWHEEL_Init(void);
void FRICWHEEL_SelfProtect(void);
bool FRICWHEEL_Adjust(uint8_t *flg);
void FRICWHEEL_Adapt(void);
void FRICWHEEL_Ctrl(void);
void FRICWHEEL_Test(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Ħ���ֵ����������
static drv_pwm_t	*fric_drv[FRIC_MOTOR_CNT];
// ң�ذ�����Ϣ
static button_t         MOUSE_L;
static button_flip_t    MOUSE_L_TRIG;     
static switch_t         SW1;
static switch_flip_t    SW1_TRIG;
static switch_t         SW2;
static switch_flip_t    SW2_TRIG;

/* Exported variables --------------------------------------------------------*/
// Ħ���ֵ��б�¿�����
fric_ramp_t 	fric_ramp = {
	.speed.target = 0,
	.speed.measure = 0, 
};

// Ħ����ģ�������
fric_ctrl_t		fric_ctrl = {
	.fric = &fric_ramp,
};

// Ħ����ģ�鴫����
fric_dev_t		fric_dev = {
	.fric_motor[FRIC_L] = &fric_motor[FRIC_L],
	.fric_motor[FRIC_R] = &fric_motor[FRIC_R],
	.rc_sensor = &rc_sensor,
    .judge_sensor = &judge_sensor
};

// Ħ����ģ����Ϣ
fric_info_t		fric_info = {
	.remote_mode = RC,
	.local_mode = FRIC_MODE_NORMAL,
	.state = FRIC_STATE_OFF,
	.level = FRIC_LEVEL_HIGH,
};

// Ħ����pwm���ձ�
int16_t fric_pwm_table[] = {
	[FRIC_LEVEL_LOW] = 480,
	[FRIC_LEVEL_MID] = 525,
	[FRIC_LEVEL_HIGH] = 640,    //785 885 885
};

// Ħ����ģ��
fric_t	fric = {
	.controller = &fric_ctrl,
	.dev = &fric_dev,
	.info = &fric_info,
	.init = FRICWHEEL_Init,
	.ctrl = FRICWHEEL_Ctrl,
	.self_protect = FRICWHEEL_SelfProtect,
    .adjust = FRICWHEEL_Adjust,
    .adapt = FRICWHEEL_Adapt,
    .test = FRICWHEEL_Test,
	.if_open = FRICWHEEL_IfOpen,
	.if_ready = FRICWHEEL_IfReady,
};

/* Private functions ---------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
static void FRICWHEEL_Stop(fric_ramp_t *fric_ramp)
{
	fric_ramp->speed.target = 0;
	fric_ramp->speed.measure = RampInt(0, fric_ramp->speed.measure, FRIC_RAMP_STEP);
										   
	fric_drv[FRIC_L]->output(fric_drv[FRIC_L], 0);
	fric_drv[FRIC_R]->output(fric_drv[FRIC_R], 0);
}

#define MAX_PWM_VAL 800
static bool FRICWHEEL_Adjust(uint8_t *flg)
{
	static uint16_t standard_cnt = 0;   //У׼��ʱ����
	static uint16_t cnt = 0;    //��������ʱ����
	
	if(*flg)
	{

		if(cnt < 4000)  //�ȸ�������4s
		{
            fric_drv[FRIC_L]->output(fric_drv[FRIC_L], MAX_PWM_VAL);
            fric_drv[FRIC_R]->output(fric_drv[FRIC_R], MAX_PWM_VAL);
		}
		else           //ÿ1ms���ν��ͣ�ֱ����С���
		{
			cnt = 4001;
			
            fric_drv[FRIC_L]->output(fric_drv[FRIC_L], MAX_PWM_VAL - standard_cnt);
            fric_drv[FRIC_R]->output(fric_drv[FRIC_R], MAX_PWM_VAL - standard_cnt);
			
			standard_cnt++;
			if(standard_cnt > MAX_PWM_VAL)   //�ﵽ��С���
			{
				standard_cnt = 0;
				cnt = 0;
				*flg = 0;
				return true;
			}
		}
		
		cnt++;
	}
	
	return false;
}

/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/
static bool FRICWHEEL_IfOpen(void)
{
	if(fric_info.state == FRIC_STATE_ON)
		return true;
	else if(fric_info.state == FRIC_STATE_OFF)
		return false;
	return false;
}

static bool FRICWHEEL_IfReady(void)
{
	if(fric_info.state == FRIC_STATE_OFF)
	   return false;
	return (abs(fric_ramp.speed.target - fric_ramp.speed.measure)<=FRICADAPTMAX*5)?true:false;
}

void FRICWHEEL_GetSysInfo(void)
{
	if(sys.remote_mode == RC)
		fric_info.remote_mode = RC;
	else if(sys.remote_mode == KEY)
		fric_info.remote_mode = KEY;
}
	
void FRICWHEEL_GetJudgeInfo(void)
{    
//    static uint16_t pwm_delay_cnt = 0;
//    judge_sensor_t *judge_sen = fric_dev.judge_sensor;
    fric_info.level = FRIC_LEVEL_HIGH;
    
//    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
//        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
//        (judge_sen->info->aerial_robot_energy.attack_time != 0)) {
//        // �ȴ�3s
//        if(pwm_delay_cnt < 1250) {
//            pwm_delay_cnt++;
//            fric_info.state = FRIC_STATE_OFF;
//        } else {
//            fric_info.state = FRIC_STATE_ON;
//        }
//    } else {
//        fric_info.state = FRIC_STATE_OFF;
//        fric_ramp.speed.target = 0;
//        fric_ramp.speed.measure = 0;
//        pwm_delay_cnt = 0;
//    }
    
    // ���˻���������Ϊ30m/s
//    if(judge_info.game_robot_status.mains_power_shooter_output) {
//        fric_info.local_mode = FRIC_MODE_NORMAL;
//        fric_info.state = FRIC_STATE_ON;
//    } else {
//        fric_info.local_mode = FRIC_MODE_BAN;
//        fric_info.state = FRIC_STATE_OFF;
//    }
}

void FRICWHEEL_GetRcInfo(void)
{
    rc_sensor_t *rc_sen = fric_dev.rc_sensor;
    
    SW1_TRIG = rc_sen->copy_switch(&SW1, &rc_sen->info->SW1);
    SW2_TRIG = rc_sen->copy_switch(&SW2, &rc_sen->info->SW2);
    MOUSE_L_TRIG = rc_sen->copy_button(&MOUSE_L, &rc_sen->info->MOUSE_L);
}

/* Ӧ�ò� --------------------------------------------------------------------*/
bool fric_ctrl_enable;
void RC_SetFricState(void)
{
    static uint16_t pwm_delay_cnt = 0;
    judge_sensor_t *judge_sen = fric_dev.judge_sensor;

    /* Ħ�����Ѿ����� */
//    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
//        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
//        (judge_sen->info->aerial_robot_energy.attack_time != 0)) 
    if(1)
    {
        /* �ӳ�δ��� */
        if(pwm_delay_cnt < 1000) {
            pwm_delay_cnt++;
            // ��ֹ����Ħ����(��ֹ����Ħ����У׼����)
            fric_ctrl_enable = false;
        } 
        /* �ӳ���� */
        else {
            // ��������Ħ����
            fric_ctrl_enable = true;
        }
    }
    /* Ħ����δ����(��ֹ����) */
    else {
        pwm_delay_cnt = 0;
        fric_ctrl_enable = false;
    }
    
    if(fric_ctrl_enable) {
        if((SW2.state == RC_SW_DOWN) && (SW1_TRIG == SW_MID_TO_UP))
        {
            if(fric_info.state == FRIC_STATE_OFF) {
                fric_info.state = FRIC_STATE_ON;
                LASER_ON();
            }
            else if(fric_info.state == FRIC_STATE_ON) {
                fric_info.state = FRIC_STATE_OFF;
//                LASER_OFF();
            }
        }
    } 
    else {
        // Ѹ�ٸ�λĦ����pwm���ֵ(��ֹ����˲�䴥��У׼����)
//        LASER_OFF();
        fric_info.state = FRIC_STATE_OFF;
        fric_ramp.speed.target = 0;
        fric_ramp.speed.measure = 0;   
    }
}

void KEY_SetFricState(void)
{
    static uint16_t pwm_delay_cnt = 0;

    // ���������п���֧Ԯ֮���Զ�����Ħ����
    judge_sensor_t *judge_sen = fric_dev.judge_sensor;
    
    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
        (judge_sen->info->aerial_robot_energy.attack_time != 0)) {
        // �ȴ�2.5s
        if(pwm_delay_cnt < 1250) {
            pwm_delay_cnt++;
            fric_info.state = FRIC_STATE_OFF;
        } else {
            fric_info.state = FRIC_STATE_ON;
        }
    } else {
        fric_info.state = FRIC_STATE_OFF;
        fric_ramp.speed.target = 0;
        fric_ramp.speed.measure = 0;
        pwm_delay_cnt = 0;
    }
    
    // ���ԣ�����������Ħ����
//    judge_sensor_t *judge_sen = fric_dev.judge_sensor;
//    
//    if((judge_sen->info->game_robot_status.mains_power_shooter_output) ||
//        (judge_sen->info->bullet_remaining.bullet_remaining_num_17mm != 0) ||
//        (judge_sen->info->aerial_robot_energy.attack_time != 0)) 
//    {
//        // �ӳ�����Ħ����(��ֹ����˲�䴥��У׼����)
//        if(pwm_delay_cnt < 1000) {
//            pwm_delay_cnt++;
//            fric_info.state = FRIC_STATE_OFF;
//        }
//        else {
//            if(MOUSE_L.state == PRESS) {
//                fric_info.state = FRIC_STATE_ON;
//                LASER_ON();
//            }
//        }
//    }
//    else 
//    {
//        // Ѹ�ٸ�λĦ����pwm���ֵ(��ֹ����˲�䴥��У׼����)
//        pwm_delay_cnt = 0;
//        LASER_OFF();
//        fric_info.state = FRIC_STATE_OFF;
//        fric_ramp.speed.target = 0;
//        fric_ramp.speed.measure = 0;
//    }
}

typedef struct{
    float last_speed;
    uint32_t last_shoot_time;
    uint32_t time_err[5];   //���δ򵯵�ʱ�������жϷ���״̬
    uint8_t num;            //�������
    int8_t single_err;      //������pwmƫ����
    int8_t burst_err;       //������pwmƫ����
    uint8_t single_i;        //������
    uint8_t burst_i;         //������
    uint8_t shoot_state;    //����״̬��0:������1:����
    uint32_t idel_time;     //�����ϴδ򵯵�ʱ�䣬�����жϷ���״̬
    float target_speed;     //Ŀ���ٶ�
}fric_adapt_t;

fric_adapt_t fa = {
    .single_err = 0,
    .burst_err = 0,
    .single_i = 2,
    .burst_i = 2,
    .target_speed = 24.5f,
};

void FRICWHEEL_Adapt(void)
{
    judge_info_t* judge_info = fric.dev->judge_sensor->info;
    if(fa.last_speed != judge_info->shoot_data.bullet_speed)
    {
//        fa.idel_time = 0;
        fa.last_speed = judge_info->shoot_data.bullet_speed;
        fa.time_err[fa.num] = HAL_GetTick() - fa.last_shoot_time;
        fa.last_shoot_time = HAL_GetTick();
//        fa.num = (fa.num + 1) % 5;
//        if(fa.time_err[0] < 150 && fa.time_err[1] < 150 && fa.time_err[2] < 150 \
//           && fa.time_err[3] < 150 && fa.time_err[4] < 150){
//            fa.shoot_state = 1;
//            fa.burst_err += constrain(fa.burst_i * (fa.target_speed - judge_info->shoot_data.bullet_speed), -FRICADAPTMAX, FRICADAPTMAX);
//            if(fa.burst_err > 40){
//                fa.burst_err = 40;
//            }
//            else if(fa.burst_err < -40){
//                fa.burst_err = -40;
//            }
//        }
//        else{
//            fa.shoot_state = 0;
            fa.single_err += constrain(fa.single_i * (fa.target_speed - judge_info->shoot_data.bullet_speed), -FRICADAPTMAX, FRICADAPTMAX);
            if(fa.single_err > 40){
                fa.single_err = 40;
            }
            else if(fa.single_err < -40){
                fa.single_err = -40;
            }
//        }
    }
//    else{
//        fa.idel_time++;
//        if(fa.idel_time > 400){
//            fa.shoot_state = 0;
//        }
//    }
}

/* ����� --------------------------------------------------------------------*/
void FRICWHEEL_Init(void)
{
	// ����Ħ���ֵ������ı�������
	fric_drv[FRIC_L] = fric_dev.fric_motor[FRIC_L]->driver;
	fric_drv[FRIC_R] = fric_dev.fric_motor[FRIC_R]->driver;
}

void FRICWHEEL_GetInfo(void)
{
	// ��ȡϵͳ��Ϣ
	FRICWHEEL_GetSysInfo();
	// ��ȡ����ϵͳ��Ϣ
	FRICWHEEL_GetJudgeInfo();	
	// ����ң��������
	FRICWHEEL_GetRcInfo();
}

void FRICWHEEL_SelfProtect(void)
{
	fric_info.state = FRIC_STATE_OFF;
	FRICWHEEL_Stop(&fric_ramp);
	FRICWHEEL_GetInfo();
}

void FRICWHEEL_RampCtrl(void)
{
    //��������Ӧ
    fric.adapt();
    
	// �������ٵȼ�����Ħ����ת��
	if(fric_info.state == FRIC_STATE_ON)
		fric_ramp.speed.target = fric_pwm_table[fric_info.level] + (fa.shoot_state ? fa.burst_err : fa.single_err);
	else
		fric_ramp.speed.target = 0;
    
	// б�¼���
    fric_ramp.speed.measure = RampInt(fric_ramp.speed.target, fric_ramp.speed.measure, FRIC_RAMP_STEP);
	fric_drv[FRIC_L]->output(fric_drv[FRIC_L], fric_ramp.speed.measure);
	fric_drv[FRIC_R]->output(fric_drv[FRIC_R], fric_ramp.speed.measure);
}

void FRICWHEEL_RcCtrl(void)
{
	RC_SetFricState();
}

void FRICWHEEL_KeyCtrl(void)
{
	KEY_SetFricState();
}

void FRICWHEEL_Ctrl(void)
{
	/*----��Ϣ����----*/
	FRICWHEEL_GetInfo();
	/*----�����޸�----*/
	if(fric_info.remote_mode == RC) {
		FRICWHEEL_RcCtrl();
	}
	else if(fric_info.remote_mode == KEY) {
		FRICWHEEL_KeyCtrl();
	}
	
	/*----�������----*/
    
	FRICWHEEL_RampCtrl();
}

void FRICWHEEL_Test(void)
{
    FRICWHEEL_GetInfo();
    
    FRICWHEEL_KeyCtrl();
    /*----�������----*/
	FRICWHEEL_RampCtrl();
}
