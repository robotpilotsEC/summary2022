/**
 * @file        system_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Decision Center.
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"

#include "cmsis_os.h"
#include "rc_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
flag_t flag = {
	.gimbal = {
		.reset_start = true,
		.reset_ok = false,
        .prepare_auto = false
	},
};

system_t sys = {
	.remote_mode = RC,
    .co_pid_mode = MECH,
	.state = SYS_STATE_RCLOST,
	.mode = SYS_MODE_NORMAL,
    .act = SYS_ACT_NORMAL
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	����ң����������Ϣ(������״̬������ң����Ϣ)
 */
static void rc_update_info(void)
{
    rc_sensor_info_t *rc_info = rc_sensor.info;
    
	if(rc_sensor.work_state == DEV_OFFLINE) {
        rc_sensor.reset(&rc_sensor);
    }
    
    rc_sensor.update_button(&rc_info->W, (rc_info->key_v & KEY_PRESSED_OFFSET_W) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->S, (rc_info->key_v & KEY_PRESSED_OFFSET_S) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->A, (rc_info->key_v & KEY_PRESSED_OFFSET_A) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->D, (rc_info->key_v & KEY_PRESSED_OFFSET_D) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->SHIFT, (rc_info->key_v & KEY_PRESSED_OFFSET_SHIFT) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->CTRL, (rc_info->key_v & KEY_PRESSED_OFFSET_CTRL) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->Q, (rc_info->key_v & KEY_PRESSED_OFFSET_Q) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->E, (rc_info->key_v & KEY_PRESSED_OFFSET_E) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->R, (rc_info->key_v & KEY_PRESSED_OFFSET_R) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->F, (rc_info->key_v & KEY_PRESSED_OFFSET_F) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->G, (rc_info->key_v & KEY_PRESSED_OFFSET_G) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->Z, (rc_info->key_v & KEY_PRESSED_OFFSET_Z) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->X, (rc_info->key_v & KEY_PRESSED_OFFSET_X) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->C, (rc_info->key_v & KEY_PRESSED_OFFSET_C) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->V, (rc_info->key_v & KEY_PRESSED_OFFSET_V) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->B, (rc_info->key_v & KEY_PRESSED_OFFSET_B) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->MOUSE_L, (rc_info->mouse_btn_l) ? PRESS : RELEASE);
    rc_sensor.update_button(&rc_info->MOUSE_R, (rc_info->mouse_btn_r) ? PRESS : RELEASE);   

    rc_sensor.update_switch(&rc_info->SW1, (switch_state_t)rc_info->s1);
    rc_sensor.update_switch(&rc_info->SW2, (switch_state_t)rc_info->s2);
    
    rc_sensor.update_stick(&rc_info->CH0, rc_info->ch0);
    rc_sensor.update_stick(&rc_info->CH1, rc_info->ch1);
    rc_sensor.update_stick(&rc_info->CH2, rc_info->ch2);
    rc_sensor.update_stick(&rc_info->CH3, rc_info->ch3);
    
    rc_sensor.update_mouse(&rc_info->MOUSE_VX, rc_info->mouse_vx);
    rc_sensor.update_mouse(&rc_info->MOUSE_VY, rc_info->mouse_vy);
    rc_sensor.update_mouse(&rc_info->MOUSE_VZ, rc_info->mouse_vz);
}

/**
 *	@brief	����ң�����л����Ʒ�ʽ
 */
uint8_t twflag = 0;
static void system_ctrl_mode_switch(void)
{
    rc_sensor_info_t *rc_info = rc_sensor.info;
    
    switch(rc_info->SW2.state)
    {
        case RC_SW_UP:
            /* ң�ػ�еģʽ -> ����ģʽ */
            if(rc_info->SW2.flip == SW_MID_TO_UP) 
            {
                // ���л���ȥ��ʱ������Ϊ����ģʽ
                sys.mode = SYS_MODE_NORMAL;	
                sys.act = SYS_ACT_NORMAL;
                sys.co_pid_mode = GYRO;
            }
            sys.remote_mode = KEY;
            break;
        
        case RC_SW_MID:
            /* ����ģʽ -> ң�ػ�еģʽ */
            if(rc_info->SW2.flip == SW_UP_TO_MID) {} 
            /* ң��������ģʽ -> ң�ػ�еģʽ */
            else if(rc_info->SW2.flip == SW_DOWN_TO_MID) {}
                
            // ң�ؿ���Ĭ��ϵͳΪ����ģʽ
//            sys.mode = SYS_MODE_NORMAL; 
//            sys.act = SYS_ACT_NORMAL;
            sys.remote_mode = RC;
            sys.co_pid_mode = MECH;
            break;
        
        case RC_SW_DOWN:
            /* ң�ػ�еģʽ -> ң��������ģʽ */
            if(rc_info->SW2.flip == SW_MID_TO_DOWN) {}
                
            /* ң�ؿ���Ĭ��ϵͳΪ������Ϊ */
//            sys.mode = SYS_MODE_NORMAL;
//            sys.act = SYS_ACT_NORMAL;
            sys.remote_mode = RC;
            sys.co_pid_mode = GYRO;
            break;
        
        default:
            break;
    }
    
    if(rc_info->SW2.state == RC_SW_MID || rc_info->SW2.state == RC_SW_DOWN){
        if(twflag == 0){
            sys.mode = SYS_MODE_NORMAL;
            sys.act = SYS_ACT_NORMAL;
            if(rc_info->thumbwheel > 500){
                twflag = 1;
                sys.mode = SYS_MODE_AUTO;
                sys.act = SYS_ACT_AIM_ARMY;
            }
        }
        else if(twflag == 1){
            if(rc_info->thumbwheel == 0)
                twflag = 2;
        }
        else if(twflag == 2){
            sys.mode = SYS_MODE_AUTO;
            sys.act = SYS_ACT_AIM_ARMY;
            if(rc_info->thumbwheel > 500){
                twflag = 3;
            }
        }
        else if(twflag == 3){
            if(rc_info->thumbwheel == 0)
                twflag = 0;
        }
        else{
            twflag = 0;
        }
    }
}

/**
 *	@brief	����ң�����л�ϵͳ��Ϊ
 */
static void system_mode_act_switch(void)
{
    rc_sensor_info_t *rc_info = rc_sensor.info;
    
    if(rc_info->SW2.state == RC_SW_UP){
        if(rc_info->MOUSE_R.state == PRESS) {
            sys.mode = SYS_MODE_AUTO;
            if(rc_info->SHIFT.state == PRESS) {
                sys.act = SYS_ACT_AIM_ARMY;
            } else {
                sys.act = SYS_ACT_AIM_OUTPOST;
            }
        } else {
            sys.mode = SYS_MODE_NORMAL;
            sys.act = SYS_ACT_NORMAL;
        }
    }
}

static void system_state_machine(void)
{
	// ���Ʒ�ʽ�л�
	system_ctrl_mode_switch();
	// ϵͳģʽ�л�(����ģʽ�²������л�)
	if(sys.remote_mode == KEY)
		system_mode_act_switch();
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	ϵͳ��������
 */
void StartSystemTask(void const * argument)
{
    static uint8_t reconnect_cnt = 0;
    
	for(;;)
	{
		portENTER_CRITICAL();
		
		// ����ң����Ϣ
		rc_update_info();
		
		/* ң������ */
		if(rc_sensor.work_state == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
            reconnect_cnt = 0;
		} 
		/* ң������ */
		else if(rc_sensor.work_state == DEV_ONLINE)
		{
			/* ң������ */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* ʧ���ָ� */
				if(sys.state == SYS_STATE_RCLOST) 
				{
                    if(reconnect_cnt < 1) {
                        reconnect_cnt++;
                    } else {
                        // ���ڴ˴�ͬ����̨��λ��־λ	
                        flag.gimbal.reset_start = true;
                        flag.gimbal.reset_ok = false;
                        // ϵͳ������λ
                        sys.remote_mode = RC;
                        sys.co_pid_mode = MECH;
                        sys.state = SYS_STATE_NORMAL;
                        sys.mode = SYS_MODE_NORMAL;
                        sys.act = SYS_ACT_NORMAL;
                    }
				}
			}
			/* ң�ش��� */
			else if(rc_sensor.errno == DEV_DATA_ERR) {
				sys.state = SYS_STATE_RCERR;
				//reset CPU
				__set_FAULTMASK(1);
				NVIC_SystemReset();
			} else {
				sys.state = SYS_STATE_WRONG;
				//reset CPU
				__set_FAULTMASK(1);
				NVIC_SystemReset();
			}
		}
		
        if(sys.state == SYS_STATE_NORMAL) {
            // ���ڴ˴��ȴ���̨��λ��������л�״̬
            if(flag.gimbal.reset_ok == true)
                system_state_machine();
        }
        
		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
