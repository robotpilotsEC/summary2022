/**
 * @file        rc_sensor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Device Rc.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"

#include "rp_math.h"
#include "drv_haltick.h"
#include "rc_potocol.h"
#include "device.h"

extern void rc_sensor_init(rc_sensor_t *rc_sen);
extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t *rc_sen);
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen);
static bool rc_if_channel_reset(rc_sensor_t *rc_sen);
static void rc_reset(rc_sensor_t *rc_sen);
static button_flip_t rc_update_button(button_t *btn, button_state_t new_btn_state);
static switch_flip_t rc_update_switch(switch_t *sw, switch_state_t new_sw_state);
static void rc_update_stick(stick_t *stick, int16_t val);
static void rc_update_mouse(mouse_speed_t *mouse_speed, int16_t val);
static button_flip_t rc_copy_button(button_t *dst, button_t *src);
static switch_flip_t rc_copy_switch(switch_t *dst, switch_t *src);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 遥控器驱动
drv_uart_t	rc_sensor_driver = {
	.type = DRV_UART2,
	.tx_byte = NULL,
};

// 遥控器信息
rc_sensor_info_t 	rc_sensor_info = {
	.offline_max_cnt = 60,
};

// 遥控器传感器
rc_sensor_t	rc_sensor = {
	.info = &rc_sensor_info,
	.init = rc_sensor_init,
	.update = rc_sensor_update,
	.check = rc_sensor_check,
	.heart_beat = rc_sensor_heart_beat,
	.work_state = DEV_OFFLINE,
	.id = DEV_ID_RC,
    .if_channel_reset = rc_if_channel_reset,
    .reset = rc_reset,
    .update_button = rc_update_button,
    .update_switch = rc_update_switch,
    .update_stick = rc_update_stick,
    .update_mouse = rc_update_mouse,
    .copy_button = rc_copy_button,
    .copy_switch = rc_copy_switch
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	遥控器数据检查
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
		rc_info->s1 = RC_SW_MID;
		rc_info->s2 = RC_SW_MID;
		rc_info->thumbwheel = 0;
	}
	else
	{
		rc_sen->errno = NONE_ERR;
	}
}

/**
 *	@brief	遥控器心跳包
 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)
{
	rc_sensor_info_t *rc_info = rc_sen->info;

	rc_info->offline_cnt++;
	if(rc_info->offline_cnt > rc_info->offline_max_cnt) {
		rc_info->offline_cnt = rc_info->offline_max_cnt;
		rc_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(rc_sen->work_state == DEV_OFFLINE)
			rc_sen->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/
static bool rc_if_channel_reset(rc_sensor_t *rc_sen)
{
    if(  (DeathZoom(rc_sen->info->ch0, 0, 50) == 0) && 
		 (DeathZoom(rc_sen->info->ch1, 0, 50) == 0) && 
		 (DeathZoom(rc_sen->info->ch2, 0, 50) == 0) && 
		 (DeathZoom(rc_sen->info->ch3, 0, 50) == 0)   )	
	{
		return true;
	}
	return false;
}

static void rc_reset(rc_sensor_t *rc_sen)
{
    // 通道值强行设置成中间值(不拨动摇杆的状态)
	rc_sen->info->ch0 = 0;
	rc_sen->info->ch1 = 0;
	rc_sen->info->ch2 = 0;
	rc_sen->info->ch3 = 0;
	// 左右开关选择强行设置成中间值状态
	rc_sen->info->s1 = RC_SW_MID;
	rc_sen->info->s2 = RC_SW_MID;
	// 鼠标
	rc_sen->info->mouse_vx = 0;
	rc_sen->info->mouse_vy = 0;
	rc_sen->info->mouse_vz = 0;
	rc_sen->info->mouse_btn_l = 0;
	rc_sen->info->mouse_btn_r = 0;
	// 键盘
	rc_sen->info->key_v = 0;
	// 左拨轮
	rc_sen->info->thumbwheel = 0;
}

static button_flip_t rc_update_button(button_t *btn, button_state_t new_btn_state)
{
    button_flip_t trig = BTN_NONE_FLIP;
    
    /* 按键状态跳变 */
    if(new_btn_state != btn->state) {
        btn->hold_time = 0;
        /* RELEASE -> PRESS */
        if(new_btn_state == PRESS) {
            trig = RELEASE_TO_PRESS;
        }
        /* PRESS -> RELEASE */
        else if(new_btn_state == RELEASE) {
            trig = PRESS_TO_RELEASE;
        }
    } 
    /* 按键状态保持 */
    else {
        btn->hold_time += millis() - btn->update_time;
    }
    
    btn->update_time = millis();
    btn->state = new_btn_state;
    btn->flip = trig;
    
    return trig;
}

static switch_flip_t  rc_update_switch(switch_t *sw, switch_state_t new_sw_state)
{
    switch_flip_t trig = SW_NONE_FLIP;
    
    /* 拨杆状态跳变 */
    if(new_sw_state != sw->state) {
        // 保持时间清零
        sw->hold_time = 0;
        if(sw->state == RC_SW_UP) {
            /* MID -> UP */
            if(new_sw_state == RC_SW_MID) {
                trig = SW_MID_TO_UP;
            }
        }
        else if(sw->state == RC_SW_MID) {
            /* UP -> MID */
            if(new_sw_state == RC_SW_UP) {
                trig = SW_UP_TO_MID;
            }
            /* DOWN -> MID */
            else if(new_sw_state == RC_SW_DOWN) {
                trig = SW_DOWN_TO_MID;
            }
        }
        else if(sw->state == RC_SW_DOWN) {
            /* MID -> DOWN */
            if(new_sw_state == RC_SW_MID) {
                trig = SW_MID_TO_DOWN;
            }
        }
    }
    /* 拨杆状态保持 */
    else {
        sw->hold_time += millis() - sw->update_time;
    }
    
    sw->update_time = millis();
    sw->state = new_sw_state;
    sw->flip = trig;
    
    return trig;
}

static void rc_update_stick(stick_t *stick, int16_t val)
{
    stick->val = val;
}

static void rc_update_mouse(mouse_speed_t *mouse_speed, int16_t val)
{
    mouse_speed->val = val;
}

static button_flip_t rc_copy_button(button_t *dst, button_t *src)
{
    button_flip_t trig = BTN_NONE_FLIP;
    
    /* 按键状态跳变 */
    if(dst->state != src->state) {
        /* RELEASE -> PRESS */
        if(src->state == PRESS) {
            trig = RELEASE_TO_PRESS;
        } 
        /* PRESS -> RELEASE */
        else if(src->state == RELEASE) {
            trig = PRESS_TO_RELEASE;
        }
    }
    dst->state = src->state;
    dst->hold_time = src->hold_time;
    
    return trig;    
}

static switch_flip_t rc_copy_switch(switch_t *dst, switch_t *src)
{
    switch_flip_t trig = SW_NONE_FLIP;
    
    /* 拨杆状态跳变 */
    if(dst->state != src->state) {
        if(src->state == RC_SW_UP) {
            /* MID -> UP */
            if(dst->state == RC_SW_MID) {
                trig = SW_MID_TO_UP;
            }
        }
        else if(src->state == RC_SW_MID) {
            /* UP -> MID */
            if(dst->state == RC_SW_UP) {
                trig = SW_UP_TO_MID;
            }
            /* DOWN -> MID */
            else if(dst->state == RC_SW_DOWN) {
                trig = SW_DOWN_TO_MID;
            }
        }
        else if(src->state == RC_SW_DOWN) {
            /* MID -> DOWN */
            if(dst->state == RC_SW_MID) {
                trig = SW_MID_TO_DOWN;
            }
        }
    }
    dst->state = src->state;
    dst->hold_time = src->hold_time;
    
    return trig;    
}
