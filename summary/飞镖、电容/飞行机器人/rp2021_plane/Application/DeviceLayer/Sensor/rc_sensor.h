#ifndef __RC_SENSOR_H
#define __RC_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "button_sensor.h"
/* Exported macro ------------------------------------------------------------*/
/* ----------------------- RC Channel Definition------------------------------*/

#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define	   RC_CH_VALUE_MAX       ((uint16_t)1684)
#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2)

/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* ----------------------- Function Definition-------------------------------- */
/* ң��ҡ��ͨ��ƫ��ֵ */
#define		RC_SW1_VALUE				(rc_sensor_info.s1)
#define		RC_SW2_VALUE				(rc_sensor_info.s2)
#define		RC_LEFT_CH_LR_VALUE			(rc_sensor_info.ch2)
#define		RC_LEFT_CH_UD_VALUE			(rc_sensor_info.ch3)
#define		RC_RIGH_CH_LR_VALUE			(rc_sensor_info.ch0)
#define		RC_RIGH_CH_UD_VALUE			(rc_sensor_info.ch1)
#define		RC_THUMB_WHEEL_VALUE		(rc_sensor_info.thumbwheel)

/* ���ң��������״̬ */
#define    IF_RC_SW1_UP      (rc_sensor_info.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_sensor_info.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_sensor_info.s1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_sensor_info.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_sensor_info.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_sensor_info.s2 == RC_SW_DOWN)

/* ��ȡ���������ƶ��ٶ� */
#define    MOUSE_X_MOVE_SPEED    (rc_sensor_info.mouse_vx)
#define    MOUSE_Y_MOVE_SPEED    (rc_sensor_info.mouse_vy)
#define    MOUSE_Z_MOVE_SPEED    (rc_sensor_info.mouse_vz)


/* �����갴��״̬ 
   ����Ϊ1��û����Ϊ0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_sensor_info.mouse_btn_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_sensor_info.mouse_btn_r == 1)


/* �����̰���״̬ 
   ����Ӧ���������£����߼����ʽ��ֵΪ1������Ϊ0 */
#define    IF_KEY_PRESSED         (  rc_sensor_info.key_v  )
#define    IF_KEY_PRESSED_W       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_sensor_info.key_v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

/* Exported types ------------------------------------------------------------*/
typedef button_info_t button_t;

typedef struct {
    button_t    W;
    button_t    S;
    button_t    A;
    button_t    D;
    button_t    SHIFT;
    button_t    CTRL;
    button_t    Q;
    button_t    E;
    button_t    R;
    button_t    F;
    button_t    G;
    button_t    Z;
    button_t    X;
    button_t    C;
    button_t    V;
    button_t    B;
    button_t    MOUSE_L;
    button_t    MOUSE_R;
} rc_buttons_t;

typedef enum {
    RC_SW_UP = 1,
    RC_SW_MID = 3,
    RC_SW_DOWN = 2
} switch_state_t;

typedef enum {
    SW_NONE_FLIP = 0,
    SW_MID_TO_UP = 1,  
    SW_MID_TO_DOWN = 2,
    SW_UP_TO_MID = 3,
    SW_DOWN_TO_MID = 4
} switch_flip_t;

typedef struct {
    switch_state_t  state;      // ��ǰ״̬
    switch_flip_t   flip;       // ��ת״̬
    uint32_t        update_time;// ����ʱ��(HALTICK: 1msΪ��λ)
    uint32_t        hold_time;  // ����ʱ��(HALTICK: 1msΪ��λ)
} switch_t;

typedef struct {
    switch_t    sw1;
    switch_t    sw2;
} rc_switchs_t;

typedef struct {
    int16_t     val;    
} stick_t;

typedef struct {
    stick_t     ch0;
    stick_t     ch1;
    stick_t     ch2;
    stick_t     ch3;
    stick_t     thumbwheel;
} rc_sticks_t;

typedef struct {
    int16_t     val;
} mouse_speed_t;

typedef struct {
    mouse_speed_t   vx;
    mouse_speed_t   vy;
    mouse_speed_t   vz;
} rc_mouse_speed_t;

typedef struct rc_sensor_info_struct {
    // raw data
	int16_t 	    ch0;
	int16_t 	    ch1;
	int16_t 	    ch2;
	int16_t 	    ch3;
	uint8_t  	    s1;
	uint8_t  	    s2;
	int16_t		    mouse_vx;
	int16_t 	    mouse_vy;
	int16_t 	    mouse_vz;
	uint8_t 	    mouse_btn_l;
	uint8_t 	    mouse_btn_r;
	uint16_t	    key_v;
	int16_t 	    thumbwheel;	
    // process data
    switch_t        SW1;
    switch_t        SW2;
    stick_t         CH0;
    stick_t         CH1;
    stick_t         CH2;
    stick_t         CH3;
    stick_t         THUMBWHEEL;
    button_t        W;
    button_t        S;
    button_t        A;
    button_t        D;
    button_t        SHIFT;
    button_t        CTRL;
    button_t        Q;
    button_t        E;
    button_t        R;
    button_t        F;
    button_t        G;
    button_t        Z;
    button_t        X;
    button_t        C;
    button_t        V;
    button_t        B;
    button_t        MOUSE_L;
    button_t        MOUSE_R;
    mouse_speed_t   MOUSE_VX;
    mouse_speed_t   MOUSE_VY;
    mouse_speed_t   MOUSE_VZ;
    
	int16_t		    offline_cnt;
	int16_t		    offline_max_cnt;
} rc_sensor_info_t;

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
    bool                (*if_channel_reset)(struct rc_sensor_struct *self);
    void                (*reset)(struct rc_sensor_struct *self);
    button_flip_t       (*update_button)(button_t *btn, button_state_t new_btn_state);
    switch_flip_t       (*update_switch)(switch_t *sw, switch_state_t new_sw_state);
    void                (*update_stick)(stick_t *stick, int16_t val);
    void                (*update_mouse)(mouse_speed_t *mouse_speed, int16_t val);
    button_flip_t       (*copy_button)(button_t *dst, button_t *src);
    switch_flip_t       (*copy_switch)(switch_t *sw, switch_t *src);
} rc_sensor_t;

extern rc_sensor_info_t     rc_sensor_info;
extern rc_sensor_t 		    rc_sensor;
/* Exported functions --------------------------------------------------------*/
	
#endif
