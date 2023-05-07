#ifndef __BUTTON_SENSOR_H
#define __BUTTON_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
    RELEASE = 0,// �ɿ�
    PRESS = 1   // ����
} button_state_t;

typedef enum {
    BTN_NONE_FLIP = 0,  // �޵�ƽ��ת
    PRESS_TO_RELEASE = 1,  // ����->�ɿ�
    RELEASE_TO_PRESS = 2   // �ɿ�->����
} button_flip_t;

typedef struct {
    button_state_t  state;      // ��ǰ״̬
    button_flip_t   flip;       // ��ת״̬
    uint32_t        update_time;// ����ʱ��(HALTICK: 1msΪ��λ)
    uint32_t        hold_time;  // ����ʱ��(HALTICK: 1msΪ��λ)
} button_info_t;

typedef struct button_sensor_struct {
	button_info_t	    *info;
	drv_io_t			*driver;
	void				(*init)(struct button_sensor_struct *self);
	void				(*update)(struct button_sensor_struct *self);
	void				(*check)(struct button_sensor_struct *self);	
	void				(*heart_beat)(struct button_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;	
} button_sensor_t;

//extern button_sensor_t button_sensor[BTN_SENSOR_CNT];

/* Exported functions --------------------------------------------------------*/

#endif
