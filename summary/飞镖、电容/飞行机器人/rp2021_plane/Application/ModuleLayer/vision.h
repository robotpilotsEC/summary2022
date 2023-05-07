#ifndef __VISION_H
#define __VISION_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "vision_sensor.h"
#include "rc_sensor.h"
#include "judge_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 *	@brief	�Ӿ�ģʽ
 */
typedef enum
{
	VISION_MODE_MANUAL		= 0,	// �ֶ�ģʽ
    VISION_MODE_AIM_ARMY    = 1,    // ������浥λ
	VISION_MODE_AIM_OUTPOST = 2,	// ����ǰ��վģʽ
} vision_mode_t;

typedef struct {
    vision_sensor_t *vision_sensor;
	rc_sensor_t		*rc_sensor; 
    judge_sensor_t  *judge_sensor;
} vision_dev_t;

typedef struct {
    vision_mode_t   mode;
} vision_info_t;

typedef struct {
	vision_dev_t		*dev;
    vision_info_t       *info;
	void			    (*init)(void);
	void			    (*test)(void);
	void			    (*ctrl)(void);
	void			    (*self_protect)(void);    
} vision_t;

extern vision_t vision;
/* Exported functions --------------------------------------------------------*/

#endif
