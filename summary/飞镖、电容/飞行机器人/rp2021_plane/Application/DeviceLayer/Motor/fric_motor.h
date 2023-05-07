#ifndef __FRIC_MOTOR_H
#define __FRIC_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct fric_motor_struct {
	void 				*info;
	drv_pwm_t			*driver;
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
} fric_motor_t;

extern fric_motor_t	fric_motor[FRIC_MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/

#endif
