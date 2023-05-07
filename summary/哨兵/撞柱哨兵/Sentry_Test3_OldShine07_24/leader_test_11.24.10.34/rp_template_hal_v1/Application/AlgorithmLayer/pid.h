#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void pid_val_init(pid_ctrl_t *pid);
void single_pid_ctrl(pid_ctrl_t *pid);
void pid_set(pid_ctrl_t *pid, float kp_value, float ki_value, float kd_value);
void cascade_pid_ctrl(pid_ctrl_t *pid);

#endif
