#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include "rp_config.h"
#include "motor.h"
#include "bmi_solve.h"


void pid_speed_control(pid_t *pid);
void pid_angle_control(pid_t *pid);

extern pid_t pid_speed[];
extern pid_t pid_angle[];
#endif

