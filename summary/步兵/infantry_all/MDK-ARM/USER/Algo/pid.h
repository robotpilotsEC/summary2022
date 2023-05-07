#ifndef __PID_H
#define __PID_H
#include "rp_config.h"
#include "rp_math.h"


typedef struct pid_ctrl {
    float		target;
    float		measure;
    float 	err;
    float 	last_err;
    float		kp;
    float 	ki;
    float 	kd;
    float 	pout;
    float 	iout;
    float 	dout;
    float 	out;
    float		integral;
    float 	integral_max;
    float 	out_max;
	  float   deta_err;
	  float   blind_err;
} pid_ctrl_t;


typedef struct pid_info_struct{
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	float		  target;
}pid_watch_info;



void  pid_val_init(pid_ctrl_t *pid);
float err_calculation(pid_ctrl_t *pid,float measure);

void single_pid_ctrl(pid_ctrl_t *pid);
void cascade_pid_ctrl(pid_ctrl_t *outpid,pid_ctrl_t *inpid);


#endif

