#include "pid.h"
#include "can.h"
#include "motor.h"
#include "math.h"
#include <stdlib.h> 
#include "rc.h"

extern motor_info_t 	motor_info[];
extern drv_can_t		motor_driver[];
extern motor_t		    motor[];

//M3508 16384  BELT_SIDE DART_WHEEL PITCH_M BELT_MID
//M2006 10000  PUSH_TURN GIMBAL YAW_M
pid_t pid_speed[] = {
    [BELT_MID] = {
        .kp = 8,
        .ki = 0.35,
        .kd = 0,
        .iout_max = 16000,
        .outmax = 16000,
    },
    [BELT_SIDE] = {
        .kp = 12,
        .ki = 0.5,
        .kd = 0,
        .iout_max = 3000,
        .outmax = 5000,
    },
    [PUSH_TURN] = {
        .kp = 20,
        .ki = 0.5,
        .kd = 0,
        .iout_max = 3000,
        .outmax = 4000,
    },
    [DART_WHEEL] = {
        .kp = 6,
        .ki = 0.3,
        .kd = 0,
        .iout_max = 5000,
        .outmax = 6000,
    },
    [PITCH_M] = {
        .kp = 8,
        .ki = 0.8,
        .kd = 0,
        .iout_max = 4000,
        .outmax = 6000,
    },
    [YAW_M] = {
        .kp = 8,
        .ki = 0.8,
        .kd = 0,
        .iout_max = 4000,
        .outmax = 6000,
    },
    [GIMBAL] = {
        .kp = 20,
        .ki = 0.5,
        .kd = 0,
        .iout_max = 2000,
        .outmax = 4000,
    },
};

pid_t pid_angle[] = {
    [BELT_MID] = {
        .kp = 0.15,
        .outmax = 5000,
    },
    [BELT_SIDE] = {
        .kp = 0.15,
        .outmax = 3000,
    },
    [PUSH_TURN] = {
        .kp = 0.1,
        .outmax = 1000,
    },
    [DART_WHEEL] = {
        .kp = 0.1,
        .outmax = 500,
    },
    [PITCH_M] = {
        .kp = 0.05,
        .outmax = 3000,
    },
    [YAW_M] = {
        .kp = 0.1,
        .outmax = 3000,
    },
    [GIMBAL] = {
        .kp = 0.1,
        .outmax = 200,
    },
};

void pid_speed_control(pid_t *pid)
{
    if(pid->sw == 0)
    {
        pid->integral = 0;
        pid->out = 0;
        pid->err_last = 0;
        return;
    }
    
    pid->err = pid->set - pid->actual;
    
    pid->integral += pid->err;
    
    float i_temp = pid->iout_max / pid->ki;
    
    if(pid->integral > i_temp)pid->integral = i_temp;
    else if(pid->integral < -i_temp)pid->integral = -i_temp;
    
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->err_last);
    
    float temp = pid->pout + pid->iout + pid->dout;
    
    if(temp > pid->outmax)pid->out = pid->outmax;
    else if(temp < -pid->outmax)pid->out = -pid->outmax;
    else pid->out = temp;
    
    pid->err_last = pid->err;
}

void pid_angle_control(pid_t *pid)
{
    if(pid->sw == 0)
    {
        pid->integral = 0;
        pid->out = 0;
        pid->err_last = 0;
        return;
    }
        
    pid->err = (int32_t)(pid->set - pid->actual) / 50 * 50;
    
    pid->integral += pid->err;
    
    float i_temp = pid->iout_max / pid->ki;
    
    if(pid->integral > i_temp)pid->integral = i_temp;
    else if(pid->integral < -i_temp)pid->integral = -i_temp;
    
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->err_last);
    
    float temp = pid->pout + pid->iout + pid->dout;
    
    if(temp > pid->outmax)pid->out = pid->outmax;
    else if(temp < -pid->outmax)pid->out = -pid->outmax;
    else pid->out = temp;
    
    pid->err_last = pid->err;
}


