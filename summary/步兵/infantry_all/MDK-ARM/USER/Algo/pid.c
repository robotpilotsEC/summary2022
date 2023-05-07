#include "pid.h"

/*
	初始化
*/

void pid_val_init(pid_ctrl_t *pid)
{
//pid->target = 0;//原来是注释的
//pid->measure = 0;//
	pid->err = 0;
	pid->last_err = 0;
	pid->integral = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}


/*
	误差计算 计算误差：目的-现在
*/
float err_calculation(pid_ctrl_t *pid,float measure)
{
//	pid->last_err = pid->err;
	pid->measure = measure;
	pid->err = pid->target - pid->measure;
	return pid->err;
}


/*
	单环pid
*/
void single_pid_ctrl(pid_ctrl_t *pid)
{
	// 保存误差值(需要在外面自行计算误差)
//	pid->err = err;
	if(abs(pid->err)<=(pid->blind_err))
		pid->err = 0;
	// 积分
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
	// p i d 输出项计算
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	// 累加pid输出值
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// 记录上次误差值
	pid->last_err = pid->err;
}


/*
	双环
*/
void cascade_pid_ctrl(pid_ctrl_t *outpid,pid_ctrl_t *inpid)
{
	/*目标值与测量值需要在外面给出*/
	pid_ctrl_t *outPid = outpid;
	pid_ctrl_t *inPid = inpid;
	
	// 外环PID
	outPid->err = outPid->target - outPid->measure;//			
//	//半圈判定
//			if(abs(motor_pid->locat_pid.err) > 4095)
//			{
//				/* 0↓ -> 8191 */
//				if(motor_pid->locat_pid.err > 0)
//					motor_pid->locat_pid.err -= 8191;

//				else if(motor_pid->locat_pid.err < 0)
//				motor_pid->locat_pid.err += 8191;
//			}	
	single_pid_ctrl(outPid);
	// 内环期望 = 外环输出
	inPid->target = outPid->out;
	// 内环PID
	inPid->err = inPid->target - inPid->measure;
	single_pid_ctrl(inPid);
}


