#include "pid.h"

/*
	��ʼ��
*/

void pid_val_init(pid_ctrl_t *pid)
{
//pid->target = 0;//ԭ����ע�͵�
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
	������ ������Ŀ��-����
*/
float err_calculation(pid_ctrl_t *pid,float measure)
{
//	pid->last_err = pid->err;
	pid->measure = measure;
	pid->err = pid->target - pid->measure;
	return pid->err;
}


/*
	����pid
*/
void single_pid_ctrl(pid_ctrl_t *pid)
{
	// �������ֵ(��Ҫ���������м������)
//	pid->err = err;
	if(abs(pid->err)<=(pid->blind_err))
		pid->err = 0;
	// ����
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
	// p i d ��������
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	// �ۼ�pid���ֵ
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// ��¼�ϴ����ֵ
	pid->last_err = pid->err;
}


/*
	˫��
*/
void cascade_pid_ctrl(pid_ctrl_t *outpid,pid_ctrl_t *inpid)
{
	/*Ŀ��ֵ�����ֵ��Ҫ���������*/
	pid_ctrl_t *outPid = outpid;
	pid_ctrl_t *inPid = inpid;
	
	// �⻷PID
	outPid->err = outPid->target - outPid->measure;//			
//	//��Ȧ�ж�
//			if(abs(motor_pid->locat_pid.err) > 4095)
//			{
//				/* 0�� -> 8191 */
//				if(motor_pid->locat_pid.err > 0)
//					motor_pid->locat_pid.err -= 8191;

//				else if(motor_pid->locat_pid.err < 0)
//				motor_pid->locat_pid.err += 8191;
//			}	
	single_pid_ctrl(outPid);
	// �ڻ����� = �⻷���
	inPid->target = outPid->out;
	// �ڻ�PID
	inPid->err = inPid->target - inPid->measure;
	single_pid_ctrl(inPid);
}


