/**
 * @file        pid.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Pid Algorithm.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void pid_val_init(pid_ctrl_t *pid)
{
//	pid->target = 0;
//	pid->measure = 0;
	pid->err = 0;
	pid->last_err = 0;
	pid->integral = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}

void single_pid_ctrl(pid_ctrl_t *pid)
{
	// �������ֵ
//	pid->err = err;
	// ����
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
	// p i d ��������
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral * pid->loop_time;
	pid->dout = pid->kd * (pid->err - pid->last_err) / pid->loop_time;
	// �ۼ�pid���ֵ
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// ��¼�ϴ����ֵ
	pid->last_err = pid->err;
}

//void cascade_pid_ctrl(pid_ctrl_t *pid)
//{
//	pid_ctrl_t *outPid = pid;
//	pid_ctrl_t *inPid = outPid->next;
//	
//	// �⻷PID
//	outPid->err = outPid->target - outPid->measure;
//	single_pid_ctrl(outPid);
//	// �ڻ����� = �⻷���
//	inPid->target = outPid->out;
//	// �ڻ�PID
//	inPid->err = inPid->target - inPid->measure;
//	single_pid_ctrl(inPid);
//}
