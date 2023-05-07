#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"

#define BELT_MID_CAN_ID		0x201U//�ư�ͬ����
#define BELT_SIDE_CAN_ID 	0x202U//���ſ�ͬ����
#define PUSH_TURN_CAN_ID	0x203U//�ư���ת
#define DART_WHEEL_CAN_ID   0x204U//���ڹ�����
#define PITCH_M_CAN_ID	    0x205U//PITCH��
#define YAW_M_CAN_ID	    0x206U//YAW��
#define GIMBAL_CAN_ID	    0x207U//������̨


typedef struct motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} motor_info_t;

typedef struct motor_reset_struct {
    uint16_t speed;         //��λʱ���ת��
    uint16_t time;          //��ʱ
    uint8_t state;          //״̬Ϊ3��λ���
    int8_t direction;       //����������븴λ�˶�������ͬ��1��������-1
    uint16_t temp_outmax;   //����pid�����������
    float ori_outmax;       //ԭ����pid�����������
    int32_t position;      //��λ����Ҫȥ��λ��
}motor_reset_t;

typedef volatile struct _pid_t{
    float set;
    float actual;
    float err;
    float err_last;
    float kp,ki,kd;
    float pout,iout,dout;
    float integral;
    float iout_max;
    float out;
    float outmax;
    uint8_t sw;
}pid_t;
extern pid_t pid_speed[];
extern pid_t pid_angle[];
typedef struct motor_struct {
	motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct motor_struct *self);
	void					(*update)(struct motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct motor_struct *self);
	void					(*heart_beat)(struct motor_struct *self);
	void					(*reset)(struct motor_struct *self);
    motor_reset_t           *res;
    pid_t                   *pid_s;
    pid_t                   *pid_a;
	dev_work_state_t		work_state;
	dev_errno_t				errno;
} motor_t;

extern motor_info_t 	motor_info[];
extern drv_can_t		motor_driver[];
extern motor_t		    motor[];

void motor_reset(motor_t *motor);
void motor_update(motor_t *motor, uint8_t *rxBuf);
void motor_init(motor_t *motor);
void CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
int16_t RampInt(int16_t final, int16_t now, int16_t ramp);


#endif


