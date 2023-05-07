/* Includes ------------------------------------------------------------------*/
#include "dial.h"
#include "drv_io.h"
#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"
#include "driver.h"
#include "control_task.h"
#include  "vision_sensor.h"
/* Private macro -------------------------------------------------------------*/
/*������ֵ*/
#define PITCH_THRESHOLD 3
#define YAW_THRESHOLD 3
/*�������*/
#define STUCK_THRESHOLD 300
#define STUCK_HANDLE_TIME 200
#define STUCK_MOTOR_SPEED 10

/* Private function prototypes -----------------------------------------------*/
void SHOOT_Init(void);
void SHOOT_Ctrl(void);
void SHOOT_Test(void);
void SHOOT_SelfProtect(void);

void DIAL_NormoalShot(void);     
void DIAL_TripleShot(void);
void DIAL_RepeatingShot(void);
void DIAL_StuckHandle(void);   

void DIAL_StopShot(void);
void SHOOTER_HeatLimit(void);            

/* Private typedef -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// ������������������
drv_can_t				*shot_drv[SHOOT_MOTOR_CNT];
motor_t			*shot_motor[SHOOT_MOTOR_CNT];
motor_info_t	*shot_motor_info[SHOOT_MOTOR_CNT];

//��Ƶ
float shoot_freq_val = SHOOT_FREQ_TEN;  //��ʼ10��Ƶ
//����
float firing_rate = FIRING_RATE_HIGH;

/* Private variables ---------------------------------------------------------*/
// ����������PID������
shoot_motor_pid_t 	shot_motor_pid[SHOOT_MOTOR_CNT] = {
	[DIAL_CLASS] = {
		.speed.kp = 8,   //6
		.speed.ki = 0.02,  //0.02
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 10000,
		.angle.kp = 0.15,  //0.15
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	},
	[FRICTION_L_CLASS] = {
		.speed.kp = 11.5f,   //11.5
		.speed.ki = 0.475f,   //0.475
		.speed.kd = 0,
		.speed.integral_max = 10000,
		.speed.out_max = 18000,
		.angle.kp = 0.55f,    //0.55
		.angle.ki = 0,
		.angle.kd = 0.008f,   //0.008
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[FRICTION_R_CLASS] = {
		.speed.kp = 11.5f,   //11.5
		.speed.ki = 0.472f,  //0.472
		.speed.kd = 0,
		.speed.integral_max = 10000,
		.speed.out_max = 18000,
		.angle.kp = 0.55f,   //0.55
		.angle.ki = 0,
		.angle.kd = 0.008f,   //0.008
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
};

// �������ģ�������
shoot_ctrl_t		shot_ctrl = {
	.motor = &shot_motor_pid,
};

// �������ģ�鴫����
shoot_dev_t		shot_dev = {
	.shot_motor[DIAL_CLASS] = &motor[DIAL],
	.shot_motor[FRICTION_L_CLASS] = &motor[FRICTION_L],
	.shot_motor[FRICTION_R_CLASS] = &motor[FRICTION_R],	
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// �������ģ����Ϣ
shoot_info_t 	shot_info = {
	.remote_mode = RC,
	.local_mode = DIAL_MODE_NORMAL,
	.normal_init_flag = false,
	.triple_init_flag = false,
	.repeating_init_flag = false,	
	.stuck_init_flag = false,
};

shoot_t shoot = {
	.controller = &shot_ctrl,
	.dev = &shot_dev,
	.info = &shot_info,
	.init = SHOOT_Init,
	.test = SHOOT_Test,
	.ctrl = SHOOT_Ctrl,
	.self_protect = SHOOT_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
 *	@brief	�������
 */
void StuckCheck(void)
{
	static uint16_t stuck_cnt = 0;

	if(!shoot.stuck)	
	{
		if(shot_info.local_mode == DIAL_MODE_REPEATING)
		{
			if(shoot.fire_open)
			{
				if(abs(shot_motor_pid[DIAL_CLASS].speed.measure) < STUCK_MOTOR_SPEED)
				{
					stuck_cnt++;
				}
				else
				{
					stuck_cnt = 0;
				}
				
				if(stuck_cnt > STUCK_THRESHOLD)
				{
					shoot.stuck = true;   //��������
					stuck_cnt = 0;    //��������
				}
			}
		}
	}
	else if(!shoot.fire_open)      //�޿���
	{
		stuck_cnt = 0;        //��������
	}
}

/**
 *	@brief	�����ٴ���
 */
void RateLimit_Process(void)
{	
	static float last_speed = 0;
	static float now_speed = 0;

	static uint16_t low_speed_cnt = 0;

	if(master_info.RxPacket.shoot_update)
	{
		last_speed = now_speed;
		now_speed = master_info.RxPacket.bullet_speed;

		master_info.RxPacket.shoot_update = false;
		
		if((master_info.RxPacket.bullet_speed > 28) && (now_speed != last_speed))
		{
			firing_rate -= 50;
			if(firing_rate < 0)
			{
				firing_rate = 0;
			}
		}
		else if((master_info.RxPacket.bullet_speed < 27) && (now_speed != last_speed))  //���ٲ������������ٹ���
		{
			low_speed_cnt++;
			if(low_speed_cnt > 0)
			{
				low_speed_cnt = 0;
				firing_rate += 50;
				if(firing_rate > 7000)
				{
					firing_rate = 7000;
				}
			}
		}

	}
}

void Fric_SpeedCtrl(void)
{
	RateLimit_Process();  //�������ƣ���ֹһֱ������
	
	if(master_info.RxPacket.ctrl_mode.fric_open == FRIC_OPEN)    //��������̨Ħ����
	{
		shoot.fric_speed_target = firing_rate;     
	}
	else if(master_info.RxPacket.ctrl_mode.fric_open == FRIC_CLOSE)     //�ر�Ħ����
	{
		shoot.fric_speed_target = 0;   
	}
	
	shot_motor_pid[FRICTION_L_CLASS].speed.target = -shoot.fric_speed_target;
	shot_motor_pid[FRICTION_R_CLASS].speed.target = shoot.fric_speed_target;
}
/**
 *	@brief	����������PID������ʼ��
 */
void SHOOT_PidParamsInit(shoot_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}
}

/**
 *	@brief	����������ж��
 */
static void SHOOT_Stop(shoot_motor_pid_t *pid)
{
	for(uint8_t i=0; i < SHOOT_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		shot_drv[i]->add_halfword(shot_drv[i], (int16_t)pid[i].out);
	}
}

/**
 *	@brief	����������PID���
 */
static void SHOOT_PidOut(shoot_motor_pid_t *pid)
{
	for(uint8_t i=0; i < SHOOT_MOTOR_CNT; i++) {
		if(shot_motor[i]->work_state == DEV_ONLINE) {
            shot_drv[i]->add_halfword(shot_drv[i], (int16_t)pid[i].out);
		} 
		else {
            shot_drv[i]->add_halfword(shot_drv[i], 0);
		}
	}    
}

/**
 *	@brief	�����������ٶȻ�
 */
static void SHOOT_Speed_PidCalc(shoot_motor_pid_t *pid, shoot_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	����������λ�û�
 */
static void SHOOT_Angle_PidCalc(shoot_motor_pid_t *pid, shoot_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	single_pid_ctrl(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	SHOOT_Speed_PidCalc(pid,MOTORx);
}


/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/
/**
 *	@brief	���������ȡϵͳ��Ϣ
 */
void SHOOT_GetSysInfo(void)
{
	/*----���Ʒ�ʽ�޸�----*/
	if(sys.remote_mode == RC) {
		shot_info.remote_mode = RC;
	}
	else if(sys.remote_mode == AUTO)
	{
		shot_info.remote_mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) 
	{
		shot_info.remote_mode = INSPECTION;
	}
	
	/*----����ģʽ�޸�----*/
}

/**
 *	@brief	��̨����Ϣ��ȡ
 */
void DIAL_GetAerialInfo(void)
{
	if(master_info.RxPacket.ctrl_mode.stop_fire)
	{
		shoot.stop_fire = true;
	}
	else
	{
		shoot.stop_fire = false;
	}
}

/**
* @brief ��̨�ֿ��ƿ���״̬
* @note Ctrl+W �ڱ�������ֹ̨ͣ����
*/
void Aerial_Ctrl(void)            
{
	if(shoot.stop_fire)
	{
		shoot.fire_open = false;   //����̨��������			
	}
}

void SHOOT_UpdateController(void)
{
	shot_motor_pid[DIAL_CLASS].speed.measure = shot_motor_info[DIAL_CLASS]->speed;
	shot_motor_pid[DIAL_CLASS].angle.measure = shot_motor_info[DIAL_CLASS]->angle_sum;
	
	shot_motor_pid[FRICTION_L_CLASS].speed.measure = shot_motor_info[FRICTION_L_CLASS]->speed;
	shot_motor_pid[FRICTION_R_CLASS].speed.measure = shot_motor_info[FRICTION_R_CLASS]->speed;
}

/* Ӧ�ò� --------------------------------------------------------------------*/
/* ����� --------------------------------------------------------------------*/
void SHOOT_Init(void)
{
	shot_drv[DIAL_CLASS] = shot_dev.shot_motor[DIAL_CLASS]->driver;
	shot_drv[FRICTION_L_CLASS] = shot_dev.shot_motor[FRICTION_L_CLASS]->driver;
	shot_drv[FRICTION_R_CLASS] = shot_dev.shot_motor[FRICTION_R_CLASS]->driver;
	
	shot_motor[DIAL_CLASS] = shot_dev.shot_motor[DIAL_CLASS];
	shot_motor[FRICTION_L_CLASS] = shot_dev.shot_motor[FRICTION_L_CLASS];
	shot_motor[FRICTION_R_CLASS] = shot_dev.shot_motor[FRICTION_R_CLASS];
	
	shot_motor_info[DIAL_CLASS] = shot_dev.shot_motor[DIAL_CLASS]->info;	
	shot_motor_info[FRICTION_L_CLASS] = shot_dev.shot_motor[FRICTION_L_CLASS]->info;	
	shot_motor_info[FRICTION_R_CLASS] = shot_dev.shot_motor[FRICTION_R_CLASS]->info;	
	
	shoot.fric_speed_target = 0;
	shoot.fire_open = false;
	shoot.stop_shot = false;
	shoot.stuck = false;
}

void SHOOT_GetInfo(void)
{
	SHOOT_GetSysInfo();
	DIAL_GetAerialInfo();
	SHOOT_UpdateController();
}

void SHOOT_SelfProtect(void)
{
	SHOOT_Stop(shot_motor_pid);
	SHOOT_PidParamsInit(shot_motor_pid, SHOOT_MOTOR_CNT);
	SHOOT_GetInfo();
}

void SHOOT_ModeSelect(void)        //�����������ģʽѡ��
{
	if(!shoot.stuck)      //���޿���
	{
		switch(shot_info.local_mode)
		{
			case DIAL_MODE_NORMAL:      DIAL_NormoalShot();      break;
			case DIAL_MODE_TRIPLE_SHOT: DIAL_TripleShot();       break;
			case DIAL_MODE_REPEATING:   DIAL_RepeatingShot();    break;
		}
	}
	else    //����������
	{
		DIAL_StuckHandle(); //��������
	}

}

void SHOOT_PidCtrl(void)
{
	// �����������ٶȻ�
	SHOOT_Speed_PidCalc(shot_motor_pid, FRICTION_L_CLASS);
	SHOOT_Speed_PidCalc(shot_motor_pid, FRICTION_R_CLASS);	
	
	if(!shoot.stuck)
	{
		if(shot_info.local_mode == DIAL_MODE_REPEATING)
		{
			// ���̵���ٶȻ�
			SHOOT_Speed_PidCalc(shot_motor_pid, DIAL_CLASS);
		}
		else
		{
			// ���̵��λ�û�
			SHOOT_Angle_PidCalc(shot_motor_pid, DIAL_CLASS);
		}
	}
	else
	{
		// ���̵��λ�û�
		SHOOT_Angle_PidCalc(shot_motor_pid, DIAL_CLASS);
	}
	// ���������������Ӧ
	SHOOT_PidOut(shot_motor_pid);
	
}

/**
* @brief ���̴򵯲���
*/
static void Shoot_Strategy(void)
{
	if(vision_sensor_info.RxPacket.RxData.identify_target)   //Ŀ������Ұ��
	{
		if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) <  PITCH_THRESHOLD) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) <  YAW_THRESHOLD))
		{
			shoot.fire_open = true;	    //����
			shoot_freq_val = SHOOT_FREQ_TEN;
		}
		else if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) < 2) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) < 2))
		{
			shoot.fire_open = true;	     //����
			shoot_freq_val = SHOOT_FREQ_HIGH;
			
		}
		else if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) <  1) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) <  1))
		{
			shoot.fire_open = true;	     //����
			shoot_freq_val = SHOOT_FREQ_HEATLIMIT;	
		}
		else    //Ŀ��û�ڿ�����ֵ��
		{
			shoot.fire_open = false;    //������
		}
		
	}
	else                //Ŀ�겻����Ұ��
	{
		shoot.fire_open = false;   //������
	}

//	if(gimbal.predict_open)  //Ԥ�⿪��
//	{
//		if(abs(vision.pitch_angle_corrected + vision.pitch_angle_offset) < SHOOT_THRESHOLD)  //����ֵ��
//		{
//			shoot.fire_open = true;	
//		}
//		else
//		{
//			shoot.fire_open = false;
//		}
//	}
//	else if(!gimbal.predict_open)   //Ԥ��ر�
//	{
//		if(abs(vision.pitch_angle_corrected) < SHOOT_THRESHOLD)  //����ֵ��
//		{
//			shoot.fire_open = true;	
//		}
//		else
//		{
//			shoot.fire_open = false;
//		}
//	}
}

uint8_t ACCESS = 0;
void SHOOT_RcCtrl(void)  //ң����Testר��
{
	shot_info.local_mode = DIAL_MODE_REPEATING;
	//�������ң��������ר��
	if(master_info.RxPacket.ctrl_mode.fire_open)
	{
		shoot.fire_open = true;
	}
	else if(!master_info.RxPacket.ctrl_mode.fire_open)
	{
		shoot.fire_open = false;
	}

}

void SHOOT_KeyCtrl(void)
{

}

/**
* @brief ��������Զ�ģʽ
* @param void
* @return void
*/
void SHOOT_AutoCtrl(void)
{
	shot_info.local_mode = DIAL_MODE_REPEATING;
//	//�������ң��������ר��
//	if(master_info.RxPacket.ctrl_mode.fire_open && ACCESS)
//	{
//		ACCESS = 0;
//		shoot.fire_open = true;
//	}
//	else if(!master_info.RxPacket.ctrl_mode.fire_open)
//	{
//		ACCESS = 1;
//		shoot.fire_open = false;
//	}
	
	if(master_info.RxPacket.ctrl_mode.dial_lock_state == DIAL_UNLOCK)
	{
		Shoot_Strategy();  //�򵯲���
	}
	else if(master_info.RxPacket.ctrl_mode.dial_lock_state == DIAL_LOCK)
	{
		shoot.fire_open = false;
	}

}

/**
* @brief ��������Լ�ģʽ
* @param void
* @return void
*/
void SHOOT_InspectionCtrl(void)
{
	shot_info.local_mode = DIAL_MODE_REPEATING;

	/*---������---*/	
	//�������ң��������ר��
	if(master_info.RxPacket.ctrl_mode.fire_open)
	{
		shoot.fire_open = true;
	}
	else if(!master_info.RxPacket.ctrl_mode.fire_open)
	{
		shoot.fire_open = false;
	}
	/*---������---*/
//	shoot.fire_open = false;
	
}

void SHOOT_Ctrl(void)
{
	/*----��Ϣ����----*/
	SHOOT_GetInfo();
	/*----�����޸�----*/ 
	if(shot_info.remote_mode == RC) {
		SHOOT_RcCtrl();
	}
	else if(shot_info.remote_mode == KEY) {
		SHOOT_KeyCtrl();
	}
	else if(shot_info.remote_mode == AUTO) {
		SHOOT_AutoCtrl();
	}
	else if(shot_info.remote_mode == INSPECTION) {
		SHOOT_InspectionCtrl();
	}
	Fric_SpeedCtrl();   //Ħ�����ٶ�������ֵ(���ڴ��޸�Ħ���ֵ��ٶ�)
	/*---��̨�ֿ��ƿ���״̬---*/	
	Aerial_Ctrl();
	/*---ǹ����������---*/
	SHOOTER_HeatLimit();
	/*---���̿������---*/	
	StuckCheck();
	/*---���ģʽѡ��---*/	
	SHOOT_ModeSelect();
	/*----�������----*/
	SHOOT_PidCtrl();	
}

void SHOOT_Test(void)
{
    
}

/**
* @brief ǹ����������
* @param void
* @return void
* ���������޼�ȥ��ǰ����ֵ�õ�ʣ�����������
* �жϿ��������Ƿ���ڷ���һ����λ����������������Ƿ�����ǹ
*/
void SHOOTER_HeatLimit(void)            
{
	static float Real_Heat_Error;
	static float Shooter_RealHeat;
	static float Shooter_LimitHeat;

	if(shot_info.local_mode == DIAL_MODE_REPEATING)       //����������������ӣ���ֹ��Ѫ
	{
		Real_Heat_Error = One_Shot_17mm + 50;
	}
	else
	{
		Real_Heat_Error = One_Shot_17mm;
	}
	
	Shooter_RealHeat = 	master_info.RxPacket.cooling_heat;
	Shooter_LimitHeat = COOLING_LIMIT;
	
	if(Shooter_LimitHeat - Shooter_RealHeat < Real_Heat_Error)   //��������С���������������
	{
		shoot.fire_open = false;
	}
}

/**
 *	@brief	��������ģʽ��������
 */
void DIAL_NormoalShot(void)      
{
//	dia_motor_pid[DIAL_CLASS].speed.target = 0; //���޸ģ���ǰֻ����������
	if(!shot_info.normal_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		shot_info.normal_init_flag = true;		
		shot_info.triple_init_flag = false;
		shot_info.repeating_init_flag = false;
		
		shot_motor_pid[DIAL_CLASS].angle.target = 0;
		shot_motor_pid[DIAL_CLASS].angle.measure = 0;				
		motor[DIAL].info->angle_sum = 0;

		shoot.fire_open = false;
	}
	if(shoot.fire_open)
	{
		shoot.fire_open = false;
		shot_motor_pid[DIAL_CLASS].angle.target += SHOOT_SINGLE_ANGLE;
	}
}

/**
 *	@brief	����������ģʽ
 */
void DIAL_TripleShot(void)
{
	if(!shot_info.triple_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		shot_info.triple_init_flag = true;
		shot_info.repeating_init_flag = false;
		shot_info.normal_init_flag = false;
		
		shot_motor_pid[DIAL_CLASS].angle.target = 0;
		shot_motor_pid[DIAL_CLASS].angle.measure = 0;		
		motor[DIAL].info->angle_sum = 0;

		shoot.fire_open = false;
	}
	if(shoot.fire_open)
	{
		shoot.fire_open = false;
		shot_motor_pid[DIAL_CLASS].angle.target += SHOOT_TRIPLE_ANGLE;
	}
}

/**
 *	@brief	��������ģʽ
 */
void DIAL_RepeatingShot(void)
{
	if(!shot_info.repeating_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		shot_info.repeating_init_flag = true;
		shot_info.normal_init_flag = false;
		shot_info.triple_init_flag = false;
		shot_info.stuck_init_flag = false;

		shot_motor_pid[DIAL_CLASS].angle.target = 0;
		shot_motor_pid[DIAL_CLASS].angle.measure = 0;
		motor[DIAL].info->angle_sum = 0;

//		shoot.fire_open = false;
	}
	if(shoot.fire_open)
	{
		shot_motor_pid[DIAL_CLASS].speed.target = shoot_freq_val;  //

//		shot_motor_pid[DIAL_CLASS].speed.target = SHOOT_FREQ_TWELVE;  //12  //����Ƶ(4)����...
	}
	else
	{
		shot_motor_pid[DIAL_CLASS].speed.target = 0;
	}
}


/**
 *	@brief	�������ֹͣ����
 */
void DIAL_StopShot(void)
{
	shot_motor_pid[DIAL_CLASS].speed.target = 0;
	shot_motor_pid[DIAL_CLASS].angle.target = 0;
	shot_motor_pid[DIAL_CLASS].angle.measure = 0;
	
	motor[DIAL].info->angle_sum = 0;
}

/**
 *	@brief	���̿�������
 */
void DIAL_StuckHandle(void)      
{
	static uint16_t stuck_finish_cnt = 0;
	if(!shot_info.stuck_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		shot_info.stuck_init_flag = true;		
		shot_info.triple_init_flag = false;
		shot_info.repeating_init_flag = false;
		shot_info.normal_init_flag = false;
		shot_motor_pid[DIAL_CLASS].angle.target = 0;
		shot_motor_pid[DIAL_CLASS].angle.measure = 0;				
		motor[DIAL].info->angle_sum = 0;
	}
	if(shoot.stuck && (stuck_finish_cnt == 0))
	{
		shot_motor_pid[DIAL_CLASS].angle.target -= SHOOT_SINGLE_ANGLE;		
	}

	stuck_finish_cnt++;
	
	if(stuck_finish_cnt > STUCK_HANDLE_TIME)
	{
		stuck_finish_cnt = 0;
		shoot.stuck = false;
	}
}

