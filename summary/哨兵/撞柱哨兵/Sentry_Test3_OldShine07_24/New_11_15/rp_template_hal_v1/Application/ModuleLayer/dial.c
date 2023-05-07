/* Includes ------------------------------------------------------------------*/
#include "dial.h"
#include "drv_io.h"
#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"
#include "driver.h"
#include "control_task.h"
#include "judge_sensor.h"
#include  "vision_sensor.h"
/* Private macro -------------------------------------------------------------*/
/*������ֵ*/
#define PITCH_THRESHOLD 3
#define YAW_THRESHOLD 5
/*�������*/
#define STUCK_THRESHOLD 200
#define STUCK_HANDLE_TIME 200
#define STUCK_MOTOR_SPEED 10

/* Private function prototypes -----------------------------------------------*/
void DIAL_Init(void);
void DIAL_Ctrl(void);
void DIAL_Test(void);
void DIAL_SelfProtect(void);

void DIAL_NormoalShot(void);     
void DIAL_TripleShot(void);
void DIAL_RepeatingShot(void);
void DIAL_StuckHandle(void);   

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// ���̵����������
drv_can_t				*dia_drv[DIAL_MOTOR_CNT];
motor_t			*dia_motor[DIAL_MOTOR_CNT];
motor_info_t	*dia_motor_info[DIAL_MOTOR_CNT];

Friction_t friction = {.Target = 0};
//int8_t dial_direction = 1; //���̷���
uint16_t fric_cnt = 0;    //Ħ���ֿ���ʱ���ʱ������Ħ����ת���ȶ���ŷ��䵯��Ŀ���

//��Ƶ
float shoot_freq_val = SHOOT_FREQ_TEN;  //��ʼ10��Ƶ
//����
float firing_rate = FIRING_RATE_HIGH;

// ���̵��PID������
dial_motor_pid_t 	dia_motor_pid[DIAL_MOTOR_CNT] = {
	[DIAL_CLASS] = {
		.speed.kp = 6,       //6
		.speed.ki = 0.02f,   //0.02
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 8000,
		.angle.kp = 0.15,    //0.15
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	},
};

// ����ģ�������
dial_ctrl_t		dia_ctrl = {
	.motor = &dia_motor_pid,
};

// ����ģ�鴫����
dial_dev_t		dia_dev = {
	.dia_motor[DIAL_CLASS] = &motor[DIAL],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// ����ģ����Ϣ
dial_info_t 	dia_info = {
	.remote_mode = RC,
	.local_mode = DIAL_MODE_NORMAL,
	.normal_init_flag = false,
	.triple_init_flag = false,
	.repeating_init_flag = false,
	.stuck_init_flag = false,
};

dial_t dial = {
	.controller = &dia_ctrl,
	.dev = &dia_dev,
	.info = &dia_info,
	.init = DIAL_Init,
	.test = DIAL_Test,
	.ctrl = DIAL_Ctrl,
	.self_protect = DIAL_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
 *	@brief	�������
 */
void StuckCheck(void)
{
	static uint16_t stuck_cnt = 0;

	if(!dial.stuck)	
	{
		if(dia_info.local_mode == DIAL_MODE_REPEATING)
		{
			if(dial.fire_open)
			{
				if(abs(dia_motor_pid[DIAL_CLASS].speed.measure) < STUCK_MOTOR_SPEED)
				{
					stuck_cnt++;
				}
				else
				{
					stuck_cnt = 0;
				}
				
				if(stuck_cnt > STUCK_THRESHOLD)
				{
					dial.stuck = true;   //��������
					stuck_cnt = 0;    //��������
				}
			}
		}
	}
	else if(!dial.fire_open)      //�޿���
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
	
	if(judge_sensor_info.ShootData.shooter_id == 1)
	{
		last_speed = now_speed;      //��¼��һ������
		now_speed = judge_sensor_info.ShootData.bullet_speed;  //��õ�ǰ����
		
		if(judge_sensor_info.shoot_update)
		{
			judge_sensor_info.shoot_update = false;      //���ǹ�����ݸ��±�־
			
			if((judge_sensor_info.ShootData.bullet_speed > 27) && (now_speed != last_speed))  //�����ٽ���
			{
				firing_rate -= 2;       //������λ,�������е���������λ
				if(firing_rate < 0)
				{
					firing_rate = 0;
				}
			}
			else if((judge_sensor_info.ShootData.bullet_speed < 26) && (now_speed != last_speed))  //���ٲ������������ٹ���
			{
				low_speed_cnt++;
				if(low_speed_cnt > 0)
				{
					low_speed_cnt = 0;
					firing_rate += 2;    //������λ,�������е���������λ
					if(firing_rate > 800)
					{
						firing_rate = 800;
					}
				}
			}
		}
	}
}

/**
 *	@brief	Ħ���ֿ���
 */
void Fric_Control(void)
{
	if(dial.fric_unlock)   //Ħ�����ѽ���
	{
		RateLimit_Process();  //�������ƣ���ֹһֱ������
		
		if(dial.fric_open)       //����Ħ����
		{
			friction.Target = firing_rate;    
		}
		else                 //�ر�Ħ����
		{
			friction.Target = 0;
		}
		NormalPwm[FRICTION_L] = friction.Target;
		NormalPwm[FRICTION_R] = friction.Target;	
		
		FRICTION_PwmOut(NormalPwm[FRICTION_L],NormalPwm[FRICTION_R]);
	}
}
/**
 *	@brief	���̵��PID������ʼ��
 */
void DIAL_PidParamsInit(dial_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}
}

/**
 *	@brief	���̺�Ħ���ֵ��ж��
 */
static void DIAL_Stop(dial_motor_pid_t *pid)
{
	for(uint8_t i=0; i < DIAL_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		dia_drv[i]->add_halfword(dia_drv[i], (int16_t)pid[i].out);
	}
	//��Ħ���ֽ�����������ֹͣ
	if(dial.fric_unlock)
	{
		FRICTION_PwmOut(0, 0);
	}
}

/**
 *	@brief	���̵��PID���
 */
static void DIAL_PidOut(dial_motor_pid_t *pid)
{
	for(uint8_t i=0; i < DIAL_MOTOR_CNT; i++) {
		if(dia_motor[i]->work_state == DEV_ONLINE) {
            dia_drv[i]->add_halfword(dia_drv[i], (int16_t)pid[i].out);
		} 
		else {
            dia_drv[i]->add_halfword(dia_drv[i], 0);
		}
	}    
}

/**
 *	@brief	���̵���ٶȻ�
 */
static void DIAL_Speed_PidCalc(dial_motor_pid_t *pid, dial_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	���̵��λ�û�
 */
static void DIAL_Angle_PidCalc(dial_motor_pid_t *pid, dial_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	single_pid_ctrl(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	DIAL_Speed_PidCalc(pid,MOTORx);
}


/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/
/**
 *	@brief	���̻�ȡϵͳ��Ϣ
 */
void DIAL_GetSysInfo(void)
{
	/*----���Ʒ�ʽ�޸�----*/
	if(sys.remote_mode == RC) {
		dia_info.remote_mode = RC;
	}
//	else if(sys.remote_mode == KEY) {
//		dia_info.remote_mode = KEY;
//	}
	else if(sys.remote_mode == AUTO)
	{
		dia_info.remote_mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) 
	{
		dia_info.remote_mode = INSPECTION;
	}
	
	/*----����ģʽ�޸�----*/
}

/**
 *	@brief	��̨����Ϣ��ȡ
 */
void DIAL_GetAerialInfo(void)
{
	if(judge_sensor.info->GameRobotStatus.robot_id == 7)//��ɫid
	{
		if((judge_sensor.info->AerialData.send_id == 6) && (judge_sensor.info->AerialData.receive_id == 7))  //���˻����͸��ڱ�
		{
			if(judge_sensor.info->AerialData.cmd == stop_fire)
			{
				dial.stop_fire = true;
				leader_info.TxPacket.ctrl_mode.stop_fire = true;
			}
			else
			{
				dial.stop_fire = false;
				leader_info.TxPacket.ctrl_mode.stop_fire = false;
			}
		}
	}
	else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//��ɫid
	{
		if((judge_sensor.info->AerialData.send_id == 106) && (judge_sensor.info->AerialData.receive_id == 107))  //���˻����͸��ڱ�
		{
			if(judge_sensor.info->AerialData.cmd == stop_fire)
			{
				dial.stop_fire = true;
				leader_info.TxPacket.ctrl_mode.stop_fire = true;
			}
			else
			{
				dial.stop_fire = false;
				leader_info.TxPacket.ctrl_mode.stop_fire = false;
			}
		}
	}
	
}

/**
* @brief ��̨�ֿ��ƿ���״̬
* @note Ctrl+W �ڱ�������ֹ̨ͣ����
*/
void Aerial_Ctrl(void)            
{
	if(dial.stop_fire)
	{
		dial.fire_open = false;     //����̨��������
		leader_info.TxPacket.ctrl_mode.fire_open = false;   //����̨��������			
	}
}

void DIAL_UpdateController(void)
{
	dia_motor_pid[DIAL_CLASS].speed.measure = dia_motor_info[DIAL_CLASS]->speed;
	dia_motor_pid[DIAL_CLASS].angle.measure = dia_motor_info[DIAL_CLASS]->angle_sum;
}

/* Ӧ�ò� --------------------------------------------------------------------*/
/* ����� --------------------------------------------------------------------*/
void DIAL_Init(void)
{
	dia_drv[DIAL_CLASS] = dia_dev.dia_motor[DIAL_CLASS]->driver;

	dia_motor[DIAL_CLASS] = dia_dev.dia_motor[DIAL_CLASS];
	
	dia_motor_info[DIAL_CLASS] = dia_dev.dia_motor[DIAL_CLASS]->info;
	
	dial.fire_open = false;
	
	dial.fric_open = false;
	
	dial.stop_fire = false;
	
	dial.lock = true;
	
	dial.stuck = false;
	
	dial.fric_unlock = false;
}

void DIAL_GetInfo(void)
{
	DIAL_GetSysInfo();
	DIAL_GetAerialInfo();
	DIAL_UpdateController();
}

void DIAL_SelfProtect(void)
{
	DIAL_Stop(dia_motor_pid);
	DIAL_PidParamsInit(dia_motor_pid, DIAL_MOTOR_CNT);
	DIAL_GetInfo();
}

void DIAL_ModeSelect(void)        //���̱���ģʽѡ��(���ڴ������ж��Ƿ�ǿ��ͣ��)
{
	if(!dial.stuck)      //���޿���
	{
		switch(dia_info.local_mode)
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

void DIAL_PidCtrl(void)
{
	if(!dial.stuck)
	{
		if(dia_info.local_mode == DIAL_MODE_REPEATING)
		{
			// ���̵���ٶȻ�
			DIAL_Speed_PidCalc(dia_motor_pid, DIAL_CLASS);
		}
		else
		{
			// ���̵��λ�û�
			DIAL_Angle_PidCalc(dia_motor_pid, DIAL_CLASS);
		}
	}
	else
	{
		// ���̵��λ�û�
		DIAL_Angle_PidCalc(dia_motor_pid, DIAL_CLASS);
	}
	// ���̵�������Ӧ
	DIAL_PidOut(dia_motor_pid);
}


/**
* @brief ���̴򵯲���
*/
static void Shoot_Strategy(void)
{
	if(vision_sensor_info.RxPacket.RxData.identify_target)   //Ŀ������Ұ��
	{
		if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) < PITCH_THRESHOLD) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) <  YAW_THRESHOLD))
		{
			dial.fire_open = true;    //����
			shoot_freq_val = SHOOT_FREQ_TWELVE;
		}
		else if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) <  2) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) < 3))
		{
			dial.fire_open = true;    //����
			shoot_freq_val = SHOOT_FREQ_HIGH;
		}
		else if((abs(vision_sensor_info.RxPacket.RxData.pitch_angle) <  1) && (abs(vision_sensor_info.RxPacket.RxData.yaw_angle) < 1))
		{
			dial.fire_open = true;    //����
			shoot_freq_val = SHOOT_FREQ_HEATLIMIT;
		}
		else    //Ŀ��û�ڿ�����ֵ��
		{
			dial.fire_open = false;    //������
		}
	}
	else                //Ŀ�겻����Ұ��
	{
		dial.fire_open = false;    //������
	}
//	if(gimbal.predict_open)  //Ԥ�⿪��
//	{
//		if(abs(vision.pitch_angle_corrected + vision.pitch_angle_offset) < SHOOT_THRESHOLD)  //����ֵ��
//		{
//			dial.fire_open = true;	
//		}
//		else
//		{
//			dial.fire_open = false;
//		}
//	}
//	else if(!gimbal.predict_open)   //Ԥ��ر�
//	{
//		if(abs(vision.pitch_angle_corrected) < SHOOT_THRESHOLD)  //����ֵ��
//		{
//			dial.fire_open = true;	
//		}
//		else
//		{
//			dial.fire_open = false;
//		}
//	}
}

uint8_t ACCESS = 0;
void DIAL_RcCtrl(void)
{
	dia_info.local_mode = DIAL_MODE_REPEATING;    //Ĭ������(ͨ���ı���Ƶ�����������������������̶�)
	
	dial.lock = false;        //���̽���
	//�������ң��������ר�� 
	if(!dial.fric_unlock)  //����̨Ħ����δ����
	{
		dial.fric_open = false;  //�ر�Ħ����
	}
	else 
	{
		if(RC_THUMB_WHEEL_VALUE >= 600)
		{
			dial.fire_open = true;
			leader_info.TxPacket.ctrl_mode.fire_open = true;
		}
		else
		{
			dial.fire_open = false;
			leader_info.TxPacket.ctrl_mode.fire_open = false;
		}
		//ң�����¿���Ħ����
		if(RC_SW1_VALUE == RC_SW_UP)
		{
			dial.fric_open = true;
		}
		else if(RC_SW1_VALUE == RC_SW_DOWN)
		{
			dial.fric_open = false;
		}
	}
}

void DIAL_KeyCtrl(void)
{

}

/**
* @brief �����Զ�ģʽ
* @param void
* @return void
*/
void DIAL_AutoCtrl(void)
{
	dia_info.local_mode = DIAL_MODE_REPEATING;    //Ĭ������(ͨ���ı���Ƶ�����������������������̶�)
	
	if(judge_sensor_info.GameStatus.game_progress == 4)      //������ʼ
	{
		dial.fric_open = true;        //����Ħ����
		fric_cnt++;          //��ʱ�Ӽ�
		if(fric_cnt < 2000)       
		{
			dial.fire_open = false;       //�رղ���
			dial.lock = true;          //��������
		}
		else     //4s
		{
			dial.lock = false;        //���̽���
			fric_cnt = 2001;
			//�򵯲���
			Shoot_Strategy();
		}
	}
	else              //������û��ʼ
	{
		dial.lock = true;          //��������
		dial.fric_open = false;        //�ر�Ħ����
		dial.fire_open = false;       //�رղ���
		fric_cnt = 0;              //��������
	}
	
}

/**
* @brief �����Լ�ģʽ
* @param void
* @return void
*/
void DIAL_InspectionCtrl(void)
{
	static bool access_one = false;
	static uint16_t unlock_time = 0;
	static bool unlock_flg = false;
	
	dia_info.local_mode = DIAL_MODE_REPEATING;    //Ĭ������(ͨ���ı���Ƶ�����������������������̶�)

	/*---������---*/
	dial.lock = false;        //���̽���	

	if(!dial.fric_unlock)  //����̨Ħ����δ����
	{		
		if((RC_THUMB_WHEEL_VALUE <= -600) && access_one)
		{
			standard_flg = 1;
			access_one = false;
		}
		else if((RC_THUMB_WHEEL_VALUE >= 600) && access_one)
		{
			FRICTION_PwmOut(0, 0);  //�������
			unlock_flg = true;   //������־
			access_one = false;
		}
		else
		{
			access_one = true;			
		}
		
		if(unlock_flg)
		{
			unlock_time++;
			if(unlock_time > 500)  //1s�����Ħ����
			{
				unlock_time = 501;
				dial.fric_unlock = true;  //����
			}
		}
	}
	else if(dial.fric_unlock)   //�ѽ��������Խ��д�
	{
		//�������ң��������ר�� 
		if(RC_THUMB_WHEEL_VALUE >= 600)
		{
			dial.fire_open = true;
			leader_info.TxPacket.ctrl_mode.fire_open = true;
		}
		else
		{
			dial.fire_open = false;
			leader_info.TxPacket.ctrl_mode.fire_open = false;
		}		
	}
	/*---������---*/
//	dial.fric_open = false;
//	dial.lock = true;          //��������
//	dial.fire_open = false;       //�رղ���
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

	if(dia_info.local_mode == DIAL_MODE_REPEATING)       //����������������ӣ���ֹ��Ѫ
	{
		Real_Heat_Error = One_Shot_17mm + 80;
	}
	else
	{
		Real_Heat_Error = One_Shot_17mm;
	}
	
	Shooter_RealHeat = judge_sensor_info.PowerHeatData.shooter_id1_17mm_cooling_heat;
	Shooter_LimitHeat = COOLING_LIMIT;
	
	if(Shooter_LimitHeat - Shooter_RealHeat < Real_Heat_Error)   //��������С���������������
	{
		dial.fire_open = false;
	}
}

void DIAL_Ctrl(void)
{
	/*----��Ϣ����----*/
	DIAL_GetInfo();
	/*----�����޸�----*/ 
	if(dia_info.remote_mode == RC) {
		DIAL_RcCtrl();
	}
	else if(dia_info.remote_mode == AUTO) {
		DIAL_AutoCtrl();
	}
	else if(dia_info.remote_mode == INSPECTION) {
		DIAL_InspectionCtrl();
	}
	/*---��̨�ֿ��ƿ���״̬---*/
	Aerial_Ctrl();
	/*---ǹ����������---*/
	SHOOTER_HeatLimit();	
	/*---���̿������---*/	
	StuckCheck();
	/*---���ģʽѡ��---*/
	DIAL_ModeSelect();
	/*----�������----*/
	DIAL_PidCtrl();	
	Fric_Control();
}

void DIAL_Test(void)
{
    
}
/**
 *	@brief	��������ģʽ��������
 */
void DIAL_NormoalShot(void)      
{
	if(!dia_info.normal_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		dia_info.normal_init_flag = true;		
		dia_info.triple_init_flag = false;
		dia_info.repeating_init_flag = false;
		
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;				
		motor[DIAL].info->angle_sum = 0;

		dial.fire_open = false;
	}
	if(dial.fire_open)
	{
		dial.fire_open = false;
		dia_motor_pid[DIAL_CLASS].angle.target += SHOOT_SINGLE_ANGLE;
	}
}

/**
 *	@brief	����������ģʽ
 */
void DIAL_TripleShot(void)
{
	if(!dia_info.triple_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		dia_info.triple_init_flag = true;
		dia_info.repeating_init_flag = false;
		dia_info.normal_init_flag = false;
		
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;						
		motor[DIAL].info->angle_sum = 0;

		dial.fire_open = false;
	}
	if(dial.fire_open)
	{
		dial.fire_open = false;
		dia_motor_pid[DIAL_CLASS].angle.target += SHOOT_TRIPLE_ANGLE;
	}
}

/**
 *	@brief	��������ģʽ
 */
void DIAL_RepeatingShot(void)
{
	if(!dia_info.repeating_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		dia_info.repeating_init_flag = true;
		dia_info.normal_init_flag = false;
		dia_info.triple_init_flag = false;
		dia_info.stuck_init_flag = false;
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;						
		motor[DIAL].info->angle_sum = 0;

//		dial.fire_open = false;
	}
	if(dial.fire_open)
	{
		dia_motor_pid[DIAL_CLASS].speed.target = shoot_freq_val;  //
	}
	else
	{
		dia_motor_pid[DIAL_CLASS].speed.target = 0;
	}
}

/**
 *	@brief	���̿�������
 */
void DIAL_StuckHandle(void)      
{
	static uint16_t stuck_finish_cnt = 0;
	if(!dia_info.stuck_init_flag)
	{ //�л�ģʽ���ȳ�ʼ��һ��
		dia_info.stuck_init_flag = true;		
		dia_info.triple_init_flag = false;
		dia_info.normal_init_flag = false;
		dia_info.repeating_init_flag = false;	
		dia_motor_pid[DIAL_CLASS].angle.target = 0;
		dia_motor_pid[DIAL_CLASS].angle.measure = 0;				
		motor[DIAL].info->angle_sum = 0;
	}
	if(dial.stuck && (stuck_finish_cnt == 0))
	{
		dia_motor_pid[DIAL_CLASS].angle.target -= SHOOT_SINGLE_ANGLE;
		
	}

	stuck_finish_cnt++;
	
	if(stuck_finish_cnt > STUCK_HANDLE_TIME)
	{
		stuck_finish_cnt = 0;
		dial.stuck = false;
	}
}

