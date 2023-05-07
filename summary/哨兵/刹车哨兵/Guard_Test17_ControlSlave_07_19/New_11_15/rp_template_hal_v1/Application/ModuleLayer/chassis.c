/**
 * @file        chassis.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       Chassis Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"

#include "chassis.h"
#include "path_sensor.h"
#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"
#include "judge_sensor.h"

/* Private macro -------------------------------------------------------------*/
/*���̵�����ʱ��*/
#define NORMAL_TIME 250
#define GRID_TIME 250
/*���̵�ɲ����ֵ*/
#define BRAKE_THRESHOLD 3000
/*���̵�ɲ������ʱ��*/
#define KEEPING_TIME 30
/*���̵ĸ�ֵ��ֵ*/
#define GIVING_THRESHOLD 3000
/*������ɲ����������ֵ*/
#define STUCK_THRESHOLD 3
/*������ɲ��ʱλ�ñ仯����ֵ*/
#define STUCK_GRID_THRESHOLD 20
/*����λ�û�Kp*/
#define GRID_KP_LARGE 0.6
#define GRID_KP_NORMAL 0.45
#define GRID_KP_LOW 0.4


/* Private function prototypes -----------------------------------------------*/
void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);
void CHASSIS_PowerLimit(void);
float AcceleratedSpeed_Cal(void);
void CHASSIS_BrakeCtrl(void);
void Brake_StuckHandle(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// ���̵����������
drv_can_t				*chas_drv[CHASSIS_MOTOR_CNT];
motor_t			*chas_motor[CHASSIS_MOTOR_CNT];
motor_info_t	*chas_motor_info[CHASSIS_MOTOR_CNT];

bool flag_grid = true;   //�����ʼ����־
bool flag_normal = true;    //����ģʽ��ʼ��
bool normal_init = true;    //��һ�ν�������ģʽ
bool access_encode = false;  //��һ�ν�����Ա������ƶ�ģʽ
float chassis_power_factor = 1;
//�����ж�Ŀ���Ƿ����ı���ɲ��
//float angle_target_pre;
//float angle_target_now;
uint8_t brake_step = 0;
uint16_t time_limit = 25;
uint16_t brake_stuck_cnt = 0;  //��ɲ������
bool brake_stuck = false;   //��ɲ��־
int32_t random_value = 0;


//���̼��ٶ�
float chassis_accelerate = 0;

/* Exported variables --------------------------------------------------------*/
// ���̵��PID������
chassis_motor_pid_t 	chas_motor_pid[CHASSIS_MOTOR_CNT] = {
	[CHASSIS_CLASS] = {
		.speed.kp = 11.75,    //11.75
		.speed.ki = 0.472,    //0.472
		.speed.kd = 0, 
		.speed.integral_max = 10000,
		.speed.out_max = 18000,
		.angle.kp = 0.45,   //0.45   0.6  0.7  0,8
		.angle.ki = 0,
		.angle.kd = 0.008,   //0.008
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	},
	[BRAKE_CLASS] = {
		.speed.kp = 6,
		.speed.ki = 0.02,
		.speed.kd = 0,
		.speed.integral_max = 4000,
		.speed.out_max = 8000,
		.angle.kp = 0.2,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	},
	
};

// ����ģ�������
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
};

// ����ģ�鴫����
chassis_dev_t		chas_dev = {
	.chas_motor[CHASSIS_CLASS] = &motor[CHASSIS],
	.chas_motor[BRAKE_CLASS] = &motor[BRAKE],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// ����ģ����Ϣ
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.local_mode = RUN_NORMAL,
};

chassis_t chassis = {
	.controller = &chas_ctrl,
	.dev = &chas_dev,
	.info = &chas_info,
	.init = CHASSIS_Init,
	.test = CHASSIS_Test,
	.ctrl = CHASSIS_Ctrl,
	.self_protect = CHASSIS_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
 *	@brief	���̵��PID������ʼ��
 */
void CHASSIS_PidParamsInit(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid_val_init(&pid[i].angle);
		pid[i].out = 0;
	}
}

/**
 *	@brief	����ȫ�����ж��
 */
static void CHASSIS_Stop(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<CHASSIS_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
	}
}




/**
 *	@brief	���̵��PID���
 */
static void CHASSIS_PidOut(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<CHASSIS_MOTOR_CNT; i++) {
		if(chas_motor[i]->work_state == DEV_ONLINE) {
            chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
		} 
		else {
            chas_drv[i]->add_halfword(chas_drv[i], 0);
		}
	}    
}

/**
 *	@brief	���̵���ٶȻ�
 */
static void CHASSIS_Speed_PidCalc(chassis_motor_pid_t *pid, chassis_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	���̵��λ�û�
 */
static void CHASSIS_Angle_PidCalc(chassis_motor_pid_t *pid, chassis_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	single_pid_ctrl(&pid[MOTORx].angle);	
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	CHASSIS_Speed_PidCalc(pid,MOTORx);
}


/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/
/**
 *	@brief	���̻�ȡϵͳ��Ϣ
 */
void CHASSIS_GetSysInfo(void)
{
	/*----���Ʒ�ʽ�޸�----*/
	if(sys.remote_mode == RC) {
		chas_info.remote_mode = RC;
	}
//	else if(sys.remote_mode == KEY) {
//		chas_info.remote_mode = KEY;
//	}      //�����ٶ���Ҫ��Ҫʹ�ü��̿��Ƶ���
	else if(sys.remote_mode == AUTO)
	{
		chas_info.remote_mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) 
	{
		chas_info.remote_mode = INSPECTION;
	}
	/*----����ģʽ�޸�----*/
}

/**
 *	@brief	��̨����Ϣ��ȡ
 */
void CHASSIS_GetAerialInfo(void)
{
	if(judge_sensor.info->GameRobotStatus.robot_id == 7)//��ɫid
	{
		if((judge_sensor.info->AerialData.send_id == 6) && (judge_sensor.info->AerialData.receive_id == 7))  //���˻����͸��ڱ�
		{
			if(judge_sensor.info->AerialData.cmd == escape)
			{
				chassis.escape = true;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == check_road)
			{
				chassis.escape = false;
				chassis.check_road = true;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == control_aerial)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = true;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == test_sentry)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = true;

			}
			else
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			chassis.control_dir = judge_sensor.info->AerialData.control_dir;
		}
	}
	else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//��ɫid
	{
		if((judge_sensor.info->AerialData.send_id == 106) && (judge_sensor.info->AerialData.receive_id == 107))  //���˻����͸��ڱ�
		{
			if(judge_sensor.info->AerialData.cmd == escape)
			{
				chassis.escape = true;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == check_road)
			{
				chassis.escape = false;
				chassis.check_road = true;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == control_aerial)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = true;
				chassis.test_sentry = false;

			}
			else if(judge_sensor.info->AerialData.cmd == test_sentry)
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = true;
			}
			else
			{
				chassis.escape = false;
				chassis.check_road = false;
				chassis.ctrl_sentry = false;
				chassis.test_sentry = false;

			}
			chassis.control_dir = judge_sensor.info->AerialData.control_dir;
		}
	}
}

void CHASSIS_UpdateController(void)
{
	static uint8_t dir_cnt_l = 0;
	static uint8_t dir_cnt_r = 0;
	
	chas_motor_pid[CHASSIS_CLASS].speed.measure = chas_motor_info[CHASSIS_CLASS]->speed;
	chas_motor_pid[CHASSIS_CLASS].angle.measure = path_sensor.info->mileage_total;	 //�ñ�������ֵ��Ϊ�⻷
	//�����˶�����
	if(chas_motor_pid[CHASSIS_CLASS].speed.measure < 0)
	{
		dir_cnt_r = 0;
		dir_cnt_l++;
		if(dir_cnt_l == 10)
		{
			chassis.run_dir = LEFT;
			dir_cnt_l = 0;
		}
	}
	else if(chas_motor_pid[CHASSIS_CLASS].speed.measure > 0)
	{
		dir_cnt_l = 0;
		dir_cnt_r++;
		if(dir_cnt_r == 10)
		{
			chassis.run_dir = RIGHT;
			dir_cnt_r = 0;
		}	
	}
	else
	{
		dir_cnt_l = 0;
		dir_cnt_r = 0;
	}
	
	chassis_accelerate = AcceleratedSpeed_Cal();  //������ٶ�
	//2006ɲ�����
	chas_motor_pid[BRAKE_CLASS].speed.measure = chas_motor_info[BRAKE_CLASS]->speed;
	chas_motor_pid[BRAKE_CLASS].angle.measure = chas_motor_info[BRAKE_CLASS]->angle_sum;
	
		
}

/* Ӧ�ò� --------------------------------------------------------------------*/
/* ����� --------------------------------------------------------------------*/
void CHASSIS_Init(void)
{
	chas_drv[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS]->driver;
	chas_drv[BRAKE_CLASS] = chas_dev.chas_motor[BRAKE_CLASS]->driver;

	chas_motor[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS];
	chas_motor[BRAKE_CLASS] = chas_dev.chas_motor[BRAKE_CLASS];
	
	chas_motor_info[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS]->info;
	chas_motor_info[BRAKE_CLASS] = chas_dev.chas_motor[BRAKE_CLASS]->info;
	
	chassis.brake_check = false;   //��2006���ж���й�
	chassis.brake_start = false;	   //��3508���ж���й�
	chassis.brake_finished = true;    //��2006���ж���й�
	chassis.init_mileage = true;
	chassis.rail_mileage = 0;
	chassis.direction = LEFT;
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;         //�ٶȻ�ģʽ
	chas_motor_pid[BRAKE_CLASS].mode = POSITION_PID;         //λ�û�ģʽ
	/*---��̨�ֿ���---*/
	chassis.check_road = false;
	chassis.control_dir = 0;
	chassis.ctrl_sentry = false;
	chassis.escape = false;
	chassis.test_sentry = false;
	/*---�����ܹ�---*/
//	chassis.spare_runnig = false;   //��ʼ��ɲ���ܹ�
	chassis.spare_runnig = true;   //��ʼ�������ܹ�
	/*---�˺�����---*/
	chassis.hurt_data_update = false;
}

void CHASSIS_GetInfo(void)
{
	CHASSIS_GetSysInfo();
	CHASSIS_GetAerialInfo();
	CHASSIS_UpdateController();
}

void CHASSIS_SelfProtect(void)
{
//	chassis_accelerate = AcceleratedSpeed_Cal();  //������ٶ�
	CHASSIS_Stop(chas_motor_pid);
	CHASSIS_PidParamsInit(chas_motor_pid, CHASSIS_MOTOR_CNT);
	CHASSIS_GetInfo();
}

void CHASSIS_PidCtrl(void)
{

	switch(chas_motor_pid[CHASSIS_CLASS].mode)
	{
			// ���̵���ٶȻ�
		case SPEED_PID:	CHASSIS_Speed_PidCalc(chas_motor_pid, CHASSIS_CLASS); break;
			// ���̵��λ�û�	
		case POSITION_PID: CHASSIS_Angle_PidCalc(chas_motor_pid, CHASSIS_CLASS);break;
	}	

	
		switch(chas_motor_pid[BRAKE_CLASS].mode)
		{
				//ɲ������ٶȻ�
			case SPEED_PID:	CHASSIS_Speed_PidCalc(chas_motor_pid, BRAKE_CLASS); break;
				//ɲ�����λ�û�
			case POSITION_PID: CHASSIS_Angle_PidCalc(chas_motor_pid, BRAKE_CLASS);break;
		}		

	//���̵��ж�����
	if(chassis.brake_start == true)
	{
		chas_motor_pid[CHASSIS_CLASS].out = 0;
	}	
	
	//ɲ�����ж�����
	if(chassis.brake_finished || chassis.brake_check)
	{
		chas_motor_pid[BRAKE_CLASS].out = 0;
	}
	// ���̵�������Ӧ
	CHASSIS_PidOut(chas_motor_pid);
}

/**
* @brief ���̼��ٶȲ����
*/
float AcceleratedSpeed_Cal(void)
{
	static int16_t pre_chassis_speed;
	static int16_t now_chassis_speed;
	static float accelerated_speed;
	static uint8_t time_interval = 0;  //����ʱ���� /ms
	
	if(time_interval == 0)   
	{
		now_chassis_speed = motor[CHASSIS].info->speed;
		pre_chassis_speed = now_chassis_speed;
	}
	else if(time_interval == 5)  //10ms
	{
		time_interval = 0;
		now_chassis_speed = motor[CHASSIS].info->speed;		
		accelerated_speed = (now_chassis_speed - pre_chassis_speed) *1.0 / 10;
	}
	
	time_interval++;
//	if(time_interval >= 5)
//	{
//		time_interval = 0;
//		now_chassis_speed = path_info->mileage_total;
//		
//	}
	return accelerated_speed;
}

/**
* @brief ɲ���������
*/
void Brake_Step_Test(void)
{
	static uint8_t step = 0;
	static uint16_t keep_time = 0;
	static uint16_t count_time_test = 0;
	static uint16_t switch_offline_time = 0;
	static bool init_angle = false;
	
	chassis_motor_pid_t *brake_pid = &chas_motor_pid[BRAKE_CLASS];
	if(chassis.brake_start)
	{
		brake_pid->mode = POSITION_PID;
		switch(step)
		{
			case 0: 
			{
				count_time_test = 0;
				switch_offline_time = 0;
				
				motor[BRAKE].info->angle_sum = 0;
				chassis.brake_finished = false;
				chassis.brake_check = false;
				brake_pid->angle.target = brake_pid->angle.measure;
				step = 1;
			}break;
			
			case 1: 
			{
				count_time_test = 0;
				
				if(path_sensor.info->down_touch)
				{
					step = 2;
				}
				else
				{
					brake_pid->angle.target -= 500;
				}
				
				switch_offline_time++;
				if(switch_offline_time >= 1500)  //2s������Ӧ
				{
					switch_offline_time = 0;
					chassis.spare_runnig = true;    //�������÷���
					step = 3;
				}
				else
				{
					chassis.spare_runnig = false;    //
				}
				init_angle = true;
			}break;
			
			case 2: 
			{
				count_time_test = 0;
				
				if(init_angle)
				{
					brake_pid->angle.target = brake_pid->angle.measure;
					init_angle = false;
				}
				switch_offline_time = 0;
				
				brake_pid->angle.target -= 500;
				keep_time++;
				if(keep_time >= KEEPING_TIME)
				{
					keep_time = 0;
					step = 3;
				}
			}break;

			case 3: 
			{
				switch_offline_time = 0;
				chassis.brake_check = true;      //ɲ�����ж��
				
				count_time_test++;
				if(count_time_test > GRID_TIME)
				{
					count_time_test = 0;
					chassis.brake_check = false;      
					chassis.brake_finished = true;      //ж��
					chassis.brake_start = false;
					step = 0;
				}
			}break;
		}
	}
}

/**
* @brief ɲ������
*/
void Brake_Step(void)
{
	static uint16_t keep_time = 0;
	static uint16_t count_time = 0;

	static uint16_t switch_offline_time = 0;
	
	path_info_t *path_info = path_sensor.info;	
	chassis_motor_pid_t *brake_pid = &chas_motor_pid[BRAKE_CLASS];
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	
	switch(brake_step)
	{		
		case 1: 
		{
			keep_time = 0;
			count_time = 0;

			chassis.brake_start = false;
			//�ڶ������ж��Ƿ񵽴�λ�ø���
			if(path_info->mileage_dif > 0)
			{
				if((Grid->target - Grid->measure) <= BRAKE_THRESHOLD)
				{
					brake_step = 2;
//					if(abs(motor[CHASSIS].info->speed) <= 5)  //�ٶ�̫����ʱ����ɲ
//					{
//						brake_step = 0;
//					}
				}
				
			}
			else if(path_info->mileage_dif < 0)
			{
				if((Grid->measure - Grid->target) <= BRAKE_THRESHOLD)
				{
					brake_step = 2;
//					if(abs(motor[CHASSIS].info->speed) <= 5)  //�ٶ�̫����ʱ����ɲ
//					{
//						brake_step = 0;
//					}
				}
			}
			else
			{
				if(abs(Grid->measure - Grid->target) <= BRAKE_THRESHOLD)
				{
					brake_step = 2;
//					if(abs(motor[CHASSIS].info->speed) <= 5)  //�ٶ�̫����ʱ����ɲ
//					{
//						brake_step = 0;
//					}
				}
				
			}
		}break;

		case 2: 
		{
			switch_offline_time = 0;
			
			keep_time = 0;
			count_time = 0;
			//����������ɲ��׼��
			motor[BRAKE].info->angle_sum = 0;
			chassis.brake_start = true;  //���̵��ж��
			chassis.brake_finished = false;  //ɲ�������ʼ����
			chassis.brake_check = false;
			brake_pid->angle.measure = 0;
			brake_pid->angle.target = brake_pid->angle.measure;
			brake_step = 3;
		
		}break;
		
		case 3: 
		{
			keep_time = 0;
			count_time = 0;
		 //���Ĳ���ɲ���˶�
			if(path_sensor.info->down_touch)
			{
				brake_step = 4;
			}
			else
			{
				brake_pid->angle.target -= 500;
			}
			
			switch_offline_time++;
			if(switch_offline_time >= 1500)  //2s������Ӧ
			{
				switch_offline_time = 0;
				chassis.spare_runnig = true;    //�������÷���
				brake_step = 5;
			}
			else
			{
				chassis.spare_runnig = false;    //
			}
		}break;
		
		case 4: 
		{
		 //���岽������ɲ��״ֱ̬������ɲס	
			brake_pid->angle.target -= 500;
			keep_time++;
			//���ݵ����ٶ�������ɲ��ʱ��
			if(keep_time >= 50)   //100ms
			{
				keep_time = 0;
				brake_step = 5;
			}
			count_time = 0;
			if(abs(motor[CHASSIS].info->speed) <= 1000)
			{
				if(keep_time >= KEEPING_TIME)   //300ms
				{
					keep_time = 0;
					brake_step = 5;
				}
			}
			else if(abs(motor[CHASSIS].info->speed) <= 3000)
			{
				if(keep_time >= (KEEPING_TIME - 5))   //300ms
				{
					keep_time = 0;
					brake_step = 5;
				}
			}
			else
			{
				if(keep_time >= (KEEPING_TIME - 10))   //300ms
				{
					keep_time = 0;
					brake_step = 5;
				}
			}
//			
//			if(abs(motor[CHASSIS].info->speed) < 10)
//			{
//				brake_step = 5;
//			}
//			else
//			{
//				brake_pid->angle.target -= 500;
//			}
		}break;

		case 5: 
		{
			switch_offline_time = 0;
		 //��������ɲ������ͷŲ��жϵ����ܷ��˶�				
			count_time++;
			chassis.brake_check = true;      //ɲ�����ж��
			keep_time = 0;
//			if((abs(chassis_accelerate) <= 5) && (count_time > 50))
//			if((abs(chassis_accelerate) <= 80) && (count_time > time_limit))
			if(count_time > time_limit)
			{
				count_time = 0;
				
				chassis.brake_check = false;
				chassis.brake_finished = true;      //ɲ�����
				chassis.brake_start = false;
				brake_step = 0;
			}
		}break;
		
	}

	
}


/*---�����˶�����---*/
/**
* @brief ��ʼѲ��
* @param void
* @return void
* ���ڵ�һ���ܹ�ʱ�Թ�����ȵ�У�����ܹ��ٶȽ�����һ����3����׼�������
*/
static void Cruise_First()
{
		pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;
    static uint8_t step = 0;	/*��ʼ������*/
    if(chassis.init_mileage)		/*δ�������ɳ�ʼ��*/
    {
			chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //�л����ٶȻ�ģʽ
      switch(step)
      {
        case 0:			/*��ʼ����������߿�*/
        {
          if(path_sensor.info->left_touch)		/*��ߵĵ㴥����*/
          {
            Speed->target = 0;
            step = 1;
          }
          else	/*δʶ�� -- �����˶�*/
          {
						chassis.direction = LEFT;
            Speed->target = INITIAL_SPEED * chassis.direction;	/*�����˶�*/
          }
          break;
        }
        case 1:
        {
					if(path_sensor.info->left_touch)
          {
						chassis.direction = RIGHT;
            Speed->target = 100 * chassis.direction;	/*����΢�������ڵ��պò������㴥���ص�λ��*/
          }
          else			/*΢����ɺ���ͣס��������һ�����ڽ׶�*/
          {
            Speed->target = 0;
            path_sensor.info->mileage_total = 0;	/*�������� -- ��ʼ��¼*/
            step = 2;
          }
          break;
        }
        case 2:
        {
          if(path_sensor.info->right_touch)	/*�Ѿ�ʶ���ҹ��*/
          {
            Speed->target = 0;
            step = 3;
          }
          else 		/*δʶ��*/
          {
						chassis.direction = RIGHT;
            Speed->target = INITIAL_SPEED * chassis.direction;			/*�����˶�*/
          }
          break;
        }
        case 3:
        {
          if(path_sensor.info->right_touch)
          {
						chassis.direction = LEFT;	
						Speed->target = 100 * chassis.direction;		/*�Ѿ�ʶ���ҵ㴥���أ���������΢�����պ�δʶ�𵽵�״̬*/
          }
          else			/*΢�����*/
          {
						Speed->target = 0;															/*ֹͣ�˶�*/
            chassis.rail_mileage = path_sensor.info->mileage_total;	/*��¼�������*/
            step = 0;																							/*��λstep*/
            chassis.init_mileage = false;											/*���±�־λ*/
            chassis.direction = LEFT;                        //�����˶���־λ
//                Chassis_process.Spot_taget = 0;//λ�û�
//                Chassis_process.getchange_flag = true;
          }
          break;
        }
      }
    }
}

/**
* @brief �����ܹ�ģʽ-��ֹ
* @param void
* @return void
*/
void CHASSIS_Static(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ
  
	Grid->target = chassis.rail_mileage *3.0/ 4;    //ͣ���Ҷ�
	
	brake_step = 0;   //����ɲ������
	chassis.brake_check = false;
	chassis.brake_finished = true;  
	chassis.brake_start = false;

	if(!flag_grid)
	{
		flag_grid = true;  //��������ģʽ��ʼ����־
	}

	if(!flag_normal)
	{
		flag_normal = true;  //��������ģʽ��ʼ����־
	}
	
  //����λ�û�Kp
	Grid->kp = GRID_KP_NORMAL;  //Kp = 0.45
}

/**
* @brief �����ܹ�ģʽ-�鿴��·
* @param void
* @return void
*/
void CHASSIS_CheckRoad(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ

	Grid->target = chassis.rail_mileage / 4;    //ͣ�����
	

	brake_step = 0;   //����ɲ������
	chassis.brake_check = false;
	chassis.brake_finished = true;  
	chassis.brake_start = false;

	if(!flag_grid)
	{
		flag_grid = true;  //��������ģʽ��ʼ����־
	}

	if(!flag_normal)
	{
		flag_normal = true;  //��������ģʽ��ʼ����־
	}
	
  //����λ�û�Kp
	Grid->kp = GRID_KP_NORMAL;  //Kp = 0.45

}

/**
* @brief �����ܹ�ģʽ-��̨�ֲٿ�
* @param void
* @return void
*/
void CHASSIS_ControlDir(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //�л����ٶȻ�ģʽ
	
	if(chassis.control_dir == LEFT)
	{
		Speed->target -= 5;
	}
	else if(chassis.control_dir == RIGHT)
	{
		Speed->target += 5;
	}
		//�޷�
	Speed->target = constrain(Speed->target, -8000, 8000);



	brake_step = 0;   //����ɲ������
	chassis.brake_check = false;
	chassis.brake_finished = true;  
	chassis.brake_start = false;

	if(!flag_grid)
	{
		flag_grid = true;  //��������ģʽ��ʼ����־
		Speed->target = 0;

	}

	if(!flag_normal)
	{
		flag_normal = true;  //��������ģʽ��ʼ����־
		Speed->target = 0;
	}
	
  //����λ�û�Kp
	Grid->kp = GRID_KP_NORMAL;  //Kp = 0.45

}

/**
* @brief �����������ֵ����
*/
void RandomValue_Generate(void)
{
  path_info_t *path_info = path_sensor.info;
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	static int32_t target_buffer;

	target_buffer = rand() % (chassis.rail_mileage);   //����Ŀ��(���)  0 ~ chassis.rail_mileage

	if(path_info->mileage_dif > 0)
	{
		if(((target_buffer - Grid->target) < -(chassis.rail_mileage / 3)) )    //�ж����ɵ��������û�д�����ֵ
		{
			random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
		}	
		
	}
	else if(path_info->mileage_dif < 0)
	{
		if(((target_buffer - Grid->target) > (chassis.rail_mileage / 3)))    //�ж����ɵ��������û�д�����ֵ
		{
			random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
		}		
	}
	else
	{
		if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 3)))    //�ж����ɵ��������û�д�����ֵ
		{
			random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
		}	
	}
	
}


/**
* @brief �����ܹ�ģʽ-����
* @param void
* @return void
*/
void CHASSIS_Normal(void)
{
//	static float grid_target_buf = 0;  //�ݴ�����Ŀ��ֵ
//	static float grid_target_now = 0;	  //��ǰĿ��ֵ
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ

	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

	chassis_motor_pid_t *brake_pid = &chas_motor_pid[BRAKE_CLASS];
	brake_pid->mode = POSITION_PID;
	//��ʼ����־λ
	if(flag_normal)
	{
		flag_normal = false;
//		brake_step = 5;
	}
	
	if(!flag_grid)
	{
		flag_grid = true;  //���������ʼ����־,Ϊ��һ���л�����ģʽ��׼��
//		brake_step = 1;
	}
	
	if(normal_init)
	{
		//����ʼֵ		
//		grid_target_buf = 10000;
		Grid->target = chassis.rail_mileage * 1.0 / 4;
		normal_init = false;
	}
	
	//��������
	switch(brake_step)
	{
		case 0: 
		{
			chassis.brake_start = false;
			chassis.brake_finished = true;
			//��һ������ֵ
			if(abs((chassis.rail_mileage -8000) - Grid->measure) > abs((8000 - Grid->measure)))	 //�����̷�������
			{
//				grid_target_buf = chassis.rail_mileage -8000;
				Grid->target = chassis.rail_mileage -8000;
				brake_step = 1;
			}
			else if(abs((chassis.rail_mileage -8000) - Grid->measure) <= abs((8000 - Grid->measure)))	 //�����̷�������
			{
//				grid_target_buf = 8000;		
				Grid->target = 8000;
				brake_step = 1;
			}
//			//�޷�
//			grid_target_buf = constrain(grid_target_buf, 8000, chassis.rail_mileage - 8000);
			//�޷�
			Grid->target = constrain(Grid->target, 8000, chassis.rail_mileage - 8000);

		}break;
		
	}
	
//	grid_target_now = Grid->target;
//	//б�£���ֹ���ִ�
//	Grid->target = RampFloat(grid_target_buf,grid_target_now,abs(grid_target_buf - grid_target_now) / 300.f);
	
	time_limit = NORMAL_TIME;
  //����λ�û�Kp
	Grid->kp = GRID_KP_LARGE;  //Kp = 0.6

}


/**
* @brief �����ܹ�ģʽ-����
* @param void
* @return void
*/
void CHASSIS_Coordinate(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ
//  path_info_t *path_info = path_sensor.info;

	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
//	static int32_t target_buffer;
//	static int32_t next_target;

	
	if(flag_grid)  //��δ���������ʼ��
	{
		flag_grid = false;
//		brake_step = 1;
	}
	
	if(!flag_normal)
	{
		flag_normal = true;  //��������ģʽ��ʼ����־
//		brake_step = 1;
	}

	if(normal_init)
	{
////		//����ʼֵ		
		Grid->target = chassis.rail_mileage * 1.0 / 2;
		normal_init = false;
	}

//	target_buffer = rand() % (chassis.rail_mileage);   //����Ŀ��(���)  0 ~ chassis.rail_mileage

	/*--����������С---*/
//	if(path_info->mileage_dif > 0)
//	{
//		if(((target_buffer - Grid->target) < -(chassis.rail_mileage / 5)) && ((target_buffer - Grid->target) > -(chassis.rail_mileage *4.0 / 5)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}	
//		
//	}
//	else if(path_info->mileage_dif < 0)
//	{
//		if(((target_buffer - Grid->target) > (chassis.rail_mileage / 5)) && ((target_buffer - Grid->target) < (chassis.rail_mileage *4.0 / 5)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}		
//	}
//	else
//	{
//		if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 5)) && (abs(abs(target_buffer) - abs(Grid->target)) < (chassis.rail_mileage *4.0/ 5)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}	
//	}
//	/*--������С---*/
//	if(path_info->mileage_dif > 0)
//	{
//		if(((target_buffer - Grid->target) < -(chassis.rail_mileage / 2)) )    //�ж����ɵ��������û�д�����ֵ
//		{
//			next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}	
//		
//	}
//	else if(path_info->mileage_dif < 0)
//	{
//		if(((target_buffer - Grid->target) > (chassis.rail_mileage / 2)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}		
//	}
//	else
//	{
//		if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 2)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}	
//	}


	//��������
	switch(brake_step)
	{
		case 0:
		{
			chassis.brake_start = false;
			chassis.brake_finished = true;

			if(abs(Grid->measure - Grid->target) < GIVING_THRESHOLD)   //����ﵽĿ������� 
			{
				Grid->target = random_value; 
				//�޷�
				Grid->target = constrain(Grid->target, 0, chassis.rail_mileage);
//				Grid->target = constrain(Grid->target, 0, chassis.rail_mileage);
				
				brake_step = 1;
			}
			else if(path_sensor.info->left_touch || path_sensor.info->right_touch)   //�������Ҷ�
			{
				Grid->target = random_value; 
				//�޷�
				Grid->target = constrain(Grid->target, 0, chassis.rail_mileage);	
//				Grid->target = constrain(Grid->target, 0, chassis.rail_mileage);	
				
				brake_step = 1;
			}
		}break;
	}
	
	
	time_limit = GRID_TIME;

	  //����λ�û�Kp
	Grid->kp = GRID_KP_LARGE;  //Kp = 0.6

}

/**
* @brief �����ܹ�ģʽ-���
* @note  �����ܹ�ӱ������ܹ�
* @param void
* @return void
*/
void CHASSIS_Mixed(void)
{
	static uint16_t time_cnt = 0;
	static uint16_t switch_mode = 0;
	uint16_t tempt = 1500 + rand() % (3000 + 1);
	
	time_cnt++;
		
	if(time_cnt < tempt)
	{
		switch(switch_mode)
		{
			case 0: CHASSIS_Normal(); break;
			case 1: CHASSIS_Coordinate(); break;
		}	
	}
	else
	{
		time_cnt = 0;
		tempt = 1500 + rand() % (3000 + 1);   //����һ��1500~3000�������
		
		if(switch_mode)
		{
			switch_mode = 0;
		}
		else
		{
			switch_mode = 1;
		}
	}
}

/**
* @brief ���̰�ȫ״̬�ж�
* @param void
* @return void
*/
void IS_Safety_Status(void)
{
	static uint16_t Cur_HP;          //��ǰѪ��
	static uint16_t Pre_HP = 600;     //��ʼѪ��Ϊ���Ѫ����600     
	static uint16_t Deducted_HP = 0;    //�ѿ�Ѫ��
	static uint16_t HP_Cnt = 0;	    //Ѫ������ʱ
	static uint16_t security_delay = 0;      //�ָ�����ȫ״̬���ӳ�
	static uint16_t Pre_HP1 = 600;   //��ʼѪ��Ϊ���Ѫ����600 
	
	Cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;  //���µ�ǰѪ��
	
	if((chassis.hurt_data_update)&&(judge_sensor.info->RobotHurt.hurt_type == 0))   //�ܵ��˺�
	{
		chassis.hurt_data_update = false;  //���ñ�־λ
		if(Pre_HP1 - Cur_HP >= 100)     //���������
		{
			chassis.safety_status = HUGE_DANGER;
		}
		else
		{
			chassis.safety_status = DANGER;
		}
		security_delay = 0;
	}
	else          //û�ܵ��˺�,3s������û�ܵ��˺���ָ���ȫ״̬
	{
		if(chassis.safety_status != SAFETY)
		{
			security_delay++;
			if(security_delay == 3000)       //6s
			{
				security_delay = 0;        //��ʱ����
				chassis.safety_status = SAFETY;
			}
		}
	}
	
	Pre_HP1 = Cur_HP;
	
	HP_Cnt++;
	if(HP_Cnt == 1500)     //ÿ3s���һ��
	{
		Deducted_HP = Pre_HP - Cur_HP;  //�����ѿ�Ѫ��
		if(Deducted_HP >= 100)        //3s�ڿ���100Ѫ����
		{
			chassis.safety_status = HUGE_DANGER;	
		}
		else if(Deducted_HP >= 50)   //3s�ڿ���50Ѫ
		{
			chassis.safety_status = DANGER;			
		}
		HP_Cnt = 0;
	}
	Pre_HP = Cur_HP;
}

/**
* @brief ����ɲ������(λ�û�ɲ��)
* @param void
* @return void
*/
void CHASSIS_BrakeCtrl(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
  
	if(chassis.spare_runnig == false)
	{
		Grid->out_max = 8000.f;
	}
  /*ɲ����ʼ*/
	Brake_Step();	  
}

/**
* @brief �����Զ��ܹ�ģʽ
* @param void
* @return void
*/
static void CHASSIS_AutoRun(void)
{	
// /*������*/
//	switch(RC_SW1_VALUE)
//	{
//		case RC_SW_UP: chas_info.local_mode = RUN_MIXED; break;
//		case RC_SW_MID: chas_info.local_mode = RUN_NORMAL; break;
//		case RC_SW_DOWN: chas_info.local_mode = RUN_COORDINATE; break;
//	}
	
	 /*-------������-------*/
	 
//	/*---�����޸�---*/
//	switch(chas_info.local_mode)
//	{
//		/*---����ģʽѲ��,������ײ���ܶ�---*/
//		case RUN_NORMAL:
//		{
//			CHASSIS_Normal(); 
//		}break;
//		/*---����ģʽ�ܹ�---*/
//		case RUN_COORDINATE:
//		{
//			CHASSIS_Coordinate();
//		}break;
//		/*---���ģʽ�ܹ�---*/		
//		case RUN_MIXED:
//		{
//			CHASSIS_Mixed();
//		}break;
//	}
//		/*---ɲ����ɲ����---*/	
//		Brake_StuckHandle();
	

 /*����ʱ����ע��(������) */ 
	
	/*--- ����Ѫ�����ѡ���Ӧ�ܹ�ģʽ(������) ---*/
	IS_Safety_Status();       //��ȫ״̬�ж�
	
	switch(chassis.safety_status)   //���ݰ�ȫ״̬ѡ���Ӧ��ģʽ
	{
		case SAFETY: chas_info.local_mode = RUN_COORDINATE; break;    
		case DANGER: chas_info.local_mode = RUN_MIXED; break;
		case HUGE_DANGER: chas_info.local_mode = RUN_MIXED; break;	
	}
	/* ��̨�ֿ��� */
	if(chassis.check_road)
	{
		CHASSIS_CheckRoad();
	}
	else if(chassis.test_sentry)   //���ԣ���Ӧ��ѵ����
	{
		CHASSIS_Coordinate();
		/*---ɲ����ɲ����---*/	
		Brake_StuckHandle();
	}
	else if(chassis.ctrl_sentry)
	{
		CHASSIS_ControlDir();
	}	
	 /* ��̨�ֲ����� */
	else 
	{
		if(judge_sensor.info->EventData.outpost == 1)        // ǰ��ս���
		{
			CHASSIS_Static();         //ͣ�����Ҷ�
		}
		else if((RC_THUMB_WHEEL_VALUE >= 50) && (RC_THUMB_WHEEL_VALUE < 600))      //ѵ��ʱģ���ò���ģ��ǰ��ս״̬������ʱ��Ҫע��
		{
			CHASSIS_Static();         //ͣ�����Ҷ�
		}
		else if(RC_THUMB_WHEEL_VALUE >= 600)      //�����ã����ڲ�������ܹ�
		{
			CHASSIS_Coordinate();
			/*---ɲ����ɲ����---*/	
			Brake_StuckHandle();
		}
		else if(judge_sensor.info->EventData.outpost == 0)             //ǰ��ս������
		{
			switch(chas_info.local_mode)
			{
				/*---����ģʽѲ��,������ײ���ܶ�---*/
				case RUN_NORMAL:
				{
					CHASSIS_Normal(); 
				}break;
				/*---����ģʽ�ܹ�---*/
				case RUN_COORDINATE:
				{
					CHASSIS_Coordinate();
				}break;
				/*---���ģʽ�ܹ�---*/		
				case RUN_MIXED:
				{
					CHASSIS_Mixed();
				}break;
			}
			/*---ɲ����ɲ����---*/	
			Brake_StuckHandle();
		}
	}

}

/**
* @brief ����ײ���ж�
* @param void
* @return void
*/
void CHASSIS_ImpactEnd(void)
{
	path_info_t *path_info = path_sensor.info;
	static uint8_t stop_access = 1;
	
	//ײ��ɲ���ж� 
	if(path_info->left_touch && stop_access)   //������˵�        
	{	
		stop_access = 0;
		chassis.direction = RIGHT;
//		if(brake_step == 1)
//		{
////			brake_step = 2;       //ǿ��ɲ��
//		}
		if(chassis.spare_runnig == false)    //ɲ���ܹ�
		{
			brake_step = 2;       //ǿ��ɲ��
		}

	}
	else if(path_info->right_touch && stop_access)   //�����Ҷ˵�
	{
		stop_access = 0;
		chassis.direction = LEFT;
//		if(brake_step == 1)
//		{
////			brake_step = 2;     //ǿ��ɲ��
//		}
		if(chassis.spare_runnig == false)    //ɲ���ܹ�
		{
			brake_step = 2;       //ǿ��ɲ��
		}
	}
	else if((!path_info->left_touch) && (!path_info->right_touch))//û���ڶ˵�
	{
		stop_access = 1;
	}
}

/**
* @brief ������ɲ����
*/
void Brake_StuckHandle(void)
{
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;

	static uint16_t cnt = 0;    //���ݸ���ʱ����
	static uint16_t stuck_time = 0;   //��ɲ�ĳ���ʱ��
	static uint16_t handle_time = 0;   //����ʱ��
	static float last_measure = 0;   //�ϴμ�¼���ڱ�λ��
	static float now_measure = 0;   //��ǰ�ڱ�λ��
	static float speed_target = 0;  //��ɲʱ���̷�ת���ٶ�
	
  cnt++;
	//ÿ��һ��ʱ�����һ��λ������
	if(cnt >= 50)   //100ms
	{
		last_measure = now_measure;
		now_measure = Grid->measure;
		cnt = 0;
	}
	//��̨��û�����ڱ���ǰ��ս������ʱ������ɲ���	
	if(!brake_stuck)
	{
		if(!chassis.check_road && !chassis.ctrl_sentry && (judge_sensor.info->EventData.outpost == 0))
		{
			if(abs(now_measure - last_measure) < STUCK_GRID_THRESHOLD)
			{
				stuck_time++;
			}
			else
			{
				stuck_time = 0;
			}
			
			if(stuck_time > 150)  //������ɲ
			{
				stuck_time = 0;
				brake_stuck_cnt++;
				brake_stuck = true;
				
				if(Grid->target - Grid->measure > 0)
				{
					speed_target = -2000;  //�����˶�
				}
				else if(Grid->target - Grid->measure < 0)
				{
					speed_target = 2000;   //�����˶�
				}
			}
		}
	}
	else
	{
		chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //�л����ٶȻ�ģʽ	
		Speed->target = speed_target;
		handle_time++;
		if(handle_time > 250)
		{
			handle_time = 0;
			brake_stuck = false;
		}			
	}
}

/****************************************************************************/
/*---�����ܹ�---*/

/**
* @brief �����ܹ�ģʽ-����(����)
* @param void
* @return void
*/
void CHASSIS_Normal_B(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ

	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

	if((abs(Grid->target - Grid->measure)) <= 5000)
	{
		if(abs((chassis.rail_mileage -5000) - Grid->measure) > abs((5000 - Grid->measure)))	 //�����̷�������
		{
			Grid->target = chassis.rail_mileage -5000;
		}
		else if(abs((chassis.rail_mileage -5000) - Grid->measure) <= abs((5000 - Grid->measure)))	 //�����̷�������
		{
			Grid->target = 5000;
		}
	}
	//�޷�
	Grid->target = constrain(Grid->target, 5000, chassis.rail_mileage - 5000);


	if(!flag_grid)
	{
		flag_grid = true;  //���������ʼ����־,Ϊ��һ���л�����ģʽ��׼��
	}
	
  //����ɲ��,����λ�û�Kp
	Grid->kp = GRID_KP_LOW;  //Kp = 0.4

}


/**
* @brief �����ܹ�ģʽ-����(����)
* @param void
* @return void
*/
void CHASSIS_Coordinate_B(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ

	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	static int32_t target_buffer;
	static int32_t next_target;

	
	if(flag_grid)  //��δ���������ʼ��
	{
		flag_grid = false;
		Grid->target = Grid->measure;
	}
	
//	target_buffer += RC_RIGH_CH_LR_VALUE / 660.f * 100;
//	Grid->target += RC_RIGH_CH_LR_VALUE / 660.f * 100;
//	Grid->target = target_buffer;
	
	target_buffer = 7000 + rand() % (chassis.rail_mileage - (7000 * 2));   //����Ŀ��(���)  7000 ~ chassis.rail_mileage-7000

	if(abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 4))    //�ж����ɵ��������û�д�����ֵ
	{
		next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
		if(abs(Grid->measure - Grid->target) < 4000)   //����ﵽĿ������� 
		{
			Grid->target = next_target; 
			//�޷�
			Grid->target = constrain(Grid->target, 7000, chassis.rail_mileage - 7000);
		}
//		else if(path_sensor.info->left_touch || path_sensor.info->right_touch)   //�������Ҷ�
//		{
//			Grid->target = next_target; 
//			//�޷�
//			Grid->target = constrain(Grid->target, 7000, chassis.rail_mileage - 7000);	
//		}
	}
	
  //����ɲ��,����λ�û�Kp
	Grid->kp = GRID_KP_LOW;  //Kp = 0.4

}


/**
* @brief �����ܹ�ģʽ-���(����)
* @note  �����ܹ�ӱ������ܹ�
* @param void
* @return void
*/
void CHASSIS_Mixed_B(void)
{
	static uint16_t time_cnt = 0;
	static uint16_t switch_mode = 0;
	uint16_t tempt = 1500 + rand() % (2500 + 1);
	
	time_cnt++;
		
	if(time_cnt < tempt)
	{
		switch(switch_mode)
		{
			case 0: CHASSIS_Normal_B(); break;
			case 1: CHASSIS_Coordinate_B(); break;
		}	
	}
	else
	{
		time_cnt = 0;
		tempt = 1500 + rand() % (2500 + 1);   //����һ��1500~2500�������
		
		if(switch_mode)
		{
			switch_mode = 0;
		}
		else
		{
			switch_mode = 1;
		}
	}
}

/**
* @brief �����Զ��ܹ�ģʽ(����)
* @param void
* @return void
*/
static void CHASSIS_AutoRun_B(void)
{
//	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ

	//ɲ��������ʼ��
	brake_step = 0;
	chassis.brake_check = false;
	chassis.brake_finished = true;  
	chassis.brake_start = false;

/////* ����ʱ����ע��
//	switch(RC_SW1_VALUE)
//	{
//		case RC_SW_UP: chas_info.local_mode = RUN_MIXED; break;
//		case RC_SW_MID: chas_info.local_mode = RUN_NORMAL; break;
//		case RC_SW_DOWN: chas_info.local_mode = RUN_COORDINATE; break;
//	}
  /*-------������-------*/
//	switch(chas_info.local_mode)
//	{
//		/*---����ģʽѲ��,������ײ���ܶ�---*/
//		case RUN_NORMAL:
//		{
//			CHASSIS_Normal_B(); 
//		}break;
//		/*---����ģʽ�ܹ�---*/
//		case RUN_COORDINATE:
//		{
//			CHASSIS_Coordinate_B();
//		}break;
//		/*---���ģʽ�ܹ�---*/		
//		case RUN_MIXED:
//		{
//			CHASSIS_Mixed_B();
//		}break;
//	}
/**************************************/

// ����ʱ����ע�� */ 
	
	/*--- ����Ѫ�����ѡ���Ӧ�ܹ�ģʽ(������) ---*/
	IS_Safety_Status();       //��ȫ״̬�ж�
	
	switch(chassis.safety_status)   //���ݰ�ȫ״̬ѡ���Ӧ��ģʽ
	{
		case SAFETY: chas_info.local_mode = RUN_COORDINATE; break;    
		case DANGER: chas_info.local_mode = RUN_MIXED; break;
		case HUGE_DANGER: chas_info.local_mode = RUN_MIXED; break;	
	}
	  /* ��̨�ֿ��� */
		if(chassis.check_road)
		{
			CHASSIS_CheckRoad();
		}
		else if(chassis.test_sentry)   //���ԣ���Ӧ��ѵ����
		{
			CHASSIS_Coordinate_B();
		}
		else if(chassis.ctrl_sentry)
		{
			CHASSIS_ControlDir();
		}	
		/* ��̨�ֲ����� */
		else            
		{
			if(judge_sensor.info->EventData.outpost == 1)        // ǰ��ս���
			{
				CHASSIS_Static();         //ͣ�����Ҷ�
			}
			else if((RC_THUMB_WHEEL_VALUE >= 50) && (RC_THUMB_WHEEL_VALUE < 600))      //ѵ��ʱģ���ò���ģ��ǰ��ս״̬������ʱ��Ҫע��
			{
				CHASSIS_Static();         //ͣ�����Ҷ�
			}
			else if(RC_THUMB_WHEEL_VALUE >= 600)      //���ڱ�����������ܹ�,������
			{
				CHASSIS_Coordinate_B();        //ȫ������ܹ�
			}
			else if(judge_sensor.info->EventData.outpost == 0)             //ǰ��ս������
			{
//				CHASSIS_Coordinate_B();        //ȫ������ܹ�
				switch(chas_info.local_mode)
				{
					/*---����ģʽѲ��,������ײ���ܶ�---*/
					case RUN_NORMAL:
					{
						CHASSIS_Normal_B(); 
					}break;
					/*---����ģʽ�ܹ�---*/
					case RUN_COORDINATE:
					{
						CHASSIS_Coordinate_B();
					}break;
					/*---���ģʽ�ܹ�---*/		
					case RUN_MIXED:
					{
						CHASSIS_Mixed_B();
					}break;
				}
			}
		}
///**************************************/	

}

/**
* @brief ����ɲ������(λ�û�ɲ��)�����ã�
* @param void
* @return void
*/
void CHASSIS_BrakeCtrl_B(void)
{
	float driving_gear_speed;   //���������ٶ�
	float drive_gear_speed;    //�Ӷ������ٶ�
	float factor_buffer;
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

	driving_gear_speed = chas_motor_info[CHASSIS_CLASS]->speed * 3.14159f * 2 * R_DRIVING_GEAR / 60 ;
	drive_gear_speed = path_sensor.info->mileage_dif * 10 / 1000.f;   //��λm/s
	
	if(driving_gear_speed == 0)
	{
		factor_buffer = 1;
	}
	else
	{
		factor_buffer = abs(drive_gear_speed) / (abs(driving_gear_speed));  
	}
	factor_buffer = constrain(factor_buffer,0,1);   //�����޷�
	//�⻷����޷���������Ҫ����
	if((sign(driving_gear_speed) * sign(drive_gear_speed)) == -1)  //�����ٶ��෴������
	{
//		Grid->out_max = 8000.f * factor_buffer * factor_buffer;              
		Grid->out_max = 8000.f * factor_buffer * factor_buffer;              
		
	}
	else
	{
		Grid->out_max = 8000.f;
	}
}


/**
* @brief ����ң��ģʽ
* @param void
* @return void
*/
void CHASSIS_RcCtrl(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //�л����ٶȻ�ģʽ
	chas_motor_pid[CHASSIS_CLASS].speed.target = RampFloat(rc_sensor.info->ch2 / 660.f * 9000, chas_motor_pid[CHASSIS_CLASS].speed.target, 20);

	brake_step = 0;
	chassis.brake_check = false;
	chassis.brake_finished = true;  
	chassis.brake_start = false;

	access_encode = true;
	
	if(!flag_grid)
	{
		flag_grid = true;  //��������ģʽ��ʼ����־
	}

	if(!flag_normal)
	{
		flag_normal = true;  //��������ģʽ��ʼ����־
	}
	
}

void CHASSIS_KeyCtrl(void)
{

}

/**
* @brief �����Զ�ģʽ
* @param void
* @return void
*/
void CHASSIS_AutoCtrl(void)
{	
	pid_ctrl_t *Speed = &chas_motor_pid[CHASSIS_CLASS].speed;

	access_encode = true;
	
	if(RC_SW1_VALUE == RC_SW_DOWN)   //������������˾ͱ��ֵ��̲���
	{
		chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;        //�л����ٶȻ�ģʽ
		Speed->target = 0;
		chassis.init_mileage = true;
	}
	else     //�����˶�
	{
		/*---��һ���ܹ��ʼ��---*/	
		Cruise_First();
		
		if(chassis.init_mileage == false)  //Ѳ����ʼ�����
		{
			if(chassis.spare_runnig == false)    //ɲ���ܹ�
			{
				/*---���������---*/						
				RandomValue_Generate();
				/*---�Զ��ܹ�---*/		
				CHASSIS_AutoRun();	
				/*---ɲ������---*/
				CHASSIS_BrakeCtrl();
				/*---ײ���ж�---*/			
				CHASSIS_ImpactEnd();
			}
			else if(chassis.spare_runnig == true)  //������ɲ���ܹ�,�����ܹ�
			{
				/*---�Զ��ܹ�---*/		
				CHASSIS_AutoRun_B();
//				/*---���ٴ���---*/			
//				CHASSIS_BrakeCtrl_B();
				/*---ײ���ж�---*/			
				CHASSIS_ImpactEnd();
			}
		}
	}
}

/**
* @brief �����Լ�ģʽ
* @param void
* @return void
*/
void CHASSIS_InspectionCtrl(void)
{
	static bool left_access = false;
	static bool right_access = false;
	
  path_info_t *path_info = path_sensor.info;
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

	if(RC_SW1_VALUE == RC_SW_DOWN)
	{
		if(access_encode)
		{
			Grid->target = Grid->measure;
			access_encode = false;
		}
		Grid->target += rc_sensor.info->ch2 / 6.f;

		 //��翪�ؼ��
		if(path_info->left_touch)
		{
			if(left_access)
			{
				left_access = false;
				Grid->target += 3000;
			}
		}
		else if(path_info->right_touch)
		{
			if(right_access)
			{
				right_access = false;
				Grid->target += -3000;
			}
		}
		else
		{
			left_access = true;
			right_access = true;
		}
	}
	else
	{
		access_encode = true;
	}
//	brake_step = 0;
//	chassis.brake_check = false;
//	chassis.brake_finished = true;  
//	chassis.brake_start = false;

	if(!flag_grid)
	{
		flag_grid = true;  //��������ģʽ��ʼ����־
	}

	if(!flag_normal)
	{
		flag_normal = true;  //��������ģʽ��ʼ����־
	}

	
	//ɲ�����ؼ��
	switch(RC_SW1_VALUE)
	{
		case RC_SW_UP: 
		{
			Brake_Step_Test();
		}break;
		case RC_SW_MID: 
		{
			chassis.brake_start = true;
		}break;
		case RC_SW_DOWN: 
		{
			chassis.brake_start = false;
			chassis.brake_finished = true;  //ɲ�����ж��
		}break;
	}
}

/**
* @brief ɲ���������Ƿ�ʧ��
*/
void BrakeMotor_Inspection(void)
{
	if(motor[BRAKE].work_state == DEV_OFFLINE)
	{
		chassis.spare_runnig = true;
	}
}


/**
* @brief �����ܹ�ģʽ�л�  ����<->ɲ��
*/
void Mode_Switch(void)
{
	static uint8_t ACCESS = 0;
	static uint8_t ACCESS1 = 0;
	
	//�ֶ��л�
	if((RC_THUMB_WHEEL_VALUE <= -600) && (ACCESS))
	{
		ACCESS = 0;
		if(!chassis.spare_runnig)
		{
			chassis.spare_runnig = true;
		}
		else if(chassis.spare_runnig)
		{
			chassis.spare_runnig = false;
		}
	}
	else if(RC_THUMB_WHEEL_VALUE == 0)
	{
		ACCESS = 1;
	}
	//��̨���л�
	if(chassis.escape && ACCESS1)
	{
		if(!chassis.spare_runnig)
		{
			chassis.spare_runnig = true;
		}
		else if(chassis.spare_runnig)
		{
			chassis.spare_runnig = false;
		}
		ACCESS1 = 0;
	}
	else if(!chassis.escape)
	{
		ACCESS1 = 1;
	}
}

/**
* @brief �����ܹ�ģʽ�Զ��л�  ����<->ɲ��
*/
void Mode_Switch_Auto(void)
{
	if((brake_stuck_cnt > STUCK_THRESHOLD) && (!brake_stuck))  //ǿ���л���ʽ
	{
		chassis.spare_runnig = true;   //һֱ���ڲ���ɲ�����ܹ췽ʽ
	}
}

void CHASSIS_Ctrl(void)
{
	/*----��Ϣ����----*/
	CHASSIS_GetInfo();
	/*---����ģʽ�ֶ��л�---*/
	Mode_Switch();
	/*---����ģʽ�Զ��л�---*/
	Mode_Switch_Auto();
	/*----�����޸�----*/
	if(chas_info.remote_mode == RC) {
		CHASSIS_RcCtrl();
	}
	else if(chas_info.remote_mode == KEY) {
		CHASSIS_KeyCtrl();
	}
	else if(chas_info.remote_mode == AUTO) {
		CHASSIS_AutoCtrl();
	}
	else if(chas_info.remote_mode == INSPECTION) {
		CHASSIS_InspectionCtrl();
	}
	/*---���̹�������---*/
	CHASSIS_PowerLimit();
	/*---ɲ��������---*/
	BrakeMotor_Inspection();
	
	/*----�������----*/
	CHASSIS_PidCtrl();	
}

void CHASSIS_Test(void)
{
    
}

/**
* @brief ���̹��ʿ���
* @param void
* @return void
* PID������ = (chassis_power_factor )^2 *  Speed->out_max, ����ϵ��Ϊ ��ʣ�໺������/60J
*/
void CHASSIS_PowerLimit(void)
{
	uint16_t Real_PowerBuffer;
	pid_ctrl_t *Speed  = &chas_motor_pid[CHASSIS_CLASS].speed;
	
		 //�����㷨
	switch(judge_sensor.work_state)
	{
		case DEV_ONLINE: 
		{
			if(judge_sensor.info->GameStatus.game_progress == 1)   //׼���׶�
			{
				chassis_power_factor = 0.3;
			}
			else
			{
				Real_PowerBuffer = judge_sensor.info->PowerHeatData.chassis_power_buffer;
				if(Real_PowerBuffer < 60)
				{
					chassis_power_factor = Real_PowerBuffer / 60.f; //0-1
				}
				else
				{
					chassis_power_factor = 1;  //1	
				}					
			}
		}break;
	
		case DEV_OFFLINE:
		{
			chassis_power_factor = 0.5;
		}break;	
	}
	
	Speed->out_max = (chassis_power_factor) * 18000.f;		
}
