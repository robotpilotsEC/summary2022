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
/*����λ�û�Kp*/
#define GRID_KP_LARGE 0.5
#define GRID_KP_NORMAL 0.5
#define GRID_KP_LOW 0.4
/*�ܹ�������Ŀ��*/
#define NORMAL_GIVEN_MIN -4000
#define NORMAL_GIVEN_MAX 4000
#define GRID_GIVEN_MIN -4000
#define GRID_GIVEN_MAX 4000


/* Private function prototypes -----------------------------------------------*/
void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);
void CHASSIS_PowerLimit(void);
float AcceleratedSpeed_Cal(void);
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
	
};

// ����ģ�������
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
};

// ����ģ�鴫����
chassis_dev_t		chas_dev = {
	.chas_motor[CHASSIS_CLASS] = &motor[CHASSIS],
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
	
//	chassis_accelerate = AcceleratedSpeed_Cal();  //������ٶ�
	
}

/* Ӧ�ò� --------------------------------------------------------------------*/
/* ����� --------------------------------------------------------------------*/
void CHASSIS_Init(void)
{
	chas_drv[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS]->driver;

	chas_motor[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS];
	
	chas_motor_info[CHASSIS_CLASS] = chas_dev.chas_motor[CHASSIS_CLASS]->info;
	
	chassis.init_mileage = true;
	chassis.rail_mileage = 0;
	chassis.direction = LEFT;
	chas_motor_pid[CHASSIS_CLASS].mode = SPEED_PID;         //�ٶȻ�ģʽ
	/*---��̨�ֿ���---*/
	chassis.check_road = false;
	chassis.control_dir = 0;
	chassis.ctrl_sentry = false;
	chassis.escape = false;
	chassis.test_sentry = false;
	/*---�˺�����---*/
	chassis.hurt_data_update = false;
	/*---����ж��״̬---*/
	chassis.unloading_force = false;
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


	//���̵��ж�����
	if(chassis.unloading_force == true)
	{
		chas_motor_pid[CHASSIS_CLASS].out = 0;
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
//  path_info_t *path_info = path_sensor.info;
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
	static int32_t target_buffer;

//	target_buffer = rand() % (chassis.rail_mileage);   //����Ŀ��(���)  0 ~ chassis.rail_mileage

	target_buffer = GRID_GIVEN_MIN + rand() % (chassis.rail_mileage + (GRID_GIVEN_MAX * 2));   //����Ŀ��(���)  -3000 ~ chassis.rail_mileage + 3000

	//���ط���Ŀ������
//	if(path_info->mileage_dif > 0)
//	{
//		if(((target_buffer - Grid->target) < -(chassis.rail_mileage / 3)) )    //�ж����ɵ��������û�д�����ֵ
//		{
//			random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}	
//		
//	}
//	else if(path_info->mileage_dif < 0)
//	{
//		if(((target_buffer - Grid->target) > (chassis.rail_mileage / 3)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}		
//	}
//	else
//	{
//		if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 3)))    //�ж����ɵ��������û�д�����ֵ
//		{
//			random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
//		}	
//	}
  //ȫ�������
	if((abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 3)))    //�ж����ɵ��������û�д�����ֵ
	{
		random_value = target_buffer;     //��¼��һ��Ŀ��ֵ
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
* @brief �����ܹ�ģʽ-����
* @param void
* @return void
*/
void CHASSIS_Normal(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ

	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;

//	if((abs(Grid->target - Grid->measure)) <= 5000)
//	{
//		if(abs((chassis.rail_mileage + 3000) - Grid->measure) > abs((-3000 - Grid->measure)))	 //�����̷�������
//		{
//			Grid->target = chassis.rail_mileage + 3000;
//		}
//		else if(abs((chassis.rail_mileage + 3000) - Grid->measure) <= abs((-3000 - Grid->measure)))	 //�����̷�������
//		{
//			Grid->target = -3000;
//		}
//	}
	
	if(chassis.direction == LEFT)
	{
		Grid->target = NORMAL_GIVEN_MIN;
	}
	else if(chassis.direction == RIGHT)
	{
		Grid->target = chassis.rail_mileage + NORMAL_GIVEN_MAX;		
	}
	//�޷�
	Grid->target = constrain(Grid->target, NORMAL_GIVEN_MIN, chassis.rail_mileage + NORMAL_GIVEN_MAX);


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
void CHASSIS_Coordinate(void)
{
	chas_motor_pid[CHASSIS_CLASS].mode = POSITION_PID;        //�л���λ�û�ģʽ
	path_info_t *path_info = path_sensor.info;
	pid_ctrl_t *Grid = &chas_motor_pid[CHASSIS_CLASS].angle;
//	static int32_t target_buffer;
	static int32_t next_target;

	
	if(flag_grid)  //��δ���������ʼ��
	{
		flag_grid = false;
		Grid->target = Grid->measure;
	}
	
//	target_buffer = GRID_GIVEN_MIN + rand() % (chassis.rail_mileage + (GRID_GIVEN_MAX * 2));   //����Ŀ��(���)  -3000 ~ chassis.rail_mileage + 3000
	
////���������ֵ
//	if(chassis.direction == LEFT)
//	{
//		
//	}
//	else if(chassis.direction == RIGHT)
//	{
//		
//	}
//	//ȫ�������ֵ
//	if(abs(abs(target_buffer) - abs(Grid->target)) > (chassis.rail_mileage / 4))    //�ж����ɵ��������û�д�����ֵ
//	{
//		next_target = target_buffer;     //��¼��һ��Ŀ��ֵ
//		if(abs(Grid->measure - Grid->target) < 4000)   //����ﵽĿ������� 
//		{
//			Grid->target = next_target; 
//			//�޷�
//			Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);
//		}
//		else if(path_info->left_touch || path_info->right_touch)   //�����������
//		{
//			Grid->target = next_target; 
//			//�޷�
//			Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);			
//		}
//	}

	//ȫ�������ֵ
	next_target = random_value;     //��¼��һ��Ŀ��ֵ
	if(abs(Grid->measure - Grid->target) < 4000)   //����ﵽĿ������� 
	{
		Grid->target = next_target; 
		//�޷�
		Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);
	}
	else if(path_info->left_touch || path_info->right_touch)   //�����������
	{
		Grid->target = next_target; 
		//�޷�
		Grid->target = constrain(Grid->target, GRID_GIVEN_MIN, chassis.rail_mileage + GRID_GIVEN_MAX);			
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
void CHASSIS_Mixed(void)
{
	static uint16_t time_cnt = 0;
	static uint16_t switch_mode = 0;
	uint16_t tempt = 1500 + rand() % (2500 + 1);
	
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
// /**************************************/	

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
	}
	else if(chassis.ctrl_sentry)
	{
		CHASSIS_ControlDir();
	}	
	else if(chassis.escape)
	{
		CHASSIS_Mixed();
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
		}
	}
 /**************************************/	

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
		chassis.unloading_force = true;        //ж��
	}
	else if(path_info->right_touch && stop_access)   //�����Ҷ˵�
	{
		stop_access = 0;
		chassis.direction = LEFT;
		chassis.unloading_force = true;        //ж��
	}
	else if((!path_info->left_touch) && (!path_info->right_touch))//û���ڶ˵�
	{
		stop_access = 1;
		chassis.unloading_force = false;        
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
			/*---���������---*/						
			RandomValue_Generate();
			/*---�Զ��ܹ�---*/		
			CHASSIS_AutoRun();	
			/*---ײ���ж�---*/			
			CHASSIS_ImpactEnd();
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

	
}




void CHASSIS_Ctrl(void)
{
	/*----��Ϣ����----*/
	CHASSIS_GetInfo();
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
