/*
*  ����  __RP_CONFIG_H����CARR_MODE����
*  �������������
*  �������� ���ִ��� ���ִ��� 
*/

#include "Chassis.h"
#include "judge_infantrypotocol.h"
#include "State.h"
#include "arm_math.h"
#include "Super_Ctrl.h"
#include "slave.h"
#include "Power_Limit.h"

extern float Yaw_Turn_Err(void);
	
int16_t	Chassis_Power_Current[4] = {0,0,0,0};
int16_t	Chassis_Turn_Current[4]  = {0,0,0,0};

chassis_t CHASSIS = {
	.Power_Lock = 0,
	.Chas_Time  = 0,
  .Chas_Lock  = 0,
	
	.GIMB_FRI_TURN_RESET = 0,
	
	.Angle_XY = 0,
	.RC_FX    = TURN_X_MAX,
	.BRAKE_FLG= 0,

};

/*	�м�pid�����������仯  */
pid_ctrl_t power_pid_mid[4] = 
{
	[0]=
	{
		.kp = POWER_PID_P,
		.ki = POWER_PID_I,
		.integral_max = POWER_PID_I_MAX,
		.out_max      = POWER_PID_O_MAX,
	},
	[1]=
	{
		.kp = POWER_PID_P,
		.ki = POWER_PID_I,
		.integral_max = POWER_PID_I_MAX,
    .out_max      = POWER_PID_O_MAX,
	},
	[2]=
	{
		.kp = POWER_PID_P,
		.ki = POWER_PID_I,
		.integral_max = POWER_PID_I_MAX,
		.out_max      = POWER_PID_O_MAX,
	},
	[3]=
	{
		.kp = POWER_PID_P,
		.ki = POWER_PID_I,
		.integral_max = POWER_PID_I_MAX,
		.out_max      = POWER_PID_O_MAX,
	},
};







/********************************  ʵʱ��״̬  **************************************/



/*-���̲���-*/	
void CHASSIS_TEST(void)
{
	State.Gimbal_init = OK;
	State.move_mode(&State.Move, CHAS_FIRST);

}

/*-���̽�������-*/	
extern Vision_Cmd_Id_t AUTO_CMD_MODE;
bool CHAS_LOCK(void)
{
	if(RC_ONLINE
	&&(State.Gimbal_init == OK)
  && State.Move.mode != NULL_MOVE)
    return 1;
	else
		return 0;
}

/*-YAWת��pid������ֵ-*/
float Yaw_Turn_Out(void)
{
	float out;//,err;
	
//	/*-����turn_pid �����ڽ���ʱ����kp ���ٳ���-*/
//	err = Yaw_Turn_Err();
//	
//	if		 (CARR_MODE == 0 || CARR_MODE == 1 || CARR_MODE == 3 || CARR_MODE == 4)
//		Gimb_Motor[GIMB_Y].pid->Turn_pid.kd = YAW_TURN_KP * (2/7 + (5/7)*abs(err) / 8191);

//	else if(CARR_MODE == 2)
//		Gimb_Motor[GIMB_Y].pid->Turn_pid.kd = YAW_TURN_KP * (5/7 + (2/7)*abs(err) / 8191);

	out = PID_CAL(&Gimb_Motor[GIMB_Y].pid->Turn_pid, &Gimb_Motor[GIMB_Y].pid->speed, 
                 Gimb_Motor[GIMB_Y].info->angle,    Gimb_Motor[GIMB_Y].info->speed, 1);

	return out;
}

/*-�ж��Ƿ���л���ת��-*/	
char Turn_Slow_Flg(void)
{
	if(CHASSIS.High_Running_Flg
		&& (CHASSIS.Ch2 || CHASSIS.Ch3)
			&&(State.Move.state.GIMB_FIRST == OK
			 || State.Move.state.CHAS_FIRST == OK))
	  return 1;
	else
		return 0;
}

/*-���Ǻ���С����-*/	
float Top_X = 0;
int16_t speed;
float Get_Tri_Top_Val(void)
{
	Top_X++;
	if(Top_X == TOP_X_MAX)Top_X = 0;
	speed = (CHASSIS.TOP_SPEED) + (2*CHASSIS.TOP_SPEED/7)*(arm_sin_f32(PI * (Top_X/TOP_X_MAX)));

  return speed;
}

/*-NO_CTRL��־��CH0ͨ��-*/
bool CHO_NO_Ctrl_Flg(void)
{
	if((State.Ctrl.state.RC == OK && abs(CH0_VALUE)) || 
		(State.Ctrl.state.KEY == OK && abs(CH0_VALUE_K)))
		return 0;
	else
		return 1;
}


/*-NO_CTRL��־��ȫ��ͨ��-*/
bool ALL_NO_Ctrl_Flg(void)
{
	if((State.Ctrl.state.RC == OK && (abs(CH0_VALUE) || abs(CH2_VALUE) || abs(CH3_VALUE))) || 
		(State.Ctrl.state.KEY == OK && (abs(CH0_VALUE_K) || abs(CH2_VALUE_K) || abs(CH3_VALUE_K))))
		return 0;
	else
		return 1;
}

bool CLIMBING(void)
{
	float err;
	if(CARR_MODE == 0 || CARR_MODE == 3 || CARR_MODE == 4)
	{
		err = Half_Turn(0 - SLAVE.pack3.slave_roll, 8191);//ǰ��0 ��0
	}
	else if(CARR_MODE == 1 || CARR_MODE == 2)
	{
		err = Half_Turn(4095 - SLAVE.pack4.slave_pit, 8191);//ǰ��0 ��0
	}
	
	err = abs(err) * 360 / 8192;
	
	if(err > 20)
		return 1;
	else
		return 0;
}


/*-������ߺ�����-*/
float CLIMB(void)
{
	float err;
	if(State.Move.state.TOP != OK || (State.Move.state.GIMB_FIRST == OK && !State.TURN_NORMAL))
	{
		if(CARR_MODE == 0 || CARR_MODE == 3 || CARR_MODE == 4)
		{
			err = Half_Turn(0 - SLAVE.pack3.slave_roll, 8191);//ǰ��0 ��0
			if((State.Move.dire == DIR_F && err > 0) || (State.Move.dire == DIR_B && err < 0))
			{
				err = abs(err) / (8191/6);
				err = constrain(err,0,1);
			}	
			else err = 0;
			
			if(SLAVE.work_state == DEV_OFFLINE || SLAVE.pack2.imu_work_state == DEV_OFFLINE)err = 0;
		}
		else if(CARR_MODE == 1 || CARR_MODE == 2)
		{
			err = Half_Turn(4095 - SLAVE.pack4.slave_pit, 8191);//ǰ��0 ��0
			if((State.Move.dire == DIR_F && err < 0) || (State.Move.dire == DIR_B && err > 0))
			{
				err = abs(err) / (8191/6);
				err = constrain(err,0,1);
			}
			else err = 0;
			
			if(SLAVE.work_state == DEV_OFFLINE || SLAVE.pack4.imu_work_state == DEV_OFFLINE)err = 0;
		}	
	}
	else
		err = 0;
	
	return err;
}



uint32_t wait_time1,wait_time2;
char Wait(void)
{
	char i;

	if(State.Move.state.CHAS_FIRST == OK && CHASSIS.Speed)i = 1;
		
	else{
		
		if(!CHASSIS.XY_FLG)
		{
			wait_time1 = HAL_GetTick();
			i = 0;
		}
		else if(CHASSIS.XY_FLG)
		{
			wait_time2 = HAL_GetTick() - wait_time1;

			if(wait_time2 > 200)i = 1;
			else  i = 0;
		}		
	}
	
	if(CHASSIS.ZZ_FLG && !CHASSIS.XY_FLG)i = 1;
	
	if(Q_FLAG || E_FLAG || C_FLAG)i = 0;
	
	return i;
}


char Get_Turn_State(void)
{
	char res = 0,num = 0;
	
	for(char i = 0; i < 4; i++)	
	{	
		if(Chas_Motor[i+4].work_state == DEV_OFFLINE)num++;
	}
	
	if(num > 1)res = 1;

	return res;
}


char SPEED_OVER_FLG(int16_t speed)
{
	char num = 0,res = 0;
	
	for(char i = 0; i < 4; i++)	
	{	
		if(abs(Chas_Motor[i].info->speed) > speed)
    {
	    if(num < 4)num++;
		}
	}
	
	if     (num > 0)
		      res = 1;
	
	else if(num == 0)
		      res = 0;	

	return res;
}

int16_t ang;
void Top_Out_Mid(void)
{
	int16_t angle = 0;
	motor_t *motor = &Gimb_Motor[GIMB_Y];
	
	angle = motor->info->angle - MEC_MID_Y_F + 2048;
	
	angle = Limit_Target(angle);
	ang = angle;
	if(abs(angle) < 4096)
	{
		motor->pid->Turn_pid.target = MEC_MID_Y_B;	
		State.Move.dire = DIR_B;
	}
	else 
	{
		motor->pid->Turn_pid.target = MEC_MID_Y_F;		
		State.Move.dire = DIR_F;
	}		
}

/*ɲ��*/
extern float W_Chan,
						 S_Chan,
						 A_Chan,
						 D_Chan;
char BRAKE_JUDGE(void)
{
  char res;
//12 43
//34 21
	if(State.Ctrl.state.KEY == OK)
	{
		if(!D_Chan && !A_Chan)
		{
			if(W_Chan && !KEY_W)
			{
				if(State.Move.dire == DIR_F)res = 12;//ǰ�ִ���
				if(State.Move.dire == DIR_B)res = 34;//���ִ���				
			}
			else if(S_Chan && !KEY_S)
			{
				if(State.Move.dire == DIR_F)res = 34;//���ִ���
				if(State.Move.dire == DIR_B)res = 12;//ǰ�ִ���			
			}
			else
				res = 0;
		}
		else if(!W_Chan && !S_Chan)
		{
			if(A_Chan && !KEY_A)
			{
				if(State.Move.dire == DIR_F)res = 13;//���ִ���
				if(State.Move.dire == DIR_B)res = 24;//���ִ���
			}
			else if(D_Chan && !KEY_D)
			{
				if(State.Move.dire == DIR_F)res = 24;//���ִ���
				if(State.Move.dire == DIR_B)res = 13;//���ִ���			
			}
			else
				res = 0;
		}
		
		else if((S_Chan && D_Chan) && (!KEY_S && !KEY_D))
		{
			if(State.Move.dire == DIR_F)res = 23;//���ִ���
			if(State.Move.dire == DIR_B)res = 23;//���ִ���
		}
		else if((W_Chan && D_Chan) && (!KEY_W && !KEY_D))
		{
			if(State.Move.dire == DIR_F)res = 14;//���ִ���
			if(State.Move.dire == DIR_B)res = 14;//���ִ���
		}		
		else if((W_Chan &&A_Chan) && (!KEY_W && !KEY_A))
		{
			if(State.Move.dire == DIR_F)res = 23;//���ִ���
			if(State.Move.dire == DIR_B)res = 23;//���ִ���		
		}
		else if((A_Chan && S_Chan) && (!KEY_A && !KEY_S))
		{
			if(State.Move.dire == DIR_F)res = 14;//���ִ���
			if(State.Move.dire == DIR_B)res = 14;//���ִ���		
		}
		else
			res = 0;
		
	}
	else
		res = 0;


  if(State.Move.state.TOP == OK || State.TURN_45)res = 0;
	
	
	return res;
}

void Brake_Flg(void)
{

	if(SPEED_OVER_FLG(1500))//�ٶȵ���1000��ȡ��ɲ��״̬
	{
		if(!CHASSIS.BRAKE_FLG)//��λ�����жϣ��޸�״̬
		{
			
			CHASSIS.BRAKE_FLG = BRAKE_JUDGE();
		}
	}
	else
		CHASSIS.BRAKE_FLG = 0;
}

void Brake_Ctrl(void)
{
	Brake_Flg();
	
	switch(CHASSIS.BRAKE_FLG)
	{
		case 12:
			Chassis_Power_Current[0] *= 0.5f;
		  Chassis_Power_Current[1] *= 0.5f;
			break;
		case 34:
			Chassis_Power_Current[2] *= 0.5f;
		  Chassis_Power_Current[3] *= 0.5f;			
			break;	
		case 13:
			Chassis_Power_Current[0] *= 0.5f;
		  Chassis_Power_Current[2] *= 0.5f;		
			break;	
		case 24:
			Chassis_Power_Current[1] *= 0.5f;
		  Chassis_Power_Current[3] *= 0.5f;	
			break;	
		case 23:
			Chassis_Power_Current[1] *= 0.5f;
		  Chassis_Power_Current[2] *= 0.5f;			
			break;			
		case 14:
			Chassis_Power_Current[0] *= 0.5f;
		  Chassis_Power_Current[3] *= 0.5f;			
			break;			
			
		default:
			break;	
	}
}	










































/*---------------------------------------------------------------

һ���������״̬����ʵʱ�ԣ����������Ƿ��˶� �Ƿ���������� ���ڸ����˶������ӡ���

---------------------------------------------------------------*/

extern int16_t cap_output_limit;
float top_flg = 0;

void Get_State(void)
{
	
	if(State.WH_SP_MAX == LOW_SP)
	{
		CHASSIS.V_MAX = SPEED_MAX * 0.6f;
		cap_output_limit = CAP_OUTPUT_LIMIT * 0.70f;		
	}	
	else if(State.WH_SP_MAX == HIG_SP)
	{
		CHASSIS.V_MAX = SPEED_MAX;	
		cap_output_limit = CAP_OUTPUT_LIMIT * 1.f;
	}

	if(JUDGE_ONLINE && State.Move.state.TOP == OK)
	{
		if((judge_sensor.info->power_heat_data.chassis_power_buffer > 55 && !CHASSIS.XY_FLG)||KEY_SHIFT)
		{
			if(top_flg > 1000)top_flg = 1000;
			else
				top_flg++;
		}
		else
		{
			if(top_flg <= 0)top_flg = 0;
			else
				top_flg--;

		}

		CHASSIS.TOP_SPEED = TOP_SPEED_MAX + top_flg / 3.f;
		
		if     (CHASSIS.TOP_SPEED < TOP_SPEED_MAX)CHASSIS.TOP_SPEED = TOP_SPEED_MAX;
		else if(CHASSIS.TOP_SPEED > 600          )CHASSIS.TOP_SPEED = 600;	
	}
	else{
	
		top_flg = 0;
		
	}
	
	if(State.Func.state.MAGAZINE == OK || State.CHASS_SLOW_FLAG || KEY_CTRL)
	{

			CHASSIS.V_MAX = SPEED_MAX * 0.15f;
	
	}
		
	CHASSIS.Running_Wheel_Num = 0;
	CHASSIS.Turnning_Wheel_Num = 0;
	CHASSIS.High_Running_Wheel_Num = 0;
	
	/*-���㴦���˶������� ���㴦�ڸ����˶�������-*/
	for(char i = 0; i < 4; i++)	
	{	
		if(abs(Chas_Motor[i].info->speed) > STO_SPEED_VALUE)
    {
	    if(CHASSIS.Running_Wheel_Num < 4)
				 CHASSIS.Running_Wheel_Num++;
		}
		
		if(abs(Chas_Motor[i].info->speed) > HIG_SPEED_VALUE)
		{
			if(CHASSIS.High_Running_Wheel_Num < 4)
				 CHASSIS.High_Running_Wheel_Num++;
		}
	}
	
	/*-�ж��Ƿ����˶�״̬-*/
	if(CHASSIS.Running_Wheel_Num > 0)
		 CHASSIS.Running_Flg = 1;//�����˶�	
	
	else if(CHASSIS.Running_Wheel_Num == 0)
		      CHASSIS.Running_Flg = 0;	

	/*-�ж��Ƿ��ڸ����˶�״̬-*/
	if(CHASSIS.High_Running_Wheel_Num > 0)
		 CHASSIS.High_Running_Flg = 1;//����	
	
	else if(CHASSIS.High_Running_Wheel_Num == 0)
		      CHASSIS.High_Running_Flg = 0;	

#if CARR_MODE	== 1
	
	/*-���㵽λ�Ķ���-*/
	for(char i = 0; i < 4; i++)	
	{	
		if(abs(Chas_Motor[i+4].pid->Mec_Out_Pid.err) > 768)
    {
	    if(CHASSIS.Turnning_Wheel_Num < 4)
				 CHASSIS.Turnning_Wheel_Num++;
		}
	}	
	
	if(CHASSIS.Turnning_Wheel_Num < 2)CHASSIS.TURNNING = 0; 
	else                              CHASSIS.TURNNING = 1; 
	
	//�����ھ�ֹ״̬�ҳ���һ�����������������ֵ�λ������
	//�з��� ��һ���ڿ���ĳһ�ϰ�����ò����ƶ� ����ì��
	if(!CHASSIS.Running_Flg)
	{
		CHASSIS.Chas_Time++;
		if(CHASSIS.Chas_Time > 1000)
			 CHASSIS.Power_Lock = 0;
		
		if(CHASSIS.Turnning_Wheel_Num == 4)
		{
			CHASSIS.Chas_Time  = 0;
			CHASSIS.Power_Lock = 1;
		}
	}
	
	/*-��¼��ʱ�����ֽǶ�-*/
  for(char i = 0; i < 4; i++)
  CHASSIS.NOW_ANGLE[i] = (Chas_Motor[i+4].info->angle);
	
	/*-��̨�����������λ��־��0-*/	
	if(State.Move.state.GIMB_FIRST != OK)CHASSIS.GIMB_FRI_TURN_RESET = 0;

#endif

	/*-���̸�λʱ�����-*/
	if(RC_OFFLINE)CHASSIS.init_time = HAL_GetTick();
	else if(HAL_GetTick() - CHASSIS.init_time > 2000)State.Chassis_init = OK;
	
}	


/***************************  ״̬ **************************************/





























/*----------------------------------

	�����ֿ���

----------------------------------*/

void Chassis_Power_Transmit(int16_t *data)
{
	float OUT[4];
	
	if(State.Move.dire == DIR_F && CHASSIS.Ch3)
	{
		OUT[0] = Chas_Motor[0].pid->out * (1 - CLIMB()/2);
	  OUT[1] = Chas_Motor[1].pid->out * (1 - CLIMB()/2);
		OUT[2] = Chas_Motor[2].pid->out * (1 + CLIMB());
	  OUT[3] = Chas_Motor[3].pid->out * (1 + CLIMB());
	}
	else if(State.Move.dire == DIR_B && CHASSIS.Ch3)
	{
		OUT[2] = Chas_Motor[2].pid->out * (1 - CLIMB()/2);
	  OUT[3] = Chas_Motor[3].pid->out * (1 - CLIMB()/2);
		OUT[0] = Chas_Motor[0].pid->out * (1 + CLIMB());
	  OUT[1] = Chas_Motor[1].pid->out * (1 + CLIMB());
	}
	else
	{
		for(char i = 0 ; i < 4 ; i++)
		{
			OUT[i] = Chas_Motor[i].pid->out;		
		}
	}
	
	for(char i = 0 ; i < 4 ; i++)
	{
		if(Chas_Motor[i].work_state == DEV_ONLINE)
			data[i] = OUT[i];		
		else
			data[i] = 0;
	}

//	for(char i = 0 ; i < 4 ; i++)
//	{
//		if(Chas_Motor[i].work_state == DEV_ONLINE)
//			data[i] = Chas_Motor[i].pid->out;	
//		else
//			data[i] = 0;
//	}
}


void Power_Slow(void)
{
	for(char i = 0; i < 4; i++)
	{
	  err_calculation(&Chas_Motor[i].pid->speed, Chas_Motor[i].info->speed);
		if(CHASSIS.Pre_Speed_Tar[i] != Chas_Motor[i].pid->speed.target)
		{
				CHASSIS.Power_X[i]   = POWER_X_INIT;//��̫С���»������΢��ת
				CHASSIS.Get_Speed[i] = Chas_Motor[i].info->speed;			
				CHASSIS.Get_Speed_Error[i] = Chas_Motor[i].pid->speed.err;		
		}
		else 
		{
			if(CHASSIS.Power_X[i] < 200)
			{
				 CHASSIS.Power_X[i]++;
			}
		}
		
		if(CHASSIS.Power_X[i] < 200)
		{
			power_pid_mid[i].target = 
			CHASSIS.Get_Speed[i] + CHASSIS.Get_Speed_Error[i] * UP_FX(CHASSIS.Power_X[i]);					
		}
		else
			power_pid_mid[i].target = Chas_Motor[i].pid->speed.target;
	}
}



void POWER_SLOW_CTRL(void)
{
	Power_Slow();
	
	for(char i = 0 ; i < 4 ; i++)
	{
		Chas_Motor[i].pid->out = 
		PID_CAL(&power_pid_mid[i],          NULL, 
						 Chas_Motor[i].info->speed, NULL, 0);
	}
}
	
void POWER_FAST_CTRL(void)
{
	for(char i = 0 ; i < 4 ; i++)
	{
		Chas_Motor[i].pid->out =
		PID_CAL(&Chas_Motor[i].pid->speed,  NULL, 
						 Chas_Motor[i].info->speed, NULL, 0);
	}
}	



/*-������pid����-*/
void Chassis_Power_Pid_Calculate(void)
{

#if POWE_MODE == 0
	
	POWER_FAST_CTRL();

#endif
#if POWE_MODE == 1
	
	if(!abs(CHASSIS.XY_FLG) || State.Move.state.TOP == OK)//����youXY����ʱ����SLOW���ƣ������ӻᶶ
		POWER_FAST_CTRL();
	else
		POWER_SLOW_CTRL();		
//	POWER_FAST_CTRL();
#endif
}

/*-----------------------�����ֿ���------------------------*/





/*-----------------------���������-------------------------*/



void Chassis_Power_Output(void)
{	

	Chassis_Power_Pid_Calculate();
	
	Chassis_Power_Transmit(Chassis_Power_Current);
	
	if(CARR_MODE == 1 && !CLIMBING())Brake_Ctrl();

	if(JUDGE_ONLINE)
	{
		Chassis_Motor_Power_Limit(Chassis_Power_Current);
		
  }
	else if(JUDGE_OFFLINE)
	{
		Judge_Offline_Power_Limit(Chassis_Power_Current);
	}
	
	Send_Current(POWER_CAN, POWER_ID, Chassis_Power_Current);

}



/*------------------------���������-----------------------*/

















#if (CARR_MODE == 0 || CARR_MODE == 2 || CARR_MODE == 3 || CARR_MODE == 4)

/************���ִ���begin**************/

void Chassis_MEC_Input(void)
{
	float angle,x,y,speed_max = 0,speed_zz,speed_xy[4],speed[4];
	
  /********--����ͨ��ֵ--********/	
	if(State.Ctrl.state.RC == OK)
  {
		CHASSIS.Ch2 = CH2_VALUE;
		CHASSIS.Ch3 = CH3_VALUE;
		CHASSIS.Ch0 = CH0_VALUE;	
	}
	else if(State.Ctrl.state.KEY == OK)
	{
		CHASSIS.Ch2 = CH2_VALUE_K;
		CHASSIS.Ch3 = CH3_VALUE_K;
		CHASSIS.Ch0 = CH0_VALUE_K;	
	}
	
  /********--����ͨ��ֵ--********/	
	
	/*--��̨���渳ֵ--*/
	if(State.Move.state.GIMB_FIRST == OK
  ||(State.Move.mode == CHAS_FIRST && State.Move.Init == ING))
	{
		
		CHASSIS.Ch0 = Yaw_Turn_Out();

	}
	
	/*--С���ݸ�ֵ--*/
	else if(State.Move.state.TOP == OK)
	{	
		if(State.TOP_MODE == HIG_SPEED)
	    CHASSIS.Ch0 = CHASSIS.TOP_SPEED;
		
		else if(State.TOP_MODE == TRI_SPEED)
			CHASSIS.Ch0 = Get_Tri_Top_Val();

	}
	
	/*--��е��ֵ--*/
  else if(State.Move.state.CHAS_FIRST == OK)
	{

		if(KEY_Q)CHASSIS.Ch0 = -SLOW_SPEED;  //����ת��
		if(KEY_E)CHASSIS.Ch0 = +SLOW_SPEED;  
		
	}
	
	else
	{
		CHASSIS.Ch2 = 0;
		CHASSIS.Ch3 = 0;		
		CHASSIS.Ch0 = 0;			
	}
	
	
  /*********- dir & speed -*********/	
	
	/*-���跽���Լ��ٶ�-*/
	if(State.Move.dire == DIR_F)
	{
		CHASSIS.X =  CHASSIS.Ch2;
		CHASSIS.Y =  CHASSIS.Ch3;
		CHASSIS.Z =  CHASSIS.Ch0;
	}
	else if(State.Move.dire == DIR_B)
	{
		CHASSIS.X = -CHASSIS.Ch2;
		CHASSIS.Y = -CHASSIS.Ch3;
		CHASSIS.Z =  CHASSIS.Ch0;
	}
	
	/*-�ٶȷֽ�-*/
	if(State.Move.state.TOP == OK
	|| State.Move.state.GIMB_FIRST == OK
	||(State.Move.mode == CHAS_FIRST && State.Move.Init == ING))
	{
		angle = Gimb_Motor[GIMB_Y].info->angle \
		      - Gimb_Motor[GIMB_Y].pid->Turn_pid.target;
	
		angle = PI * Limit_Target(angle) / 4095.f;
		
		if(State.TURN_45)angle += PI/4;
			
		x = CHASSIS.X * cos(angle) - CHASSIS.Y * sin(angle);
		y = CHASSIS.X * sin(angle) + CHASSIS.Y * cos(angle);

		CHASSIS.X = x;
		CHASSIS.Y = y;

	}
	
	/*-����ת��-*/
  CHASSIS.V_rate = CHASSIS.V_MAX / 660;
	
	speed_zz = CHASSIS.Z * CHASSIS.V_rate;
		
	speed_xy[0] = ( CHASSIS.X + CHASSIS.Y) * CHASSIS.V_rate;
	speed_xy[1] = ( CHASSIS.X - CHASSIS.Y) * CHASSIS.V_rate;	
	speed_xy[2] = (-CHASSIS.X + CHASSIS.Y) * CHASSIS.V_rate;	
	speed_xy[3] = (-CHASSIS.X - CHASSIS.Y) * CHASSIS.V_rate;
	
	/*-XYת������-*/
	speed_max = 0;
	
	for(char i = 0 ; i < 4 ; i++)
	{
		if(abs(speed_xy[i]) > abs(speed_max))speed_max = speed_xy[i];		
	}
	
	if((abs(speed_max) > (CHASSIS.V_MAX)))
	{
		CHASSIS.V_rate = (CHASSIS.V_MAX) / abs(speed_max);
	}
	else
		CHASSIS.V_rate = 1;

	for(char i = 0 ; i < 4 ; i++)
	{
		speed_xy[i] = speed_xy[i] * CHASSIS.V_rate;
	}
	speed_zz = speed_zz * Get_Symbol(CHASSIS.V_rate);

	speed[0] = speed_xy[0] + speed_zz; 
	speed[1] = speed_xy[1] + speed_zz; 
	speed[2] = speed_xy[2] + speed_zz; 
	speed[3] = speed_xy[3] + speed_zz; 
	
	/*���ٶ�����*/
	speed_max = 0;
	
	for(char i = 0 ; i < 4 ; i++)
	{
		if(abs(speed[i]) > abs(speed_max))speed_max = speed[i];		
	}
	
	if((abs(speed_max) > (SPEED_MAX)))
	{
		CHASSIS.V_rate = (SPEED_MAX) / abs(speed_max);
	}
	else
		CHASSIS.V_rate = 1;
	
	/*-Ŀ��ֵ-*/
	Chas_Motor[POWER_0].pid->speed.target = speed[0] * CHASSIS.V_rate;          
	Chas_Motor[POWER_1].pid->speed.target = speed[1] * CHASSIS.V_rate;
	Chas_Motor[POWER_2].pid->speed.target = speed[2] * CHASSIS.V_rate;
	Chas_Motor[POWER_3].pid->speed.target = speed[3] * CHASSIS.V_rate;
}


/*��������ת����ֹͣ���ƽӿ�*/
void Chassis_Motor_Stop(void)
{
	CHASSIS.Ch2 = 0;
	CHASSIS.Ch3 = 0;
	CHASSIS.Ch0 = 0;	
	
	CHASSIS.V_rate = 0;//����Ҫ
	
	Chas_Motor[POWER_0].pid->speed.target = 0;                 
	Chas_Motor[POWER_1].pid->speed.target = 0;
	Chas_Motor[POWER_2].pid->speed.target = 0;
	Chas_Motor[POWER_3].pid->speed.target = 0;

	/*-ʧ������Ȼ����һ��ʱ��-*/
	Chassis_Power_Output();
}



float speed_0,speed_1,speed_2,speed_3;
float target_0,target_1,target_2,target_3;
void CHASSIS_CTRL(void)
{
	
	speed_0 = abs(Chas_Motor[POWER_0].pid->speed.measure);
	speed_1 = abs(Chas_Motor[POWER_1].pid->speed.measure);
	speed_2 = abs(Chas_Motor[POWER_2].pid->speed.measure);
	speed_3 = abs(Chas_Motor[POWER_3].pid->speed.measure);
		
	target_0 = abs(Chas_Motor[POWER_0].pid->speed.target);
	target_1 = abs(Chas_Motor[POWER_1].pid->speed.target);
	target_2 = abs(Chas_Motor[POWER_2].pid->speed.target);
	target_3 = abs(Chas_Motor[POWER_3].pid->speed.target);	
	
	
	
	if(CHAS_LOCK())
	{		
    /*-״̬-*/		
		Get_State();
    /*-����-*/	
		Chassis_MEC_Input();		
    /*-���-*/	
		Chassis_Power_Output();

	}
}


/************���ִ���end***********************/

#endif



#if CARR_MODE == 1



/**-----------------------���ִ���--------------------------**/




/*************
 *��ת����pid���� 
 *ת��ģʽ��
 *0 ֱ��ת��
 *1 Ѹ��ת��
 *2 ���ת��
 *
 **************/
void Turn_Normal(void)
{
	CHASSIS.USE_TURN_FAST = 0;
	for(char i = 0 ; i < 4 ; i++)
	{
		Chas_Motor[i+4].pid->out = 
		PID_CAL(&Chas_Motor[i+4].pid->Mec_Out_Pid,
         		&Chas_Motor[i+4].pid->Mec_Inn_Pid, 
				     Chas_Motor[i+4].info->angle,
             Chas_Motor[i+4].info->speed, 1);
  }
}

void Turn_Fast(void) 
{
	CHASSIS.USE_TURN_FAST = 1;
	for(char i = 0 ; i < 4 ; i++)
	{
		Chas_Motor[i+4].pid->out = 
		PID_CAL(&Chas_Motor[i+4].pid->Mec_Out_Pid,
         		&Chas_Motor[i+4].pid->Mec_Inn_Pid, 
				     Chas_Motor[i+4].info->angle,
             Chas_Motor[i+4].info->speed, 2);
  }
}


void Chassis_Turn_Pid_Calculate(void)
{

#if TURN_MODE == 0		

	Turn_Normal();
	
#endif
	
#if TURN_MODE == 1		
	
	if(State.Move.state.CHAS_FIRST == OK)
	{
		Turn_Normal();
	}
	else
	{
    Turn_Fast();	
	}
//	Turn_Normal();
	 
#endif		
}

void Chassis_Turn_Transmit(int16_t *data)
{
	for(char i = 0 ; i < 4 ; i++)
	{
		if(Chas_Motor[i+4].work_state == DEV_ONLINE)
			data[i] = Chas_Motor[i+4].pid->out;	
		else
			data[i] = 0;
	}
}


/********
 *
 *��ת�������
 *��ѡ��Ƿ���й��ʿ��ƣ��洫�㷨���ƣ�
 *
 *******/
void Chassis_Turn_Output(void)
{
	Chassis_Turn_Pid_Calculate();
	Chassis_Turn_Transmit(Chassis_Turn_Current);
  
	if(JUDGE_ONLINE)
	{
		if(POWER_CAL == 2)
			Chassis_Motor_Power_Limit(Chassis_Turn_Current);
			
		else if(POWER_CAL == 4)
			Chassis_Turn_Power_Limit(Chassis_Turn_Current);
  }
	else
		Judge_Offline_Power_Limit(Chassis_Turn_Current);

	Send_Current(TURNN_CAN, TURNN_ID, Chassis_Turn_Current);		
	
}


















/***********
 *����Ӧ����3X 2Y 0Z
 *��ͨ�����Ѱ��ת��Ƕ� table
 *
 *�������֣�
 *1ͨ����ֵ
 *2��Ȩ��ֵ
 *3���ͨ���ٶ�
 *4������Ƕ�
 **********/
uint32_t Stop_time, Stop_saty_time;
int table[4] = {-3,+3,-1,+1};
void Get_Str_Value(void)
{
	
	//��¼��һ�ε�ͨ��ֵ�����Ҵ��ڶ��ָ�λ���̽����ᱻ���£��Դ˼�¼�ϴ��˶�����
	if(!CHASSIS.GIMB_FRI_TURN_RESET)
	{
		CHASSIS.CH2_LAST_TURN = CHASSIS.Ch2;
		CHASSIS.CH3_LAST_TURN = CHASSIS.Ch3;
	}
	
/*****************ͨ����ֵ**********************/	
	
	//ң��ģʽ
	if(State.Ctrl.state.RC == OK)
  {
		CHASSIS.Ch2 = CH2_VALUE;
		CHASSIS.Ch3 = CH3_VALUE;
		CHASSIS.Ch0 = CH0_VALUE;	
	}
	//����ģʽ
	else if(State.Ctrl.state.KEY == OK)
	{
		CHASSIS.Ch2 = CH2_VALUE_K;
		CHASSIS.Ch3 = CH3_VALUE_K;
		CHASSIS.Ch0 = CH0_VALUE_K;	
	}
	
/*****************��Ȩ��ֵ**********************/
	
	//С�����¸�ֵ ��̨�����¸�ֵ ��еģʽ ��ʼ���¸�ֵ
	if(State.Move.state.TOP == OK)
	{		
		if(State.TOP_MODE == HIG_SPEED)
	    CHASSIS.Ch0 = CHASSIS.TOP_SPEED;
			
		else if(State.TOP_MODE == TRI_SPEED)
			CHASSIS.Ch0 = Get_Tri_Top_Val();
			
		if(abs(CHASSIS.Ch0) > 660)CHASSIS.Ch0 = 660 * Get_Symbol(CHASSIS.Ch0);

	}

	else if(State.Move.state.GIMB_FIRST == OK)
	{
		//���� !CHASSIS.Ch0��֤�����Զ�����
		if(ALL_NO_Ctrl_Flg() && !CHASSIS.Ch0)//CHASSIS.Running_Flg && 
		{
			if((CHASSIS.CH2_LAST_TURN || CHASSIS.CH3_LAST_TURN) && !CHASSIS.GIMB_FRI_TURN_RESET)
			{
				CHASSIS.GIMB_FRI_TURN_RESET = 1;		
				//��¼��ʼʱ��
				Stop_time = HAL_GetTick();			
			}
			
			Stop_saty_time = HAL_GetTick() - Stop_time;
				
			//ʱ�䵽���˳�
			if(Stop_saty_time > GIMB_RESET_TIME)
			{
				CHASSIS.GIMB_FRI_TURN_RESET = 0;		
			}
		}	
		//��Ϊ���ƽ����˳�
		else
		{
			CHASSIS.GIMB_FRI_TURN_RESET = 0;		
		}
		
		//����
		if(CHASSIS.GIMB_FRI_TURN_RESET)
		{
			CHASSIS.Ch0 = 0;
			CHASSIS.Ch2 = CHASSIS.CH2_LAST_TURN;
			CHASSIS.Ch3 = CHASSIS.CH3_LAST_TURN;	
		}
		else
		{
			if(!State.TURN_NORMAL)
			{
				CHASSIS.Ch0 = 0;//������̨�Ƿ���н��и�ֵ
			}
			else
			{ 
				CHASSIS.Ch0 = Yaw_Turn_Out();
			}			
		}
//			if(!State.TURN_NORMAL)
//			{
//				CHASSIS.Ch0 = 0;//������̨�Ƿ���н��и�ֵ
//			}
//			else
//			{ 
//				if(abs(Yaw_Turn_Out()) < 1.f)
//					CHASSIS.Ch0 = 0;
//				else
//					CHASSIS.Ch0 = Yaw_Turn_Out();
//			}	

	}

	else if(State.Move.mode == CHAS_FIRST && State.Move.Init == ING)
	{
		CHASSIS.Ch0 = Yaw_Turn_Out();
	}
	
  else if(State.Move.state.CHAS_FIRST == OK)
	{
		if(KEY_Q)CHASSIS.Ch0 = -SLOW_SPEED;  //����ת��
		if(KEY_E)CHASSIS.Ch0 = +SLOW_SPEED;  
	}
	
	else
	{
		CHASSIS.Ch2 = 0;
		CHASSIS.Ch3 = 0;		
		CHASSIS.Ch0 = 0;			
	}
	/*---------------------״̬------------------------*/
	
	/*-XYZ��0��־-*/
	CHASSIS.ZZ_FLG = Get_Symbol(CHASSIS.Ch0);	
	CHASSIS.XY_FLG = Get_Symbol(CHASSIS.Ch2 + CHASSIS.Ch3);

	/*-�޿��Ʊ�־λ-*/
	if(abs(CHASSIS.ZZ_FLG) || abs(CHASSIS.XY_FLG))
	{
		CHASSIS.NO_CTRL = 0;
	}			
	else if(!abs(CHASSIS.ZZ_FLG) && !abs(CHASSIS.XY_FLG) && !CHASSIS.Running_Flg)
	{
		CHASSIS.NO_CTRL = 1;
	}			
	
/*****************���ͨ���ٶ�**********************/
	
	/*-�������˶���λ״̬������ٶ�ֵΪ0-*/
	if(CHASSIS.GIMB_FRI_TURN_RESET)
		CHASSIS.V_rate = 0;
	else
		CHASSIS.V_rate = CHASSIS.V_MAX / 660.f;
	
	arm_sqrt_f32(CHASSIS.Ch2 * CHASSIS.Ch2
						 + CHASSIS.Ch3 * CHASSIS.Ch3,
							&CHASSIS.Speed_XY);
	
	CHASSIS.Speed_XY = CHASSIS.Speed_XY;

	CHASSIS.Speed_ZZ = abs(CHASSIS.Ch0);	

	/*-���·����ٶ�-*/
	CHASSIS.Speed = CHASSIS.Speed_XY + CHASSIS.Speed_ZZ;
	
	if(CHASSIS.Speed > 660)
	{	
		if(State.Move.state.TOP == OK)
		{
			CHASSIS.Speed_XY = 660.f - CHASSIS.Speed_ZZ;
			CHASSIS.Speed_ZZ = CHASSIS.Speed_ZZ;
		}
		else
		{
			CHASSIS.Speed_XY = (CHASSIS.Speed_XY / CHASSIS.Speed) * 660.f;
			CHASSIS.Speed_ZZ = (CHASSIS.Speed_ZZ / CHASSIS.Speed) * 660.f;
		}
	}
	
/*****************������Ƕ�**********************/

	CHASSIS.ANGLE_RC_XY = atan2(CHASSIS.Ch3, CHASSIS.Ch2);	

	switch(CHASSIS.ZZ_FLG)
	{
		case  1:
			for(char i = 0; i < 4; i++)
			CHASSIS.Angle_ZZ[i] = PI * table[3-i]/4;			
			break;
		
		case -1:
			for(char i = 0; i < 4; i++)
			CHASSIS.Angle_ZZ[i] = PI * table[i]/4;			
			break;
		
		case  0:
			for(char i = 0; i < 4; i++)
			CHASSIS.Angle_ZZ[i] = 0;			
			break;		
	}
	
}
/***************************************************************************************/









/******************
 *�����нǶȲ���
 *��ң�����ǶȻ����仯����PI/2 ��PI/4��
 *���ǶȲ���
 ******************/

/*-��ò����Ƕ�-*/
void Get_Add_Value(void)
{
  float err, angle_rc_xy;
	
	angle_rc_xy = CHASSIS.ANGLE_RC_XY;
	
	if(State.Move.state.GIMB_FIRST == OK || State.Move.state.TOP == OK)
	{
		err = Half_Turn(Gimb_Motor[GIMB_Y].info->angle \
							    - Gimb_Motor[GIMB_Y].info->MID, 8191);
		
		if(Q_FLAG || E_FLAG || C_FLAG)err = 0;//Ѹ��ת����ⷭ��
		
		CHASSIS.Angle_XY = angle_rc_xy + PI * err / 4095;
		
	}
	else if(State.Move.state.CHAS_FIRST == OK) 
	{
		if     (State.Move.dire == DIR_F)
		{		  
			CHASSIS.Angle_XY = angle_rc_xy;
		}
		else if(State.Move.dire == DIR_B)
		{   		  
      CHASSIS.Angle_XY = angle_rc_xy + PI;
		}
	}
}



void Get_Angle_Value(void)
{
	
	for(char i = 0; i < 4; i++)	
	{
		CHASSIS.TARGET_ANGEL_LAST[i] = CHASSIS.TARGET_ANGEL[i];
		
		CHASSIS.TARGET_ANGEL[i] = 4095.f / PI * \
		atan2(CHASSIS.Speed_XY * arm_sin_f32(CHASSIS.Angle_XY) 
				+ CHASSIS.Speed_ZZ * arm_sin_f32(CHASSIS.Angle_ZZ[i]), \
					CHASSIS.Speed_XY * arm_cos_f32(CHASSIS.Angle_XY) 
				+ CHASSIS.Speed_ZZ * arm_cos_f32(CHASSIS.Angle_ZZ[i]));

		CHASSIS.TARGET_ANGEL[i] = Limit_Target(CHASSIS.TARGET_ANGEL[i] - 2047);	
		
		CHASSIS.TARGET_NOR_ERR[i] = 
		Half_Turn(CHASSIS.TARGET_ANGEL[i] - (Chas_Motor[i+4].info->angle), 8191);
		
		/*-ͨ����Ȧ�жϺ���������Ƚϣ��ó��Ƿ�ת���ٶȷ���-*/
		CHASSIS.TARGET_FAS_ERR[i] = Half_Turn(CHASSIS.TARGET_NOR_ERR[i], 4095);
		
		if(CHASSIS.TARGET_FAS_ERR[i] != CHASSIS.TARGET_NOR_ERR[i])
			 CHASSIS.Speed_Dir[i] = -1;
		else 
			 CHASSIS.Speed_Dir[i] = +1;
		
	}	
}


void Get_Targer_Real(void)
{
	for(char i = 0; i < 4; i++)	
	CHASSIS.TARGET_REAL[i]	= CHASSIS.TARGET_ANGEL[i];
}

void Get_Turn_Value(void)
{
	Get_Angle_Value();
	Get_Targer_Real();

	if(Wait())
	{
		Chas_Motor[TURN_0].pid->Mec_Out_Pid.target = CHASSIS.TARGET_REAL[0];
		Chas_Motor[TURN_1].pid->Mec_Out_Pid.target = CHASSIS.TARGET_REAL[1];
		Chas_Motor[TURN_2].pid->Mec_Out_Pid.target = CHASSIS.TARGET_REAL[2];
		Chas_Motor[TURN_3].pid->Mec_Out_Pid.target = CHASSIS.TARGET_REAL[3];		
	}
}


/**********************
 *����ȡ��������ֵ
 *������õ��ٶ�
 *������õ��ٶȷ���
 * c = atan2(A * sin(a) + B * sin(b),A * cos(a) + B * cos(b);
 *	C^2 =  A^2 + B^2 + 2 * A * B * cos(a - b);
 ***********************/
void Get_Speed_Value(void)
{
	float SP;
	
	for(char i = 0; i < 4; i++)
	{	
		SP =  CHASSIS.Speed_XY * CHASSIS.Speed_XY
		    + CHASSIS.Speed_ZZ * CHASSIS.Speed_ZZ
		+ 2 * CHASSIS.Speed_XY * CHASSIS.Speed_ZZ
		* arm_cos_f32(CHASSIS.Angle_XY - CHASSIS.Angle_ZZ[i]);
		
		arm_sqrt_f32(SP, &CHASSIS.Speed_Val[i]);
	}
}

void Get_Speed_Direc(void)
{
	if(Get_Turn_State())
	{
			CHASSIS.V_rate = 0;
	}
	
	for(char i = 0; i < 4; i++)
	{
		CHASSIS.Pre_Speed_Tar[i] = CHASSIS.Speed_Real[i];

#if TURN_MODE == 0

		CHASSIS.Speed_Real[i] = pow(-1, i) * CHASSIS.Speed_Val[i] * CHASSIS.V_rate;

#endif	
#if TURN_MODE == 1

		CHASSIS.Speed_Real[i] = CHASSIS.Speed_Dir[i] * pow(-1, i) * CHASSIS.Speed_Val[i] * CHASSIS.V_rate;

#endif

	}
	
}



void Get_Power_Value(void)
{
	Get_Speed_Value();
	Get_Speed_Direc();
	
	Chas_Motor[POWER_0].pid->speed.target = CHASSIS.Speed_Real[0];                
	Chas_Motor[POWER_1].pid->speed.target = CHASSIS.Speed_Real[1]; 
	Chas_Motor[POWER_2].pid->speed.target = CHASSIS.Speed_Real[2]; 
	Chas_Motor[POWER_3].pid->speed.target = CHASSIS.Speed_Real[3];	

}

/*---------------------------------------------------------------------------*/



		

/*��������*/
void POWE_CTRL(void)
{
	Get_Power_Value();	
	Chassis_Power_Output();
}

void TURN_CTRL(void)
{
	Get_Turn_Value();
	Chassis_Turn_Output();		
}


/*��������ת���� ����ֵ*/
void Get_Value(void)
{
	Get_Str_Value();	
	Get_Add_Value();
}


/*��������ת���� Ŀ��ֵ*/
void Get_Target(void)
{
	Get_Turn_Value();
	Get_Power_Value();	
}


/*��������ת���� ���ֵ*/
void Send_Out(void)
{
	Chassis_Turn_Output();		

	/*-����Ѹ��ת�� ������ת����� ����-*/
	if(!CHASSIS.TURNNING || CHASSIS.USE_TURN_FAST)
	{
		Chassis_Power_Output();
	}		
}



/*��������ת����ֹͣ���ƽӿ�*/
void Chassis_Motor_Stop(void)
{
	CHASSIS.Ch2 = 0;
	CHASSIS.Ch3 = 0;
	CHASSIS.Ch0 = 0;	

	CHASSIS.V_rate = 0;//����Ҫ
	
	POWE_CTRL();		
}



/*�ܿ���*/
void CHASSIS_CTRL(void)
{	
	if(CHAS_LOCK())
	{
		/*-״̬-*/	
		Get_State();																																																																																											
		/*-����*/	
		Get_Value();
		/*-Ŀ��-*/	
		Get_Target();
		/*���*/
		Send_Out();
	
	}
	
}




/*--------------------------���ִ���end------------------------------*/

#endif

