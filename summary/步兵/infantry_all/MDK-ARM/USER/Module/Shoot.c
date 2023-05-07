/*
	发射机构相关功能函数
	使用can2发送，统一Shot_Current发送
*/

#include "Shoot.h"
#include "judge_infantrypotocol.h"
#include "State.h"
#include "Gimbal.h"
#include "stm32f4xx_hal.h"

void Task_Speed(void);
extern judge_sensor_t	judge_sensor;

int16_t Shot_Current[4] = {0,0,0,0};
int16_t Shot_Stop_Cmd[4] = {0,0,0,0};

void Barrel_Change(char dir);
//25-125
























/*========================================================================================================================================*/


/*系统功能状态初始化*/

void FRIC_INIT(void)
{
	//跳变沿改变摩擦轮状态
	if(State.Func.state.FRICTION == NO)
	   State.Func.state.FRICTION =  OK;
	
	else if(State.Func.state.FRICTION == OK)
		      State.Func.state.FRICTION =  NO;
}
//单发初始化 重置标志位
void SHOT_INIT(void)
{
	State.Func.state.SHOT_ONCE = OK;
}

void Get_Shoot_Pid(bool mode)
{
	if(!mode)
	{
		Shot_Motor[BOX].pid->speed.kp = 3.f;
		Shot_Motor[BOX].pid->speed.ki = 0.5f;
		Shot_Motor[BOX].pid->speed.kd = 0.f;
		Shot_Motor[BOX].pid->speed.integral_max = 5000.f;
		Shot_Motor[BOX].pid->speed.out_max = 10000.f;
		Shot_Motor[BOX].pid->speed.blind_err = 0.f;
	}
	else if(mode)
	{
		Shot_Motor[BOX].pid->speed.kp = 6.f;
		Shot_Motor[BOX].pid->speed.ki = 0.5f;
		Shot_Motor[BOX].pid->speed.kd = 0.f;
		Shot_Motor[BOX].pid->speed.integral_max = 5000.f;
		Shot_Motor[BOX].pid->speed.out_max = 10000.f;
		Shot_Motor[BOX].pid->speed.blind_err = 0.f;
	}
}

void Get_Barrel_Pid(bool mode)
{
	if(!mode)
	{
		Shot_Motor[BOX].pid->speed.kp = 3.f;
		Shot_Motor[BOX].pid->speed.ki = 0.5f;
		Shot_Motor[BOX].pid->speed.kd = 0.f;
		Shot_Motor[BOX].pid->speed.integral_max = 5000.f;
		Shot_Motor[BOX].pid->speed.out_max = 10000.f;
		Shot_Motor[BOX].pid->speed.blind_err = 0.f;
	}
	else if(mode)
	{
		Shot_Motor[BOX].pid->speed.kp = 6.f;
		Shot_Motor[BOX].pid->speed.ki = 0.8f;
		Shot_Motor[BOX].pid->speed.kd = 0.f;
		Shot_Motor[BOX].pid->speed.integral_max = 5000.f;
		Shot_Motor[BOX].pid->speed.out_max = 10000.f;
		Shot_Motor[BOX].pid->speed.blind_err = 0.f;
	}
}

/*-判断摩擦力速度是否达到-*/
bool Fric_Err_OK(void)
{
	if((abs(Shot_Motor[FRIC_L].pid->speed.err) < 700 && Shot_Motor[FRIC_L].pid->speed.target && Shot_Motor[FRIC_L].work_state == DEV_ONLINE) &&
	   (abs(Shot_Motor[FRIC_R].pid->speed.err) < 700 && Shot_Motor[FRIC_R].pid->speed.target && Shot_Motor[FRIC_R].work_state == DEV_ONLINE))
	  return 1;
	else
		return 0;
}
	
extern bool Change_Flg;
bool Barrel_OK(void)
{
	bool res = 0;
	
	if(CARR_MODE != 3){
		
		res = 1;
	
	}
	else if(CARR_MODE == 3){
	
		if(State.Barrel == OK && !Change_Flg && Shot_Motor[BARREL].work_state == DEV_ONLINE){
		
			res = 1;
			
		}
		else{
		
			res = 0;
		
		}
	
	}
	else
		res = 1;

	return res;
}




/*---------------------------------------------------------------------*/
/*---------------------------------------------------------------------*/
/*------------------------------INPUT----------------------------------*/
//4680,5000,6900
//int16_t sp = 3000;6900

void Friction_Input(int16_t v)
{
	char temR,temL;

	int16_t tem_err_L,tem_err_R;
	
	/*获得温度误差 30度为准*/
	temL = Shot_Motor[FRIC_L].info->temperature;
	temR = Shot_Motor[FRIC_R].info->temperature;
		
	tem_err_L = temL - 30;
	tem_err_R = temR - 30;
	/*温度处理*/
	
	if(v == 0)
	{
		Shot_Motor[FRIC_L].pid->speed.target = 0;
		Shot_Motor[FRIC_R].pid->speed.target = 0;
	}
	else
	{
//		v = 2000;
		Shot_Motor[FRIC_L].pid->speed.target = -(v - Temp * tem_err_L/10);
		Shot_Motor[FRIC_R].pid->speed.target =  (v - Temp * tem_err_R/10);			
	}

}

void Box_Stay_Input(int16_t v)
{
	if(COOLING_HEAT_ALLOW || JUDGE_OFFLINE)
	{
		Shot_Motor[BOX].pid->speed.target = -v;	
	}
	else 
	{
		Shot_Motor[BOX].pid->speed.target = 0;	
	}		
}

void Box_Once_Input(void)
{
	if(COOLING_HEAT_ALLOW || JUDGE_OFFLINE)
	{
		Shot_Motor[BOX].info->angle_sum = 0;
		Shot_Motor[BOX].pid->angle.target = -8191*4.5f ;
		State.Func.state.SHOT_ONCE = ING;
	}
	else
	{
		Shot_Motor[BOX].pid->angle.target = 
		Shot_Motor[BOX].info->angle_sum;		
		State.Func.state.SHOT_ONCE = ING;
	}
	
}

//+ 顺
void Barrel_Speed_Input(int16_t v)
{
	
	Shot_Motor[BARREL].pid->speed.target = v;	

}

void Barrel_Stay_Input(int32_t angle)
{
	Shot_Motor[BARREL].pid->angle.target = angle;
}



/*=====================================================================================================*/
/*=====================================================================================================*/
/*======================================OUTPUT=========================================================*/

void Friction_Output(void)
{
	Shot_Motor[FRIC_L].pid->out = 
	PID_CAL(&Shot_Motor[FRIC_L].pid->speed, NULL,
					 Shot_Motor[FRIC_L].info->speed, NULL ,0);
	
	Shot_Motor[FRIC_R].pid->out = 
	PID_CAL(&Shot_Motor[FRIC_R].pid->speed, NULL,
					 Shot_Motor[FRIC_R].info->speed, NULL ,0);
		
	
	Shot_Current[Shot_Motor[FRIC_L].driver->drv_id] = 
		 Shot_Motor[FRIC_L].pid->out;
	
	Shot_Current[Shot_Motor[FRIC_R].driver->drv_id] = 
		 Shot_Motor[FRIC_R].pid->out;

}
	
void Box_Stay_Output(void)
{
	Shot_Motor[BOX].pid->out = 
	PID_CAL(&Shot_Motor[BOX].pid->speed, NULL,
					 Shot_Motor[BOX].info->speed, NULL ,0);
	
	Shot_Current[Shot_Motor[BOX].driver->drv_id] = 
		Shot_Motor[BOX].pid->out;
	
}

void Box_Once_Output(void)
{
	Shot_Motor[BOX].pid->out = 
	PID_CAL(&Shot_Motor[BOX].pid->angle,
        	&Shot_Motor[BOX].pid->speed,
					 Shot_Motor[BOX].info->angle_sum,
         	 Shot_Motor[BOX].info->speed ,0);
	
	Shot_Current[Shot_Motor[BOX].driver->drv_id] = 
		Shot_Motor[BOX].pid->out;

	if(abs(Shot_Motor[BOX].pid->angle.err) < 5)
	{
		State.Func.state.SHOT_ONCE = NO;
	}
}

char Box_tar_set = 0;
void Box_Stuck_Input(void)
{
	if((!Box_tar_set) && (Shot_Motor[BOX].pid->angle.target < 0 || Shot_Motor[BOX].pid->speed.target < 0))
	{ 
		Get_Shoot_Pid(1);
		Shot_Motor[BOX].info->angle_sum = 0;
		Shot_Motor[BOX].pid->angle.target = +8191 * 4.5f ;

		State.Func.state.SHOT_STUCK = ING;
		Box_tar_set = 1;//1
	}
	
	else if((!Box_tar_set) && (Shot_Motor[BOX].pid->angle.target > 0))
	{
		Get_Shoot_Pid(1);
		Shot_Motor[BOX].info->angle_sum = 0;
		Shot_Motor[BOX].pid->angle.target = -8191 * 4.5f ;
		
		State.Func.state.SHOT_STUCK = ING;
		Box_tar_set = 1;
	}
}

void Box_Stuck_Output(void)
{
	if(Box_tar_set == 1)
	{
		Shot_Motor[BOX].pid->out = 
		PID_CAL(&Shot_Motor[BOX].pid->angle,
						&Shot_Motor[BOX].pid->speed,
						 Shot_Motor[BOX].info->angle_sum,
						 Shot_Motor[BOX].info->speed ,0);
	}

	Shot_Current[Shot_Motor[BOX].driver->drv_id] = 
	Shot_Motor[BOX].pid->out;

	if(abs(Shot_Motor[BOX].pid->angle.err) < 10)	
	{
		State.Func.state.SHOT_STUCK = NO;
		Box_tar_set = 0;			
	}

}


void Barrel_Speed_Output(void)
{
//  Get_Barrel_Pid(1);
	Shot_Motor[BARREL].pid->out = 
	PID_CAL(&Shot_Motor[BARREL].pid->speed, NULL,
					 Shot_Motor[BARREL].info->speed, NULL ,0);
	
	Shot_Current[Shot_Motor[BARREL].driver->drv_id] = 
		Shot_Motor[BARREL].pid->out;

}

void Barrel_Stay_Output(void)
{
//  Get_Barrel_Pid(0);
	Shot_Motor[BARREL].pid->out = 
	PID_CAL(&Shot_Motor[BARREL].pid->angle,
        	&Shot_Motor[BARREL].pid->speed,
					 Shot_Motor[BARREL].info->angle_sum,
         	 Shot_Motor[BARREL].info->speed ,0);	
	
	Shot_Current[Shot_Motor[BARREL].driver->drv_id] = 
		Shot_Motor[BARREL].pid->out;

}

/*=====================================================================================================*/





































/*-------------------------------------------------------------------------------------------------------
COVERRRRRRRRR
COVERRRRRRRRR
COVERRRRRRRRR
COVERRRRRRRRR
COVERRRRRRRRR
-------------------------------------------------------------------------------------------------------*/
void Cover_Open(void)
{
	if(CARR_MODE == 3)
		COVER_PwmOut(50);//180
	else
		COVER_PwmOut(180);//180
}


void Cover_Close(void)
{
	if(CARR_MODE == 3)
		COVER_PwmOut(215);//250
	else
		COVER_PwmOut(250);//250
}





/*-------------------------------------------------------------------------------------------------------
BARRELLLLLLL
BARRELLLLLLL
BARRELLLLLLL
BARRELLLLLLL
BARRELLLLLLL
-------------------------------------------------------------------------------------------------------*/
//0 1 2
//枪管切换方式
void Barrel_Change(char dir)
{
	switch(dir){

		case 0:	
		Barrel_Stay_Output();		
		break;	
		
		case 1:	
		Barrel_Speed_Input(+2000);
		Barrel_Speed_Output();		
		break;	
		
		case 2:		
		Barrel_Speed_Input(-2000);
		Barrel_Speed_Output();		
		break;	
		
		case 3:		
		Barrel_Speed_Input(0);
		Barrel_Speed_Output();		
		break;		
	
	}
	
}

uint32_t Barrel_Stop_Time;
uint32_t Barrel_Change_Time;
uint32_t Barrel_Start_Time;

int32_t Barrel_Fri_Tar;
int32_t Barrel_Sec_Tar;

bool Change_Flg = 0;
char Barrel_State = 0;

void Barrel(void)
{
	if(Barrel_State == 1){
	
		State.Barrel = NO;
	  State.Shoot_init = NO;
		State.Barrel_Wise = STAY;	
	  Barrel_State = 0;
		
	}
	
	if(Shot_Motor[BARREL].work_state != DEV_ONLINE){
		
		State.Barrel = NO;
	  State.Shoot_init = NO;
		State.Barrel_Wise = STAY;

	}
	
	//枪管未准备好
	if(State.Barrel == NO){
	
		Change_Flg = 0;
		

		if(State.Barrel_Wise != CLOCK){
		
			State.Barrel_Wise = CLOCK;
			
		}
		else if(State.Barrel_Wise != ANTICLOCK){
		
			State.Barrel_Wise = ANTICLOCK;		
		
		}
		
		State.Barrel = ING;
		Barrel_Stop_Time = HAL_GetTick();
		Barrel_Start_Time = HAL_GetTick();
	}
	//枪管准备中
	else if(State.Barrel == ING){
	
		//未初始化
		if(!State.Shoot_init){
			
			if(State.Barrel_Wise == CLOCK){
			
				Barrel_Change(1);
	
				
			}
			else if(State.Barrel_Wise == ANTICLOCK){
				
				Barrel_Change(2);	
			
			}
			
			if(Motor_Stucking(&Shot_Motor[BARREL])){
			
				if(HAL_GetTick() - Barrel_Stop_Time > 100){
					
					Barrel_Stay_Input(Shot_Motor[BARREL].info->angle_sum);
					
					if(State.Barrel_Wise == CLOCK){
					
						Barrel_Fri_Tar = Shot_Motor[BARREL].info->angle_sum;
						State.Barrel = NO;
					
					}
					else if(State.Barrel_Wise == ANTICLOCK){
					
						Barrel_Sec_Tar = Shot_Motor[BARREL].info->angle_sum;
					  State.Barrel = OK;
						
						if(Barrel_Fri_Tar != 0 || Barrel_Sec_Tar != 0)State.Shoot_init = OK;
						else{
							State.Barrel = NO;
							State.Barrel_Wise = STAY;
						}
					}				
				}
			}
			else{
			
				Barrel_Stop_Time = HAL_GetTick();
			
			}	
		}
		//已初始化
		else if(State.Shoot_init){
			
			if(State.Barrel_Wise == CLOCK){
			
				Barrel_Stay_Input(Barrel_Fri_Tar);
				Barrel_Change(0);
				
			}
			else if(State.Barrel_Wise == ANTICLOCK){
				
				Barrel_Stay_Input(Barrel_Sec_Tar);
				Barrel_Change(0);	
			
			}
      
			if(abs(Shot_Motor[BARREL].pid->angle.err) < 1500 || HAL_GetTick() - Barrel_Start_Time > 500)State.Barrel = OK;
			
		}
	}
	//枪管已经准备好
	else if(State.Barrel == OK){
		
		Barrel_Change(0);	
		
		//热量不足
		if(((!BARREL_1_ALLOW && COOLING_HEAT_OK_2) || (!BARREL_2_ALLOW && COOLING_HEAT_OK_1)) && !Change_Flg){
			
			Barrel_Change_Time = HAL_GetTick();
			Change_Flg = 1;			
			
		}
		
		//切换延时
		if(Change_Flg && (HAL_GetTick() - Barrel_Change_Time) > 100){
		
			State.Barrel = NO;
			Change_Flg = 0;
		
		}
	}
}

void Barrel_Offline(void)
{
	if(Barrel_State == 0 || Shot_Motor[BARREL].work_state != DEV_ONLINE){
	
		State.Barrel = NO;
	  State.Shoot_init = NO;
		State.Barrel_Wise = STAY;	
	  Barrel_State = 1;
		
	}
	
	Change_Flg = 0;
	
	if(State.Barrel == NO){
	
		Barrel_Stop_Time = HAL_GetTick();
		State.Barrel = ING;

	}
	else if(State.Barrel == ING){
	
		Barrel_Change(2);	
		
		if(Motor_Stucking(&Shot_Motor[BARREL])){
		
			if(HAL_GetTick() - Barrel_Stop_Time > 100){
				
				Barrel_Stay_Input(Shot_Motor[BARREL].info->angle_sum);
				State.Barrel = OK;
				State.Shoot_init = OK;
				
			}
		}	
		else{
		
			Barrel_Stop_Time = HAL_GetTick();
		
		}
	}
	else if(State.Barrel == OK){
		
		Barrel_Change(0);	
		
	}
}

//卸力控制
void Barrel_Unload(void)
{

	Shot_Motor[BARREL].pid->out = 0;
	
	Shot_Current[Shot_Motor[BARREL].driver->drv_id] = 
		Shot_Motor[BARREL].pid->out;

}






/*-------------------------------------------------------------------------------------------------------
FRICTIONNNN
FRICTIONNNN
FRICTIONNNN
FRICTIONNNN
FRICTIONNNN
-------------------------------------------------------------------------------------------------------*/
float shoot_limit,

	    get_pre_shoot_sp,
      get_pre_shoot_limit;
      
int16_t FRI_SPEED_TAR = 0,
			  speed_high_flg = 0,
        shoot_adopt_flg = 0;

char speed_limit_change = 0,
	
     speed_low_num = 0,
		 hig_adopt_flg = 0,
	   low_adopt_flg = 0;


void Fri_Speed_Add(void)
{

	if(CARR_MODE == 3 && BARREL_2_ONLINE){
	
		if(State.Barrel_Wise == ANTICLOCK)shoot_limit = SHOOT_SPEED_LIMIT;
		
		else if(State.Barrel_Wise == CLOCK)shoot_limit = SHOOT_SPEED_LIMIT_ADD;
	
	}
	else
		shoot_limit = SHOOT_SPEED_LIMIT;
	
	
	if(get_pre_shoot_limit != shoot_limit)
	{
		speed_limit_change = 1;
		
		low_adopt_flg = 0;
		hig_adopt_flg = 0;
		
		speed_low_num = 0;
	}
	
	if(get_pre_shoot_sp != SHOOT_SPEED)
	{
		speed_limit_change = 0;
	}
	

	if(get_pre_shoot_sp != SHOOT_SPEED && !speed_limit_change)
	{	
		if     (SHOOT_SPEED < (shoot_limit - 2.f))
		{
			speed_low_num++;
		}		
		
		/*超速判断*/ 		/*低速判断*/
		if(shoot_limit < SHOOT_SPEED || speed_low_num == 3){
		
			speed_high_flg = speed_high_flg + (shoot_limit - SHOOT_SPEED - 1) * 120;
			speed_low_num = 0;
			
		}
		
		/*适应判断*/			
		if     (SHOOT_SPEED < (shoot_limit - 1.05f))
		{
			low_adopt_flg++;
			hig_adopt_flg = 0;
		}
		else if(SHOOT_SPEED > (shoot_limit - 0.95f))
		{
			low_adopt_flg = 0;
			hig_adopt_flg++;
		}			
	}
	
	/*适应调整*/
	if(low_adopt_flg == 2)
	{	
		shoot_adopt_flg++;
		hig_adopt_flg = 0;		
		low_adopt_flg = 0;
	}

	if(hig_adopt_flg == 2)
	{	
		shoot_adopt_flg--;
		hig_adopt_flg = 0;
		low_adopt_flg = 0;
	}
	
	get_pre_shoot_sp    = SHOOT_SPEED;
	get_pre_shoot_limit = shoot_limit;	
	
}


void Friction_Open(void)
{
  int16_t Fri_Speed = 0;

	/*失联处理*/
	if(JUDGE_OFFLINE)Fri_Speed = Fri_15;
	
	/*在线处理*/
	else
	{	
		/*查表速度*/
		switch((uint16_t)shoot_limit)
		{
			case 0:
				Fri_Speed = 0;
				break;
			case 15:
				Fri_Speed = Fri_15;
				break;
			case 18:
				Fri_Speed = Fri_18;
				break;		
			case 20:
				Fri_Speed = Fri_20;
				break;	
			case 22:
				Fri_Speed = Fri_22;
				break;			
			case 30:
				Fri_Speed = Fri_30;
				break;	
			default:
				if(shoot_limit < 40 && shoot_limit)
					Fri_Speed = Fri_15 + (shoot_limit - 15) * 110;
				else
					Fri_Speed = Fri_30;
				break;
		}
		
		FRI_SPEED_TAR = Fri_Speed;
		
		/*超速处理 过低处理*/
		Fri_Speed = Fri_Speed + speed_high_flg;
		
		Fri_Speed = Fri_Speed + shoot_adopt_flg * 5;
		
		if(shoot_limit == 0)Fri_Speed = 0;
		
	}

//	Fri_Speed = shot_sp;
	
	Friction_Input(Fri_Speed);
	Friction_Output();
}



/*-------------------------------------------------------------------------------------------------------
BOXXXXXXXXX
BOXXXXXXXXX
BOXXXXXXXXX
BOXXXXXXXXX
BOXXXXXXXXX
-------------------------------------------------------------------------------------------------------*/
void Box_Stay_Ctrl(void)
{
	float box_sp;
	
	if(JUDGE_ONLINE)
	{
		if     (!COOLING_HEAT_LOW)box_sp = 4000;
		else if( COOLING_HEAT_LOW)box_sp = 2500;

	}
	else
	{
		box_sp = 2500;
	}
	
	Get_Shoot_Pid(0);
	Box_Stay_Input(box_sp);
	Box_Stay_Output();
}


void Box_Once_Ctrl(void)
{
	/*-跳变沿触发-*/
	if(State.Func.state.SHOT_ONCE == OK)
	{
		Get_Shoot_Pid(1);
		Box_Once_Input();
	}
	Box_Once_Output();
}



void Friction_Close(void)
{
	Friction_Input(0);
	Friction_Output();
}

void Box_Stop_Ctrl(void)
{
	Get_Shoot_Pid(0);
	Box_Stay_Input(0);
	Box_Stay_Output();
}





/*-------------------------------------------------------------------------------------------------------
STUCKKKKKKK
STUCKKKKKKK
STUCKKKKKKK
STUCKKKKKKK
STUCKKKKKKK
-------------------------------------------------------------------------------------------------------*/


uint32_t stuck_judge_time1;
uint32_t stuck_judge_time2;
uint32_t stuck_start_time;

/*-堵转检测-*/
bool Judge_Stuck(void)
{
	/*-确认堵转-*/
	if(Motor_Stucking(&Shot_Motor[BOX])){
		
		stuck_judge_time2 = HAL_GetTick();
		
	}
	else{
		
		stuck_judge_time1 = HAL_GetTick();
		stuck_judge_time2 = HAL_GetTick();
		
	}
	
	if(stuck_judge_time2 - stuck_judge_time1 > 500)
		return 1;
	else 
		return 0;
}


/*-堵转后处理-*/
void Box_Stuck_Judge(void)
{
	/*-判断是否堵转-*/
	if(Judge_Stuck()){
		
		State.Func.state.SHOT_STUCK = OK;

		stuck_judge_time1 = HAL_GetTick();
		stuck_judge_time2 = HAL_GetTick();
		
	  State.Func.state.SHOT_ONCE = NO;
	  State.Func.state.SHOT_STAY = NO;
		
		/*-计入开始时间-*/
		stuck_start_time = HAL_GetTick();
		
		Box_tar_set = 0;
	}
	
	/*-计算堵转时间-*/
	if(State.Func.state.SHOT_STUCK == NO){
		
		/*-复位计数器-*/
		Box_tar_set = 0;
		
	}	
	/*-长时间处于堵转强制复位-*/
	else if(State.Func.state.SHOT_STUCK == ING){
		
		if(HAL_GetTick() - stuck_start_time > 3000)
		{
			State.Func.state.SHOT_STUCK = NO;
			Box_tar_set = 0;	
		}
	}	
}


void Box_Stuck_Ctrl(void)
{
	Box_Stuck_Input();
	Box_Stuck_Output();
}


/*-发射过程进行堵转检测-*/
void Shoot_Ing_Ctrl(void)
{
	if(Barrel_OK())Box_Stuck_Judge();
}











/*-------------------------------------------------------------------------------------------------------






-------------------------------------------------------------------------------------------------------*/

void Cov_Ctrl(void){

	if     (State.Func.state.MAGAZINE == OK)Cover_Open();
	else if(State.Func.state.MAGAZINE == NO)Cover_Close();	
	
}

void Fri_Ctrl(void){

	if     (State.Func.state.FRICTION == OK)Friction_Open();
	else if(State.Func.state.FRICTION == NO)Friction_Close();

}

void Box_Ctrl(void){

	if(Fric_Err_OK() && Barrel_OK() && (State.Func.state.MAGAZINE == NO) && (COOLING_HEAT_ALLOW || JUDGE_OFFLINE))
	{
		if     (State.Func.state.SHOT_STUCK != NO)Box_Stuck_Ctrl();//到位后切为NO
		else if(State.Func.state.SHOT_ONCE  != NO)Box_Once_Ctrl(); //到位后切为NO		
		else if(State.Func.state.SHOT_STAY  == OK)Box_Stay_Ctrl();
		else                                      Box_Stop_Ctrl();	
	}
	else{
		
		Box_Stop_Ctrl();	
		
	}
}

//枪管总控制
void Bar_Ctrl(void)
{
	if(CARR_MODE == 3 && POSITION == 1){
	
		if(BARREL_2_ONLINE){
		
			Barrel();
			
		}
		else if(!BARREL_2_ONLINE){
		
			Barrel_Offline();
		
		}
	}
}

/*
 *判断是否处于射击，并且开启摩擦轮
 *弹匣初始化后才开始检测
 */
void Shoot_State(void)
{
	
	COVER_WEAK();
	
	/*-摩擦轮转速未达到，堵转不可以打弹-*/
	if(!(Fric_Err_OK() && (State.Func.state.MAGAZINE == NO)))
	{
		State.Func.state.SHOT_STAY  = NO;
	  State.Func.state.SHOT_ONCE  = NO;
		State.Func.state.SHOT_STUCK = NO;

		State.Shoot_state = NO;	
	}
	
	/*堵转下关闭射击*/
	if(State.Func.state.SHOT_STUCK != NO)
	{
		State.Func.state.SHOT_STAY = NO;
	  State.Func.state.SHOT_ONCE = NO;
	}
	
	/*-是否处于射击-*/
	if(State.Func.state.SHOT_STUCK == NO 
	&& State.Func.state.SHOT_STAY  == NO 
	&& State.Func.state.SHOT_ONCE  == NO){
		
		State.Shoot_state = NO;
		
	}
	else{
		
		Shoot_Ing_Ctrl();
		State.Shoot_state = ING;	
		
	}
	
	if(State.Func.state.MAGAZINE == OK)
	   State.Func.state.FRICTION = NO;
	
	
	/*速度补偿*/
	Fri_Speed_Add();
	
}


/*-----------------------------------------------------------------------------------------------------------------------*/	

/*-----------------------------------------------------------------------------------------------------------------------*/	

/*-----------------------------------------------------------------------------------------------------------------------*/	




/*
 *优先级：堵转 单发 连发
 */
void Shoot_Ctrl(void)
{  

	Cov_Ctrl();
	Fri_Ctrl();
	Box_Ctrl();	

	Bar_Ctrl();	
	
	Send_Current(SHOOT_CAN, SHOOT_ID, Shot_Current);
}

/**
 睡眠模式停止发射
 遥控失联 停转
 **/
void Shot_Stop(void)
{
	Friction_Close();
	Box_Stop_Ctrl();
  Barrel_Unload();
	
	COVER_SLEEP();
	
	Send_Current(SHOOT_CAN, SHOOT_ID, Shot_Current);
}







/*======================接口=======================*/


/*-执行处-*/
void SHOOT_CTRL(void)
{
	if(RC_ONLINE)
	{
		Shoot_State();
		
		Shoot_Ctrl();
		
		Task_Speed();
	}

//	Barrel_Ctrl_TEST();

//	Friction_Input(5000);
//	Friction_Output();
//	Send_Current(SHOOT_CAN, SHOOT_ID, Shot_Current);
}




/**
 * 统计
 **/
shoot_recode_t sh_rc = {

 .min = 30,
 .max = 0,
};

char tem_L,tem_R;
float sp_L,sp_R,err;

uint32_t shoot_use_time;
uint32_t get_shooting_time;
void Task_Speed(void)
{
		sp_L = Shot_Motor[FRIC_L].pid->speed.measure;
		sp_R = Shot_Motor[FRIC_R].pid->speed.measure;
		err = sp_L + sp_R;
		
		tem_L = Shot_Motor[FRIC_L].info->temperature;
		tem_R = Shot_Motor[FRIC_R].info->temperature;
	
		sh_rc.pre_speed = sh_rc.speed;
    sh_rc.speed     = judge_sensor.info->shoot_data.bullet_speed;
	
		if(sh_rc.pre_speed != sh_rc.speed && !speed_limit_change)
		{
			shoot_use_time = HAL_GetTick() - get_shooting_time;
			get_shooting_time = HAL_GetTick();
			
		  if(sh_rc.speed<12 && sh_rc.speed>=11)sh_rc.sp.cnt_11++;
			if(sh_rc.speed<13 && sh_rc.speed>=12)sh_rc.sp.cnt_12++;
			if(sh_rc.speed<14 && sh_rc.speed>=13)sh_rc.sp.cnt_13++;
			if(sh_rc.speed<15 && sh_rc.speed>=14)sh_rc.sp.cnt_14++;
			if(sh_rc.speed<16 && sh_rc.speed>=15)sh_rc.sp.cnt_15++;	
			
		  if(sh_rc.speed<17 && sh_rc.speed>=16)sh_rc.sp.cnt_16++;
			if(sh_rc.speed<18 && sh_rc.speed>=17)sh_rc.sp.cnt_17++;
			if(sh_rc.speed<19 && sh_rc.speed>=18)sh_rc.sp.cnt_18++;
			if(sh_rc.speed<20 && sh_rc.speed>=19)sh_rc.sp.cnt_19++;
			if(sh_rc.speed<21 && sh_rc.speed>=20)sh_rc.sp.cnt_20++;	
					
		  if(sh_rc.speed<22 && sh_rc.speed>=21)sh_rc.sp.cnt_21++;
			if(sh_rc.speed<23 && sh_rc.speed>=22)sh_rc.sp.cnt_22++;
			if(sh_rc.speed<24 && sh_rc.speed>=23)sh_rc.sp.cnt_23++;
			if(sh_rc.speed<25 && sh_rc.speed>=24)sh_rc.sp.cnt_24++;
			if(sh_rc.speed<26 && sh_rc.speed>=25)sh_rc.sp.cnt_25++;	
				
			if(sh_rc.speed<27 && sh_rc.speed>=26)sh_rc.sp.cnt_26++;
			if(sh_rc.speed<28 && sh_rc.speed>=27)sh_rc.sp.cnt_27++;
			if(sh_rc.speed<29 && sh_rc.speed>=28)sh_rc.sp.cnt_28++;
			if(sh_rc.speed<30 && sh_rc.speed>=29)sh_rc.sp.cnt_29++;
			if(sh_rc.speed>=30)sh_rc.sp.cnt_30++;
					
			sh_rc.cnt++;
			
			if(sh_rc.cnt)
			{
				sh_rc.sp_cnt += sh_rc.speed;
				sh_rc.pj   = sh_rc.sp_cnt / sh_rc.cnt;
			}
			
	    if(sh_rc.speed > sh_rc.max)
				sh_rc.max = sh_rc.speed;
			if(sh_rc.speed < sh_rc.min && sh_rc.speed != 0)
				sh_rc.min = sh_rc.speed;
			
			sh_rc.jicha = sh_rc.max - sh_rc.min;
			
		}
}








