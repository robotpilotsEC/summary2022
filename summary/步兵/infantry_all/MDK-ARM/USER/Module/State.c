/*

	模式切换以及状态


· 模式切换	  
· 模式初始化 只进行一次
· 模式运行   不断循环
· 
 
*/
#include "State.h"
#include "rc.h"
#include "key.h"
#include "Auto.h"
#include "Shoot.h"
#include "Gimbal.h"
#include "Chassis.h"

void STATE_INIT(void);
////////////////////////////////////////////////////////
void Move_state_init(void);
void Func_state_init(void);
void Ctrl_state_init(void);

////////////////////////////////////////////////////////

State_t State = 
{
	.Ctrl.mode       = NULL_CTRL,
	.Move.mode       = NULL_MOVE,
	.Move.dire       = NULL_DIR,
	.Func.mode       = NULL_FUNC,
	.Enemy           = NULL_ENEMY,
	.Barrel          = NO,
	.TOP_MODE        = LOW_SPEED,
	.init            = STATE_INIT,
	.ctrl_mode       = Ctrl_mode_change,
	.move_mode       = Move_mode_change,
	.func_mode       = Func_mode_change,
	.Shoot_init      = NO,
  .Barrel_Wise     = STAY,
};



/* 状态初始化，功能初始化，返回完成 */
/**/
extern uint32_t stuck_judge_time1,
								stuck_judge_time2;

state_t  RC_init(void){Ctrl_state_init();Rc_Init();STATE_INIT();  return OK;}
state_t KEY_init(void){Ctrl_state_init();Key_Init();STATE_INIT(); return OK;}

state_t Gimb_f_init(void){Move_state_init();GIMBAL_INIT_NOW();return OK;}
state_t  Top_f_init(void){Move_state_init();GIMBAL_INIT_NOW();return OK;}
state_t Chas_f_init(void){Move_state_init();
	
	if(State.Move.Init == NO)GIMBAL_PIT_NOW_SET();
	
	GIMBAL_YAW_INIT();
	
//	return OK;
  if(abs(Yaw_Turn_Err()) < Gimb_Motor[GIMB_Y].pid->Turn_pid.blind_err + 5)
    return OK;
	else 
		return ING;
}



void res_shot_time(void){stuck_judge_time1 = HAL_GetTick();stuck_judge_time2 = HAL_GetTick();}
	
state_t   FUN_RESET_init(void){Func_state_init();                  return OK;}
state_t SHOT_SINGLE_init(void){Func_state_init();SHOT_INIT(); res_shot_time();return OK;}
state_t   SHOT_STAY_init(void){Func_state_init();res_shot_time();  return OK;}
state_t  AUTO_SHOOT_init(void){Func_state_init();SHOT_AUTO_INIT(); return OK;}	
state_t    FRICTION_init(void){Func_state_init();FRIC_INIT();      return OK;}
state_t    MAGAZINE_init(void){Func_state_init();GIMBAL_PIT_INIT();return OK;}

/*-----------------------------------------*/

void  RC_ctrl(void){State.Ctrl.state.RC  = OK;}
void KEY_ctrl(void){State.Ctrl.state.KEY = OK;}
	
void GIMB_FIRST_ctrl(void){State.Move.state.GIMB_FIRST = OK;}
void CHAS_FIRST_ctrl(void){State.Move.state.CHAS_FIRST = OK;}
void  TOP_FIRST_ctrl(void)
{
	State.Move.state.TOP = OK;
	if(State.Move.state.TOP == OK)
	{
		switch(F_FLAG%2)
		{
			case 0:
				State.TOP_MODE = HIG_SPEED;
				break;
			case 1:
				State.TOP_MODE = TRI_SPEED;
				break;
		}
	}
}

/*-----------------------------------------*/
//void AUTO_SHOOT_ctrl(void){State.Move.state.AUTO_SHOOT = OK;}

void    MAGAZINE_ctrl(void){State.Func.state.MAGAZINE   = OK;}
void  AUTO_SHOOT_ctrl(void){State.Func.state.AUTO_SHOOT = OK;}
void   SHOT_STAY_ctrl(void){State.Func.state.SHOT_STAY  = OK;}
void    FRICTION_ctrl(void){}
void   FUN_RESET_ctrl(void){}
void SHOT_SINGLE_ctrl(void){}




//////////////////////////////////////////////////
/*================模式初始化控制函数====================*/

state_t Ctrl_mode_init(Ctrl_mode_t *mode)
{
	mode->Init = ING;
	
	if(mode->mode == RC)
		 mode->Init = RC_init();
	
	else if(mode->mode == KEY)
		 mode->Init = KEY_init();
	else
		mode->Init = OK;	
	return mode->Init;
}

/**/
state_t Move_mode_init(Move_mode_t *mode)
{

	if(mode->mode == GIMB_FIRST)
		 mode->Init = Gimb_f_init();
	
	else if(mode->mode == CHAS_FIRST)
		 mode->Init = Chas_f_init();
	
	else if(mode->mode == TOP)
		 mode->Init = Top_f_init();
	
	else
		mode->Init = OK;
	
	return mode->Init;
}



/**/
state_t Func_mode_init(Func_mode_t *mode)
{
	mode->Init = ING;
	
	if(mode->mode == FUN_RESET)
		 mode->Init = FUN_RESET_init();
	
	else if(mode->mode == SHOT_ONCE)
		 mode->Init = SHOT_SINGLE_init();
	
	else if(mode->mode == SHOT_STAY)
		 mode->Init = SHOT_STAY_init();	
	
	else if(mode->mode == FRICTION)
		 mode->Init = FRICTION_init();	
	
	else if(mode->mode == MAGAZINE)
		 mode->Init = MAGAZINE_init();

	else if(mode->mode == AUTO_SHOOT)
		 mode->Init = AUTO_SHOOT_init();		
		
	else
		mode->Init = OK;
	return mode->Init;
}


/*================模式运行====================*/

void Ctrl_mode_handle(Ctrl_mode_t *mode)
{
	if(mode->mode == RC)
		RC_ctrl();
	
	else if(mode->mode == KEY)
		KEY_ctrl();
}



void Move_mode_handle(Move_mode_t *mode)
{	
	if(mode->mode == GIMB_FIRST)
		GIMB_FIRST_ctrl();
	
	else if(mode->mode == CHAS_FIRST)
		CHAS_FIRST_ctrl();
	
	else if(mode->mode == TOP)
		TOP_FIRST_ctrl();
	
	else if(mode->mode == AUTO_SHOOT)
		AUTO_SHOOT_ctrl();
}



void Func_mode_handle(Func_mode_t *mode)
{
	if(mode->mode == FUN_RESET)
		FUN_RESET_ctrl();
	
	else if(mode->mode == MAGAZINE)
		MAGAZINE_ctrl();
	
	else if(mode->mode == FRICTION)
		FRICTION_ctrl();
	
	else if(mode->mode == SHOT_ONCE)
		SHOT_SINGLE_ctrl();
	
	else if(mode->mode == SHOT_STAY)
		SHOT_STAY_ctrl();
	
	else if(mode->mode == AUTO_SHOOT)
		AUTO_SHOOT_ctrl();	
}


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/*===============模式转换====================*/


void Ctrl_mode_change(Ctrl_mode_t *mode,ctrl_mode_t ctrl)
{
	mode->mode = ctrl;
	if(mode->pre_mode != mode->mode)
	{	
		mode->pre_mode = mode->mode;
		mode->Init = NO;		
	}
	
	if(mode->Init != OK)
		 mode->Init = Ctrl_mode_init(mode);
	else if(mode->Init == OK)
		Ctrl_mode_handle(mode);
}


void Move_mode_change(Move_mode_t *mode,move_mode_t ctrl)
{
	mode->mode = ctrl;
	if(mode->pre_mode != mode->mode)
	{	
		mode->pre_mode = mode->mode;
		mode->Init = NO;		
	}
	
	if(mode->Init != OK)
		 mode->Init = Move_mode_init(mode);
  if(mode->Init == OK)
		Move_mode_handle(mode);
}


void Func_mode_change(Func_mode_t *mode,func_mode_t ctrl)
{
	mode->mode = ctrl;
	if(mode->pre_mode != mode->mode)
	{	
		mode->pre_mode = mode->mode;
		mode->Init = NO;		
	}
	
	if(mode->Init != OK)
		 mode->Init = Func_mode_init(mode);
  if(mode->Init == OK)
		 Func_mode_handle(mode);
}

/*================模式标志位初始化====================*/

void Func_state_init(void)
{
	State.Func.state.MAGAZINE     = NO;
	State.Func.state.SHOT_STAY    = NO;
}

/*
·开启小陀螺不对功能S2上锁
·其他模式下会对
*/
void Move_state_init(void)
{
	{
		
		State.Move.state.CHAS_FIRST  = NO;
		State.Move.state.GIMB_FIRST  = NO;
		State.Move.state.TOP         = NO;
		State.TOP_MODE = LOW_SPEED;
		State.CHASS_SLOW_FLAG = 0;
		State.SW2_LOCK = 0;
		TW_CNT         = 0;
	}
}

void Ctrl_state_init(void)
{
	State.Ctrl.state.KEY = NO;
	State.Ctrl.state.RC  = NO;
	
	State.Move.state.CHAS_FIRST = NO;
	State.Move.state.GIMB_FIRST = NO;
	State.Move.state.TOP        = NO;
	
	State.Func.state.MAGAZINE   = NO;
	State.Func.state.FRICTION   = NO;
	State.Func.state.SHOT_STAY  = NO;
	State.Func.state.AUTO_SHOOT = NO;		
	TW_CNT         = 0;
}
	
void STATE_INIT(void)
{
	State.Ctrl.Init = NO;
	State.Move.Init = NO;
	State.Func.Init = NO;
	
	State.Ctrl.state.KEY = NO;
	State.Ctrl.state.RC  = NO;
	
	State.Move.state.CHAS_FIRST = NO;
	State.Move.state.GIMB_FIRST = NO;
	State.Move.state.TOP        = NO;	
	
	State.Func.state.FRICTION   = NO;
	State.Func.state.MAGAZINE   = NO;
	State.Func.state.SHOT_ONCE  = NO;
	State.Func.state.SHOT_STAY  = NO;
	State.Func.state.SHOT_STUCK = NO;
	State.Func.state.AUTO_SHOOT = NO;	
	
	State.Ctrl.mode = NULL_CTRL;
	State.Move.mode = NULL_MOVE;
	State.Func.mode = NULL_FUNC;
	
	State.TOP_MODE = HIG_SPEED;
//  State.Barrel = NO;
	
	State.Shoot_state = NO;	
	
	State.Chassis_init = NO;
	State.Gimbal_init = NO;

	State.VISION_TEST = 0;
	State.TURN_45     = 0;
	State.CHASS_SLOW_FLAG = 0;	
	
	State.WH_SP_MAX = HIG_SP;	
	State.TURN_NORMAL = STAY_MIDD;
	
	State.SW1_LOCK = 0;
	State.SW2_LOCK = 0;
	State.CH_LOCK = 0;	

	State.AUTO_STATE = NO;
	State.AUTO_ING   = NO_AUTO;
	State.AUTO_SHOT_READ = 0;
	State.AUTO_SHOOTING = 0;
	State.AUTO_DF_FLG = 0;
	
	State.SPIN_OUT_FLG = 0;
}

/*========================================*/

/*
·解锁
·键盘模式自动开锁
*/
void LOCK_OPEN(void)//键盘模式自动开锁
{
	if(RC_ONLINE)
	{
		if      (SW1_UP)State.ctrl_mode(&State.Ctrl, KEY);
		else if(!SW1_UP)State.ctrl_mode(&State.Ctrl, RC);
	
		if(SW1_UP || SW1_MID)State.SW1_LOCK = 1; 
		
		if(SW1_UP || SW2_MID)State.SW2_LOCK = 1;

	}
	else
	{
		STATE_INIT();
	}
}

/*
·LOCK仅对遥控器模式作用，键盘模式下自动开锁
·未解锁下只开启云台跟随与小陀螺功能
*/
void LOCK_CTRL(void)
{
	if(State.SW1_LOCK)//
	{
		if     (State.Ctrl.state.RC  == OK)RC_S1_CTRL();
		else if(State.Ctrl.state.KEY == OK)KB_CTRL();
	}
	else 
	{
	  if(RC_ONLINE)
		{
			 if(TW_VALUE > 600)
					State.move_mode(&State.Move, TOP);	
			 
			 else if(State.Move.state.TOP == NO)
 	  			State.move_mode(&State.Move, GIMB_FIRST);
			 
			 if(TW_VALUE < -600)State.Move.state.TOP = NO;
			 
		}
		else 
			 State.move_mode(&State.Move, NULL_MOVE);
	}
}

/*
·LOCK仅对遥控器模式作用，键盘模式下自动开锁
·未解锁下功能全部复位
*/
void LOCK_FUNC(void)
{
	if(State.SW2_LOCK)
	{
		if     (State.Ctrl.state.RC  == OK)RC_S2_CTRL();
		else if(State.Ctrl.state.KEY == OK)ENEMY();		
	}	
	else 
	{
		if(RC_ONLINE)
			 State.func_mode(&State.Func, FUN_RESET);	
		else
			 State.func_mode(&State.Func, NULL_FUNC);	
	}
}

void VISION_TEST_OPEN(void)
{
	if(SW1_UP && SW2_DOWN && (TW_VALUE < -300))
	{
	
		State.ctrl_mode(&State.Ctrl, RC);
		State.VISION_TEST = 1;
			
		State.SW2_LOCK = 0;
		
	}
	else if(SW1_UP && SW2_DOWN && (TW_VALUE > 300) && State.VISION_TEST)
	{	
		State.VISION_TEST = 0;
	}
}
/***-------------------------------------------***/	
/***-------------------------------------------***/	
/*
·控制中心
0非测试模式 1底盘测试
2自瞄测试   3云台pid设置 
*/
void STATE_CENTER(void)
{ 
	if(TEST_MODE == 0)
	{
		if(RC_ONLINE)
		{
			if(State.Gimbal_init == OK)
			{	
				VISION_TEST_OPEN();
				
				if(State.VISION_TEST)
				{
					VISION_TEST();
				}
				else
				{					
					LOCK_OPEN();

					LOCK_CTRL();
					LOCK_FUNC();			
				}
			}
		}
		else
		{
			STATE_INIT();
		}
	}
	
	else 
	{
		if(RC_ONLINE)
    {
			//测试模式下默认为遥控控制
			if(State.Ctrl.state.RC == NO)State.ctrl_mode(&State.Ctrl, RC);
			
			if     (TEST_MODE == 1)CHASSIS_TEST();
		
			else if(TEST_MODE == 2)VISION_TEST();
			
			else if(TEST_MODE == 3)SET_GIMB_PID();

		}
		else
		{
			STATE_INIT();
			
		}
	}
}

/***-------------------------------------------***/	
/***-------------------------------------------***/		












