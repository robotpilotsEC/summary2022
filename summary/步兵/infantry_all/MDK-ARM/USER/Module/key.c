/*
*	  按键判断与对应动作
*	  包括遥控器
*/

#include "key.h"
#include "State.h"
#include "Auto.h"
#include "judge_sensor.h"


extern Vision_Cmd_Id_t AUTO_CMD_MODE;
extern uint32_t Barrel_Change_Time;
extern bool Change_Flg;

void RC_S1_CTRL(void)
{
	if(SW1_MID && TW_VALUE < -600){
		
		if(State.Move.state.TOP == NO)
			State.move_mode(&State.Move, TOP);	
		
	}
	else if(SW1_MID && TW_VALUE > 600){

		if(State.Move.state.TOP == OK)State.Move.state.TOP = NO;
		
	}
	
	else if(State.Move.state.TOP == NO && SW1_DOWN){
		
		if(State.Move.state.CHAS_FIRST == NO)
			State.move_mode(&State.Move, CHAS_FIRST);
		
	}
		
	else if(State.Move.state.TOP == NO && SW1_MID){
	
		if(State.Move.state.GIMB_FIRST == NO)
			State.move_mode(&State.Move, GIMB_FIRST);
		
	}
	
	if(TW_VALUE == -660){

		if((State.Move.state.CHAS_FIRST == OK) && State.Barrel == OK){
		
			Barrel_Change_Time = HAL_GetTick();
			Change_Flg = 1;		
			
		}

	}
	
}

void RC_S2_CTRL(void)
{
	if(SW2_MID)State.func_mode(&State.Func, FUN_RESET);
	
	if(SW2_UP)
	{
		if((State.Move.state.GIMB_FIRST == OK || State.Move.state.TOP == OK)
		&& State.Func.state.SHOT_ONCE  == NO)	
			State.func_mode(&State.Func, SHOT_STAY);
		
		if(State.Move.state.CHAS_FIRST == OK
		&& State.Func.state.SHOT_ONCE  == NO)	
			State.func_mode(&State.Func, SHOT_ONCE);
	}
	
	if(SW2_DOWN)
	{
		if((State.Move.state.GIMB_FIRST == OK || State.Move.state.TOP == OK))
			State.func_mode(&State.Func, FRICTION);
		
		if(State.Move.state.CHAS_FIRST == OK)	
			State.func_mode(&State.Func, MAGAZINE);
	}	
}

void ENEMY(void)
{
	if(SW2_UP)  State.Enemy = RED;
	if(SW2_MID) State.Enemy = BLUE;
	if(SW2_DOWN)State.Enemy = AUTO;
}



/*====================================================*/
/*===================自动复位=========================*/
/*
 * 在某些条件，功能自动复位
	 射击禁止复位条件：单发未结束，左键按下
		
		特指键盘模式下
		单发射击条件：打符模式下到位 鼠标单点
		连发射击条件：自瞄模式下到位 鼠标长点	
*/
void Shot_Reset(void)
{		
//		if(!MOUSE_LEFT && State.Func.state.SHOT_ONCE == NO && !R_FLAG && !State.AUTO_SHOOTING)
	if(State.AUTO_ING == ZM){
		
		if(!State.AUTO_SHOOTING && State.Func.state.SHOT_ONCE == NO){
		
			State.Func.mode     = NULL_FUNC;
			State.Func.pre_mode = NULL_FUNC;
			State.Func.state.SHOT_STAY = NO;
		
		}
	}

	else if(!MOUSE_LEFT && State.Func.state.SHOT_ONCE == NO && !R_FLAG)
	{
		State.Func.mode     = NULL_FUNC;
		State.Func.pre_mode = NULL_FUNC;
		State.Func.state.SHOT_STAY = NO;
	}
	
	/*-射击关弹仓-*/
	if(State.Shoot_state == ING)State.Func.state.MAGAZINE = NO;
}

/*-移动模式禁止复位条件：不处于小陀螺和机械模式下-*/
void Move_Reset(void)
{
	if(State.Move.state.CHAS_FIRST == NO && State.Move.state.TOP == NO)
		
	 State.move_mode(&State.Move, GIMB_FIRST);
}

/*自瞄自动复位*/
void Auto_Reset(void)
{
	if(!MOUSE_RIGH && !KEY_G && !KEY_Z && !KEY_X){
		
		State.Func.state.AUTO_SHOOT = NO;
		AUTO_CMD_MODE = CMD_AIM_OFF;	
		
	}
}
/*-自动射击控制-*/
void Auto_Shoot_Ctrl(void)
{
	if(!State.VISION_TEST && TEST_MODE == 0){
		
		State.AUTO_SHOOTING = AUTO_SHOOT_CTRL();
		
	}
}

void Barrel_Change_Key(void)
{
	if(State.Barrel == OK && rc.info->Mouse_ROLL < -1){

			Barrel_Change_Time = HAL_GetTick();
			Change_Flg = 1;		
	}
}
/*====================自动复位=======================*/
/*====================================================*/


//射击
void Key_Mouse_L(void)//
{
  KEY_State_Judge(&Keyboard.Mouse_L ,MOUSE_LEFT, CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_L); 
  switch(Keyboard.Mouse_L.State)
  {
    case UP:
      break;
    case PRESS:	
			
			if(State.Func.state.MAGAZINE == OK)
				 State.Func.state.MAGAZINE = NO;		
			else if(State.Func.state.FRICTION == NO)
				 State.Func.state.FRICTION = OK;		
			
			else if(State.AUTO_STATE != ZM)		
				 State.func_mode(&State.Func, SHOT_ONCE);
			
			break;
    case SHORT_DOWN:
      break;  
    case DOWN:
			if(State.Func.state.SHOT_ONCE == NO && State.AUTO_STATE != ZM)//单发打完后才连发
			State.func_mode(&State.Func, SHOT_STAY);
      break;
    case RELAX:			
      break;
  }
}
//自瞄
void Key_Mouse_R(void)//
{
  KEY_State_Judge(&Keyboard.Mouse_R ,MOUSE_RIGH, CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_R); 
  switch(Keyboard.Mouse_R.State)
  {
    case UP:
      break;
    case PRESS:
			
			State.Func.state.MAGAZINE = NO;		
			if(AUTO_CMD_MODE == CMD_AIM_OFF)
			{
				State.func_mode(&State.Func, AUTO_SHOOT);
				AUTO_CMD_MODE = CMD_AIM_AUTO;			
			}
			
      break;
    case SHORT_DOWN:
      break;  
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

/***=================================================***/

void Key_W(void)
{
  KEY_State_Judge(&Keyboard.W ,KEY_W , CHANGE_TIM , LONG_CHANGE_TIM_W);
  switch(Keyboard.W.State)
  {
    case UP:			
      break;
    case PRESS:
      break;
    case SHORT_DOWN:			
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_S(void)
{
  KEY_State_Judge(&Keyboard.S ,KEY_S, CHANGE_TIM , LONG_CHANGE_TIM_S);
  switch(Keyboard.S.State)
  {
    case UP:
      break;
    case PRESS:
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_D(void)
{
  KEY_State_Judge(&Keyboard.D ,KEY_D, CHANGE_TIM , LONG_CHANGE_TIM_D);
  switch(Keyboard.D.State)
  {
    case UP:
      break;
    case PRESS:
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:		
      break;
  }
}

void Key_A(void)
{
  KEY_State_Judge(&Keyboard.A ,KEY_A, CHANGE_TIM , LONG_CHANGE_TIM_A);
  switch(Keyboard.A.State)
  {
    case UP:
      break;
    case PRESS:
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:	
      break;
  }
}
/*键盘值转为遥控通道值 end*/



/*功能键 begin*/

void Key_Q(void)//
{
	if(abs(Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.err) < 40 && Q_FLAG)Q_FLAG = 0;
	
  KEY_State_Judge(&Keyboard.Q ,KEY_Q, CHANGE_TIM , LONG_CHANGE_TIM_Q);
  switch(Keyboard.Q.State)
  {
    case UP: //抬起状态
      break;
    case PRESS: //按下瞬间
			Q_FLAG = !Q_FLAG;
			E_FLAG = 0;
			C_FLAG = 0;			
	
			if(Q_FLAG)
			{
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = Limit_Target(YAW + 2047);
			  Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.kp = 0.92;
			}
				
			else
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = YAW;
      break;
    case SHORT_DOWN: //短按			
      break;
    case DOWN: //长按
      break;
    case RELAX: //松开瞬间
      break;
  }
}

void Key_E(void)//
{
	if(abs(Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.err) < 40 && E_FLAG)E_FLAG = 0;
	
  KEY_State_Judge(&Keyboard.E ,KEY_E, CHANGE_TIM , LONG_CHANGE_TIM_E);
  switch(Keyboard.E.State)
  {
    case UP: //抬起状态
      break;
    case PRESS: //按下瞬间
			Q_FLAG = 0;
			E_FLAG = !E_FLAG;
			C_FLAG = 0;
		
			if(E_FLAG)
			{
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = Limit_Target(YAW - 2047);
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.kp = 0.92;
			}
			else
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = YAW;
			
      break;
    case SHORT_DOWN: //短按			
      break;
    case DOWN: //长按
      break;
    case RELAX: //松开瞬间
      break;
  }
}

void Key_C(void)//
{
	if(abs(Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.err) < 40 && C_FLAG)C_FLAG = 0;
	
  KEY_State_Judge(&Keyboard.C ,KEY_C, CHANGE_TIM , LONG_CHANGE_TIM_C);
  switch(Keyboard.C.State)
  {
 		case UP:		
      break;
    case PRESS:
			Q_FLAG = 0;
			E_FLAG = 0;
			C_FLAG = !C_FLAG;
		
			if(C_FLAG)
			{
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = Limit_Target(YAW - 4095);
			  Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.kp = 0.90;
			}
			else
				Gimb_Motor[GIMB_Y].pid->Imu_Out_Pid.target = YAW;
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}



//////////////////////////////////////////////

void Key_F(void)//
{
  KEY_State_Judge(&Keyboard.F ,KEY_F, CHANGE_TIM , LONG_CHANGE_TIM_F);
  switch(Keyboard.F.State)
  {
    case UP: //抬起状态
			if(State.Move.state.TOP == NO)F_FLAG = 0;
      break;
    case PRESS: //按下瞬间
			F_FLAG++;
      break;
    case SHORT_DOWN: //短按
			State.move_mode(&State.Move, TOP);	
      break;
    case DOWN: //长按
      break;
    case RELAX: //松开瞬间
      break;
  }
}



 
uint32_t b_1,b_2;//时间计数标志位 负责计算B键按两下的间隔时间
uint32_t box_1;
void Key_B(void)
{
  KEY_State_Judge(&Keyboard.B ,KEY_B, CHANGE_TIM , LONG_CHANGE_TIM_B);
  switch(Keyboard.B.State)
  {
    case UP:				
      break;
    case PRESS:		
			B_FLAG++;			
			if(  B_FLAG%2 )b_1 = HAL_GetTick();
			if(!(B_FLAG%2))b_2 = HAL_GetTick();	
			if(abs(b_2 - b_1) < 500)
			{
				State.func_mode(&State.Func, MAGAZINE);
				box_1 = HAL_GetTick();
			}
		  else if(State.Func.state.MAGAZINE == OK && (HAL_GetTick() - box_1) > 400)
			{
				State.Func.state.MAGAZINE = NO;	
			}
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
			if(State.Move.state.CHAS_FIRST == OK)
			State.CHASS_SLOW_FLAG = !State.CHASS_SLOW_FLAG;
      break;
    case RELAX:
      break;
  }
}

uint32_t r_time1 = 0;
void Key_R(void)//
{
  KEY_State_Judge(&Keyboard.R ,KEY_R, CHANGE_TIM , LONG_CHANGE_TIM_R);
  switch(Keyboard.R.State)
  {
    case UP:
			if(!COOLING_HEAT_ALLOW)R_FLAG = 0;
	
			if(R_FLAG && (HAL_GetTick() - r_time1) > 1200)R_FLAG = 0;
				
      break;
    case PRESS:
			R_FLAG = !R_FLAG;
			if(!COOLING_HEAT_ALLOW)R_FLAG = 0;
			if(R_FLAG)
			{
				r_time1 = HAL_GetTick();
				State.func_mode(&State.Func, SHOT_STAY);
			}
      break;
    case SHORT_DOWN:		
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}


void Key_X(void)//
{
  KEY_State_Judge(&Keyboard.X ,KEY_X, CHANGE_TIM , LONG_CHANGE_TIM_X);
  switch(Keyboard.X.State)
  {
    case UP:		
      break;
			
    case PRESS:
			if(AUTO_CMD_MODE == CMD_AIM_OFF)
			{
				State.func_mode(&State.Func, AUTO_SHOOT);
				AUTO_CMD_MODE = CMD_AIM_SMALL_BUFF;	
			}
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_Z(void)//
{
  KEY_State_Judge(&Keyboard.Z ,KEY_Z, CHANGE_TIM , LONG_CHANGE_TIM_Z);
  switch(Keyboard.Z.State)
  {
    case UP:	
      break;
			
    case PRESS:
			if(AUTO_CMD_MODE == CMD_AIM_OFF)
			{
				State.func_mode(&State.Func, AUTO_SHOOT);
				AUTO_CMD_MODE = CMD_AIM_BIG_BUFF;	
			}
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_G(void)
{
  KEY_State_Judge(&Keyboard.G ,KEY_G, CHANGE_TIM , LONG_CHANGE_TIM_G);
  switch(Keyboard.G.State)
  {
    case UP:			
      break;
			
    case PRESS:
			if(AUTO_CMD_MODE == CMD_AIM_OFF)
			{
				State.func_mode(&State.Func, AUTO_SHOOT);
				AUTO_CMD_MODE = CMD_AIM_ANTOP;	
			}
      break;
    case SHORT_DOWN:		
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}





uint32_t v_1,v_2;//时间计数标志位 负责计算B键按两下的间隔时间 
void Key_V(void)
{
  KEY_State_Judge(&Keyboard.V ,KEY_V, CHANGE_TIM , LONG_CHANGE_TIM_V);
  switch(Keyboard.V.State)
  {
		case UP:		
      break;
    case PRESS:
			if(State.Move.state.GIMB_FIRST == OK)
			{
				V_FLAG++;			
				if(  V_FLAG%2 )v_1 = HAL_GetTick();
				if(!(V_FLAG%2))v_2 = HAL_GetTick();	
				if(abs(v_2-v_1) < 500)
				{
					State.TURN_45 = !State.TURN_45;
					Judge_Dir(&Gimb_Motor[GIMB_Y]);						
				}
			}
      break;
    case SHORT_DOWN:		
      break;
    case DOWN:
			State.TURN_NORMAL = 0;	
			State.TURN_45 = 0;		
      break;
    case RELAX:
      break;
  }
}



void Key_ctrl(void)//
{
  KEY_State_Judge(&Keyboard.CTRL ,KEY_CTRL, CHANGE_TIM , LONG_CHANGE_TIM_CTRL);
  switch(Keyboard.CTRL.State)
  {
    case UP:
      break;
    case PRESS:
			if(State.Func.state.MAGAZINE == OK)
			{
				State.Func.state.MAGAZINE = NO;
			}
			else if(State.Move.state.TOP == NO)
			{
				State.TURN_45 = 0;
				State.TURN_NORMAL = 1;		
				State.Move.dire = DIR_F;//强制切换为朝前
        Gimb_Motor[GIMB_Y].pid->Turn_pid.target = MEC_MID_Y_F;	
			}			

      break;
    case SHORT_DOWN:
			State.move_mode(&State.Move, GIMB_FIRST);
			break;
    case DOWN:
			State.move_mode(&State.Move, CHAS_FIRST);//
      break;
    case RELAX:			
      break;
  }
}

void Key_SHIFT(void)
{
  KEY_State_Judge(&Keyboard.SHIFT ,KEY_SHIFT, CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);
  switch(Keyboard.SHIFT.State)
  {
    case UP:
			State.WH_SP_MAX = LOW_SP;
      break;
    case PRESS:
			State.WH_SP_MAX = HIG_SP;
      break;
    case SHORT_DOWN:
			
      break;  
    case DOWN:

      break;
    case RELAX:
      break;
  }
}
/*功能键 end*/


void KB_CTRL(void)
{
		if(!SW2_DOWN && TW_VALUE < -300)NVIC_SystemReset();
	
	  Key_Channel_Update();//更新模拟通道值
	  
		Key_Q();
		Key_E();
		Key_C();
		
		Key_B();
		Key_V();	
	
		Key_F();
		Key_R();
	
		Key_Z();	
		Key_X();	
		Key_G();	
		
	  if(KEY_Z && KEY_X)AUTO_CMD_MODE = CMD_AIM_ANDF;
	
		if(!KEY_A && !KEY_W && !KEY_D && !KEY_S)Key_ctrl();
	
		Key_SHIFT();

		Key_Mouse_L();
		Key_Mouse_R();

		Auto_Shoot_Ctrl();
    Barrel_Change_Key();
		
		Shot_Reset();
		Move_Reset();
		Auto_Reset();
}
