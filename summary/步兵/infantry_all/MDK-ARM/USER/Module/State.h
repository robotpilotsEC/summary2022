#ifndef __STATE_H
#define __STATE_H

#include "rp_config.h"
#include "POTOCAL.h"



typedef enum { 
 NO  = 0,//未ok
 OK  = 1,//已ok
 ING = 2,//正在进行
} state_t;

typedef enum { 
 STAY  = 0,//未ok
 CLOCK  = 1,//已ok
 ANTICLOCK = 2,//正在进行
} wise_t;

////////////////////////

typedef enum {
 NULL_CTRL,
 KEY,
 RC,
} ctrl_mode_t;


typedef enum {
 NULL_MOVE,
 GIMB_FIRST,
 CHAS_FIRST,
 TOP,
// AUTO_SHOOT,
} move_mode_t;


//////////////////////////////////
typedef enum {
 NULL_FUNC,
 MAGAZINE,
	
 AUTO_SHOOT,
 SHOT_ONCE, 
 SHOT_STAY,
 SHOT_STUCK,
	
 FRICTION,//键盘下开启后永不关闭，handle内标志位置1就行
 FUN_RESET,
		
} func_mode_t;

typedef enum {
 NULL_DIR,
 DIR_F,
 DIR_B,
} dire_t;

typedef enum {

 RED = 0,
 BLUE,
 AUTO,	
 NULL_ENEMY,	
	
} enemy_t;

typedef enum { 
	LOW_SPEED = 1,
	HIG_SPEED = 2,
	TRI_SPEED = 3,
} top_mode_t;

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
typedef struct Func_state_struct{
	state_t   SHOT_STUCK;	
	state_t   AUTO_SHOOT;	
	state_t   SHOT_ONCE;
	state_t		SHOT_STAY;
	
	state_t   MAGAZINE;	
	state_t   FRICTION;
} Func_state_t;

typedef struct Ctrl_state_struct{
	state_t   KEY;
	state_t   RC;
} Ctrl_state_t;

typedef struct Move_state_struct{
	state_t   GIMB_FIRST;
	state_t   CHAS_FIRST;
	state_t   TOP;
//	state_t   AUTO_SHOOT;	
} Move_state_t;



/*+++++++++++++++++++++++++++++++++++++++++++++*/

typedef struct Ctrl_struct{
	ctrl_mode_t   mode;
	ctrl_mode_t   pre_mode;
	
	Ctrl_state_t  state;	
	state_t       Init;
} Ctrl_mode_t;

typedef struct Move_struct{
	move_mode_t   mode;
	move_mode_t   pre_mode;
	dire_t        dire;
	
	Move_state_t  state;
	state_t       Init;
} Move_mode_t;


typedef struct Func_struct{	
	func_mode_t    mode;//表示是否需要初始化
	func_mode_t    pre_mode;
	
	Func_state_t  state;//表示是否运行
	state_t       Init;	
} Func_mode_t;

////////////////////////////////////////////////
typedef enum
{
	DF,
	ZM,
	NO_AUTO,
}auto_ing_t;

typedef enum
{
	LOW_SP,
	HIG_SP,
}speed_t;

////////////////////////////////////////////////
/*
Ctrl_mode  -> Rc Key        Init Reset
Move_mode  -> Direction Gimb_first Chas_first  Init Reset
Shoot      -> Single Long   Init Reset 
Enemy      -> Red Blue Auto
*/
typedef struct State_struct{
	Ctrl_mode_t    Ctrl;
	Move_mode_t    Move;	
	Func_mode_t    Func;
	
	state_t        Shoot_state;	
	
	state_t        Chassis_init;
	state_t        Gimbal_init;
	state_t        Shoot_init;	
	
	state_t        Barrel;

	bool           VISION_TEST;
	
  bool           CHASS_SLOW_FLAG;

  bool           TURN_NORMAL;
	
  bool           TURN_45;

	state_t				 AUTO_STATE;
	
	auto_ing_t     AUTO_ING;

	bool           AUTO_SHOT_READ;
	
	bool           AUTO_SHOOTING;
	
	char           AUTO_DF_FLG;

	bool           SPIN_OUT_FLG;

	speed_t        WH_SP_MAX;

	enemy_t        Enemy;
  wise_t         Barrel_Wise;
	top_mode_t     TOP_MODE;
	
	bool           SW1_LOCK;  //控制锁
	bool           SW2_LOCK;  //功能锁
	bool           CH_LOCK;  //通道锁	
	
	void					(*init)(void);
	void          (*ctrl_mode)(struct Ctrl_struct *Ctrl,ctrl_mode_t ctrl);
	void          (*move_mode)(struct Move_struct *Ctrl,move_mode_t ctrl);
	void          (*func_mode)(struct Func_struct *Ctrl,func_mode_t ctrl);

}State_t;


void Ctrl_mode_change(Ctrl_mode_t *mode,ctrl_mode_t ctrl);
void Move_mode_change(Move_mode_t *mode,move_mode_t ctrl);
void Func_mode_change(Func_mode_t *mode,func_mode_t ctrl);


#if 0
__weak bool					 RC_init(void);
__weak bool    			KEY_init(void);

__weak bool      Gimb_f_init(void);
__weak bool      Chas_f_init(void);
__weak bool       Top_f_init(void);

__weak bool    MEC_ctrl_init(void);
__weak bool    IMU_ctrl_init(void);

__weak bool   FUN_RESET_init(void);
__weak bool SHOT_SINGLE_init(void);
__weak bool   SHOT_STAY_init(void);
__weak bool  SHOT_STUCK_init(void);
__weak bool    FRICTION_init(void);
__weak bool    MAGAZINE_init(void);

__weak bool          RC_ctrl(void);
__weak bool         KEY_ctrl(void);

__weak bool  GIMB_FIRST_ctrl(void);
__weak bool  CHAS_FIRST_ctrl(void);
	
__weak bool    MEC_CTRL_ctrl(void);
__weak bool    IMU_CTRL_ctrl(void);

__weak bool    MAGAZINE_ctrl(void);
__weak bool    FRICTION_ctrl(void);
__weak bool SHOT_SINGLE_ctrl(void);
__weak bool   SHOT_STAY_ctrl(void);
__weak bool  SHOT_STUCK_ctrl(void);
#endif

extern State_t State;



void STATE_INIT(void);
void STATE_CENTER(void);


#endif

