#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "POTOCAL.h"
#include "S_function.h"


#define STO_SPEED_VALUE       10 //8000max
#define HIG_SPEED_VALUE       300 //8000max
#define TURN_X_MAX            500
#define TOP_X_MAX             3000
#define GIMB_RESET_TIME       1500 //��̨���� ��λ���農ֹʱ��

typedef enum { 
	N_TURN,
	F_TURN,
	B_TURN,
} turn_dir_t;

typedef enum { 
	NOR_TURN,
  FAS_TURN,
} turn_mode_t;

typedef struct{
	float 	Ch0;
	float 	Ch2;
	float 	Ch3;	
		
	float   CH2_LAST_TURN;
	float   CH3_LAST_TURN;
	
	int  ZZ_FLG; //ͨ����Ϊ0��־
	int  XY_FLG;	//

	uint32_t  Chas_Time;
	bool Power_Lock;
	bool Chas_Init;
	bool Chas_Lock;
 
	uint32_t init_time;
	
	float Speed_Real[4];	
	float Speed_Val[4];	
	int   Speed_Dir[4];	
	
	float Speed_XY;
	float Speed_ZZ;
	float Speed;
  float V_rate;
	float V_MAX;
	
	float TOP_SPEED;
	//����
	float X;
	float Y;
	float Z;	

  float Get_Speed_Error[4];	//���Ŀ���ٶ����
  float Pre_Speed_Tar[4];	 //�ϴ��ٶ�Ŀ��ֵ
  float Get_Speed[4];     //��ô�ʱ�ٶ�
  float Power_X[4];      //���ߺ���Xֵ

	float RC_FX;
	float Get_RC_XY;
	float Get_RC_XY_Err;
	float Get_RC_XY_Last;		
	
	float ANGLE_RC_XY;
	float ANGLE_RC_XY_LAST;
	
	//������ϵ�̶����򣬶��ڵ��̶��ԵĽǶ�
	float Angle_XY;	// XY
	float Angle_ZZ[4];	// 	
	
  float NOW_ANGLE[4]; //���ִ�ʱ�Ƕ�
	
	float TARGET_REAL[4];	//PID�����Ŀ��ֵ������һ��Ϊ����������λ��
	float TARGET_ANGEL[4];	//�ٿ�����������Ŀ��ֵ������һ��Ϊ����������λ��
	float TARGET_ANGEL_LAST[4];	//��һ�βٿ�����������Ŀ��ֵ������һ��Ϊ����������λ��
	
	float TARGET_NOR_ERR[4];//����ת�����
	float TARGET_FAS_ERR[4];//Ѹ��ת�����	
	
	
	char Running_Wheel_Num;
	char Turnning_Wheel_Num;
	bool Running_Flg;
	
	char High_Running_Wheel_Num;
	bool High_Running_Flg;
	
	char GIMB_FRI_TURN_RESET;//����̨����ʹ��
//	char GIMB_FRI_GET_LAST_ANGLE;
	char NO_CTRL;
	
	char USE_TURN_FAST;
	
	char TURNNING;
	
	char BRAKE_FLG;
} chassis_t;


void Chassis_Power_Output(void);
void CHASSIS_TEST(void);
void Chassis_Motor_Stop(void);
void CHASSIS_CTRL(void);



extern int16_t	Chassis_Power_Current[4];
extern int16_t	Chassis_Turn_Current[4];
#endif








