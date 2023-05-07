#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

#include "rp_driver.h"
#include "rp_device.h"
#include "rp_algo.h"
#include "rp_sys.h"

/*****************************************************************
 *imu
 *��λ����ǰ
 *pit ������������ 
 *yaw ������������ ˮƽ4096
 *rol ������������ ˮƽ0
 *
 *6020
 *��ʱ����������
 *
 *rc ǰ������
 ******************************************************************/
 


/*------------------
 *0 old_mec
 *1 new_rud
 *2 omni1
 *3 double
 *4 new_mec
 ------------------*/
#define CARR_MODE 4   
#define POSITION  1
/*-λ�� 1�� 0��-*/


/*-------------
	0 �ǲ���ģʽ 
	1 ���̲���
	2 �������  
	3 ��̨pid���� 
-------------*/
#define TEST_MODE 0  

#define SET_MID_MODE 0    //����6020�����ֵ 0�ǵ��� 1����

#define USART_TEST 0

/*-ģ��ѡ����-*/
#define CHASSIS_MD (1 && POSITION)
#define GIMBAL_MD  (1 && POSITION)
#define SHOOT_MD   (1 && POSITION)

#define VISION_MD  (1 && POSITION)
#define STATE_MD   (1 && POSITION)
#define CAP_MD     ((CARR_MODE == 0 && POSITION == 0)\
                   || (CARR_MODE == 1 && POSITION == 1)\
									 || (CARR_MODE == 2 && POSITION == 1)\
									 || (CARR_MODE == 3 && POSITION == 0)\
									 || (CARR_MODE == 4 && POSITION == 0)\
									 || (CARR_MODE == 5 && POSITION == 0))


/*--���ͳ��--*/
#define SHOOT_CNT 0

/*---------------------------------------
 * �����㷨��
 * 0 ������
 *
 * 1 �����ֿ����洫�����㷨   
 * 2 ������ͬʱ�����洫�����㷨 
 *
 * 3 �����ֿ���2022���ݹ����㷨   
 * 4 ������ͬʱ����2022���ݹ����㷨  
 ---------------------------------------*/
#define POWER_CAL 4					

/*--------------------------
 * �����ּ��ٷ�ʽ
 * 0 ֱ�Ӽ��� 
 * 1 �������� 
 *����������ʼֵ
 --------------------------*/
#define POWE_MODE    0    
#define POWER_X_INIT 10 

#define MOUSE_X_RATE 15.f//6.46f  //���ת��Ϊͨ��ֵ�ı���
#define MOUSE_Y_RATE 10.f//8.49f  //���ת��Ϊͨ��ֵ�ı���

//���̸���ģʽ �������� ���660
#define SLOW_SPEED 100.f  

//������kp
#define IMU_NORM_KP 1.f
#define IMU_AUTO_KP 0.1f

//���ת������
#define SPEED_MAX     8000.f  //����ٶ� ���ת�� ���8000
#define TOP_SPEED_MAX 300.f  //С��������ٶ� ���660


//����ת��
#define Box_fast 4000


//����ʹ������
#define TURN_MODE    1    //0 ֱ��ת�� 1 Ѹ��ת�� 
#define STAY_MIDD    1    //0 ������   1 ������̨���� 





#if CARR_MODE == 0

//���Ħ����ת��
#define Fri_15 4400
#define Fri_18 4700
#define Fri_20 4950
#define Fri_22 5200
#define Fri_30 7550
#define Fri_DF Fri_30

#define Temp 100

#define low_volt 15
#define CAP_OUTPUT_LIMIT 150

#define AIM_X 995
#define AIM_Y 540
#endif

#if CARR_MODE == 1

//���Ħ����ת��
#define Fri_15 4700
#define Fri_18 5100
#define Fri_20 4960
#define Fri_22 5200
#define Fri_30 7150
#define Fri_DF Fri_30

#define Temp 80

#define low_volt 15
#define CAP_OUTPUT_LIMIT 200

#define AIM_X 995
#define AIM_Y 540
#endif

#if CARR_MODE == 2

//���Ħ����ת��
#define Fri_15 4400
#define Fri_18 4700
#define Fri_20 4950
#define Fri_22 5200
#define Fri_30 7020
#define Fri_DF Fri_30

#define Temp 80

#define low_volt 15
#define CAP_OUTPUT_LIMIT 150

#define AIM_X 990
#define AIM_Y 540
#endif

#if CARR_MODE == 3

//���Ħ����ת��
#define Fri_15 4400
#define Fri_18 4700
#define Fri_20 4950
#define Fri_22 5200
#define Fri_30 7020
#define Fri_DF Fri_30

#define Temp 80

#define low_volt 15
#define CAP_OUTPUT_LIMIT 150

#define AIM_X 980
#define AIM_Y 540
#endif

#if CARR_MODE == 4

//���Ħ����ת��
#define Fri_15 4400
#define Fri_18 4700
#define Fri_20 4950
#define Fri_22 5200
#define Fri_30 7020
#define Fri_DF Fri_30

#define Temp 80

#define low_volt 15
#define CAP_OUTPUT_LIMIT 150

#define AIM_X 980
#define AIM_Y 540
#endif






/* Exported functions --------------------------------------------------------*/

//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(uint32_t addr);	//���ö�ջ��ַ

#endif
