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
 *复位键朝前
 *pit 向下数据增大 
 *yaw 向左数据增大 水平4096
 *rol 向右数据增大 水平0
 *
 *6020
 *逆时针数据增大
 *
 *rc 前右正数
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
/*-位置 1上 0下-*/


/*-------------
	0 非测试模式 
	1 底盘测试
	2 自瞄测试  
	3 云台pid设置 
-------------*/
#define TEST_MODE 0  

#define SET_MID_MODE 0    //舵轮6020电机中值 0非调试 1调试

#define USART_TEST 0

/*-模块选择开启-*/
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


/*--射击统计--*/
#define SHOOT_CNT 0

/*---------------------------------------
 * 功率算法：
 * 0 不开启
 *
 * 1 动力轮开启祖传功率算法   
 * 2 舵向轮同时开启祖传功率算法 
 *
 * 3 动力轮开启2022电容功率算法   
 * 4 舵向轮同时开启2022电容功率算法  
 ---------------------------------------*/
#define POWER_CAL 4					

/*--------------------------
 * 动力轮加速方式
 * 0 直接加速 
 * 1 缓慢加速 
 *上升函数初始值
 --------------------------*/
#define POWE_MODE    0    
#define POWER_X_INIT 10 

#define MOUSE_X_RATE 15.f//6.46f  //鼠标转换为通道值的比例
#define MOUSE_Y_RATE 10.f//8.49f  //鼠标转换为通道值的比例

//底盘跟随模式 缓慢速率 最大660
#define SLOW_SPEED 100.f  

//陀螺仪kp
#define IMU_NORM_KP 1.f
#define IMU_AUTO_KP 0.1f

//电机转速设置
#define SPEED_MAX     8000.f  //最大速度 电机转速 最大8000
#define TOP_SPEED_MAX 300.f  //小陀螺最大速度 最大660


//拨盘转速
#define Box_fast 4000


//舵轮使用配置
#define TURN_MODE    1    //0 直接转向 1 迅速转向 
#define STAY_MIDD    1    //0 不保持   1 保持云台归中 





#if CARR_MODE == 0

//射击摩擦轮转速
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

//射击摩擦轮转速
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

//射击摩擦轮转速
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

//射击摩擦轮转速
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

//射击摩擦轮转速
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

//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(uint32_t addr);	//设置堆栈地址

#endif
