/**
 * @file        a3_potocol.c
 * @author      RobotPilots@2021
 * @Version     V1.0
 * @date        28-December-2020
 * @brief       DJI A3 (Fly Controller) Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "a3_potocol.h"

#include "drv_tim.h"
#include "rp_math.h"
#include "rc_sensor.h"

/* Private macro -------------------------------------------------------------*/
#define FLYER_SW2_UP            2175
#define FLYER_SW2_MID           1660
#define FLYER_SW2_DOWN          1147
#define FLYER_THUMBWHEEL_UP     2320
#define FLYER_THUMBWHEEL_MID    1660
#define FLYER_THUMBWHEEL_DOWN   1007

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// 飞手遥控器
// sw1 - 用于飞手切换飞行模式
// sw2 - 回传信息

/* Exported variables --------------------------------------------------------*/
/**
 *  up: 2175
 *  mid: 1660
 *  down: 1147
 *  Channel7 直通通道脉宽设置为 1000us ~ 2320us
 */
uint8_t flyer_sw2;
/**
 *  up: 2320
 *  mid: 1660
 *  down: 1007
 *  Channel5 直通通道脉宽设置为 1000us ~ 2320us 勾选REV
 */
uint32_t flyer_thumbwheel;

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	解析飞手遥控sw2通道数据协议
 */
void FLYER_SW2_ICHandler(uint32_t val)
{
    if(!IfInDeathZoomInt(val, 1000, 2320))
        return;
    
    if(IfInDeathZoomInt(val, FLYER_SW2_UP-100, FLYER_SW2_UP+100)) {
        flyer_sw2 = RC_SW_UP;
    }
    else if(IfInDeathZoomInt(val, FLYER_SW2_MID-100, FLYER_SW2_MID+100)) {
        flyer_sw2 = RC_SW_MID;
    }
    else if(IfInDeathZoomInt(val, FLYER_SW2_DOWN-100, FLYER_SW2_DOWN+100)) {
        flyer_sw2 = RC_SW_DOWN;
    }
}

/**
 *	@brief	解析飞手遥控拨轮通道数据协议
 */
void FLYER_THUMBWHEEL_ICHandler(uint32_t val)
{
    if(!IfInDeathZoomInt(val, 1000, 2320))
        return;
    
    flyer_thumbwheel = val;
}
