/**
 * @file        drv_tim.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-August-2020
 * @brief       TIMER Driver Package(Based on HAL).
 * 
 * @History     V1.1(28-December-2020)
 *              1.add TIM input capture process.
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

#include "main.h"
#include "drv_haltick.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
__WEAK void FLYER_SW2_ICHandler(uint32_t val);
__WEAK void FLYER_THUMBWHEEL_ICHandler(uint32_t val);

static void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
static void PWM_Init(void);

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    IC_WAIT_POSEDGE,    // 初始化默认为上升沿捕获
    IC_WAIT_NEGEDGE,
} input_capture_state_t;

typedef struct {
    input_capture_state_t   state;
    uint8_t                 overflow_cnt;
    int32_t                 posedge_cap_val;
    int32_t                 negedge_cap_val;
    int32_t                 highlevel_val;
} input_capture_t;

/* Private variables ---------------------------------------------------------*/
input_capture_t ic_flyer_sw2;
input_capture_t ic_flyer_thumbwheel;

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2)
{
	FRIC_PWM_L = pwm1 + 1000;
	FRIC_PWM_R = pwm2 + 1000;
}

/** 
 *  @note
 *  摩擦轮校准
 *  1.进入debug模式, 程序需要进入到main()
 *  2.将下面这个变量置1, 再全速run即可
 */
uint8_t test_fric_calibrate = 0;
uint16_t test_fric_calibrate_pwm = 1000;
static void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    if( !test_fric_calibrate ) {
        FRICTION_PwmOut(0, 0);
    } else {
        FRICTION_PwmOut(test_fric_calibrate_pwm, test_fric_calibrate_pwm);
        delay_ms(4000);
        while(test_fric_calibrate_pwm) {
            test_fric_calibrate_pwm -= 1;
            FRICTION_PwmOut(test_fric_calibrate_pwm, test_fric_calibrate_pwm);
            delay_ms(2);
        }
    }
}

/**
 *  @brief  输入捕获
 */
void IC_Init(void)
{
    HAL_TIM_IC_Start_IT(&FLYER_SW2_TIM, FLYER_SW2_CHNL);
    HAL_TIM_IC_Start_IT(&FLYER_THUMBWHEEL_TIM, FLYER_THUMBWHEEL_CHNL);
}
    
/* Exported functions --------------------------------------------------------*/
void TIM_Init(void)
{
    PWM_Init();
    IC_Init();
}

/**
  * @brief   callback this function when tim interrupt 
  * @param   TIM_Handle htimx
  * @usage   call in tim handler function TIMx_IRQHandler()
  */
void DRV_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
    /* 飞手遥控的SW2通道捕获 */
    if(htim == &FLYER_SW2_TIM)
    {
        /* 定时器计数溢出 */
        if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE))
        {
            // 定时器计数溢出
            ic_flyer_sw2.overflow_cnt++;    
        }
        
        /* 定时器通道捕获 */
        if(__HAL_TIM_GET_FLAG(htim, FLYER_SW2_FLAG_CCx))
        {
            __HAL_TIM_CLEAR_FLAG(htim, FLYER_SW2_FLAG_CCx);
            
            /* 捕获上升沿 */
            if(ic_flyer_sw2.state == IC_WAIT_POSEDGE)
            {
                // 重置溢出计数
                ic_flyer_sw2.overflow_cnt = 0;
                // 记录捕获上升沿的时刻
                ic_flyer_sw2.posedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_SW2_CHNL);
                // 设置为捕获下降沿
                ic_flyer_sw2.state = IC_WAIT_NEGEDGE;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_SW2_CHNL, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            /* 捕获下降沿 */
            else if(ic_flyer_sw2.state == IC_WAIT_NEGEDGE)
            {
                // 记录捕获下降沿的时刻
                ic_flyer_sw2.negedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_SW2_CHNL);
                // 计算捕获的高电平脉宽
                ic_flyer_sw2.highlevel_val = ic_flyer_sw2.negedge_cap_val - ic_flyer_sw2.posedge_cap_val + ic_flyer_sw2.overflow_cnt*65536;
                // 完成飞手遥控sw2通道的捕获
                FLYER_SW2_ICHandler(ic_flyer_sw2.highlevel_val);
                // 设置为捕获上升沿
                ic_flyer_sw2.state = IC_WAIT_POSEDGE;
                //__HAL_TIM_SetCounter(htim, 0);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_SW2_CHNL, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }
    
    /* 飞手遥控的拨轮通道捕获 */
    if(htim == &FLYER_THUMBWHEEL_TIM)
    {
        /* 定时器计数溢出 */
        if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE))
        {
            // 定时器计数溢出
            ic_flyer_thumbwheel.overflow_cnt++;
        }
        
        /* 定时器通道捕获 */
        if(__HAL_TIM_GET_FLAG(htim, FLYER_THUMBWHEEL_FLAG_CCx))
        {
            __HAL_TIM_CLEAR_FLAG(htim, FLYER_THUMBWHEEL_FLAG_CCx);

            /* 捕获上升沿 */
            if(ic_flyer_thumbwheel.state == IC_WAIT_POSEDGE)
            {
                // 重置溢出计数
                ic_flyer_thumbwheel.overflow_cnt = 0;
                // 记录捕获上升沿的时刻
                ic_flyer_thumbwheel.posedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_THUMBWHEEL_CHNL);
                // 设置为捕获下降沿
                ic_flyer_thumbwheel.state = IC_WAIT_NEGEDGE;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_THUMBWHEEL_CHNL, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            /* 捕获下降沿 */
            else if(ic_flyer_thumbwheel.state == IC_WAIT_NEGEDGE)
            {
                // 记录捕获下降沿的时刻
                ic_flyer_thumbwheel.negedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_THUMBWHEEL_CHNL);
                // 计算捕获的高电平脉宽
                ic_flyer_thumbwheel.highlevel_val = ic_flyer_thumbwheel.negedge_cap_val - ic_flyer_thumbwheel.posedge_cap_val + ic_flyer_thumbwheel.overflow_cnt*65536;
                // 完成飞手遥控拨轮通道的捕获
                FLYER_THUMBWHEEL_ICHandler(ic_flyer_thumbwheel.highlevel_val);
                // 设置为捕获上升沿
                ic_flyer_thumbwheel.state = IC_WAIT_POSEDGE;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_THUMBWHEEL_CHNL, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }        
    }

    // 清除定时器溢出更新标志位
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
}

/* IC Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 飞手遥控通道捕获 处理协议
 */
__WEAK void FLYER_SW2_ICHandler(uint32_t val)
{
}

__WEAK void FLYER_THUMBWHEEL_ICHandler(uint32_t val)
{	
}
