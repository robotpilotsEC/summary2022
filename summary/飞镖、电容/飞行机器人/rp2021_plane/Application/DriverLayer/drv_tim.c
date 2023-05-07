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
    IC_WAIT_POSEDGE,    // ��ʼ��Ĭ��Ϊ�����ز���
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
 *  Ħ����У׼
 *  1.����debugģʽ, ������Ҫ���뵽main()
 *  2.���������������1, ��ȫ��run����
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
 *  @brief  ���벶��
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
    /* ����ң�ص�SW2ͨ������ */
    if(htim == &FLYER_SW2_TIM)
    {
        /* ��ʱ��������� */
        if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE))
        {
            // ��ʱ���������
            ic_flyer_sw2.overflow_cnt++;    
        }
        
        /* ��ʱ��ͨ������ */
        if(__HAL_TIM_GET_FLAG(htim, FLYER_SW2_FLAG_CCx))
        {
            __HAL_TIM_CLEAR_FLAG(htim, FLYER_SW2_FLAG_CCx);
            
            /* ���������� */
            if(ic_flyer_sw2.state == IC_WAIT_POSEDGE)
            {
                // �����������
                ic_flyer_sw2.overflow_cnt = 0;
                // ��¼���������ص�ʱ��
                ic_flyer_sw2.posedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_SW2_CHNL);
                // ����Ϊ�����½���
                ic_flyer_sw2.state = IC_WAIT_NEGEDGE;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_SW2_CHNL, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            /* �����½��� */
            else if(ic_flyer_sw2.state == IC_WAIT_NEGEDGE)
            {
                // ��¼�����½��ص�ʱ��
                ic_flyer_sw2.negedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_SW2_CHNL);
                // ���㲶��ĸߵ�ƽ����
                ic_flyer_sw2.highlevel_val = ic_flyer_sw2.negedge_cap_val - ic_flyer_sw2.posedge_cap_val + ic_flyer_sw2.overflow_cnt*65536;
                // ��ɷ���ң��sw2ͨ���Ĳ���
                FLYER_SW2_ICHandler(ic_flyer_sw2.highlevel_val);
                // ����Ϊ����������
                ic_flyer_sw2.state = IC_WAIT_POSEDGE;
                //__HAL_TIM_SetCounter(htim, 0);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_SW2_CHNL, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }
    
    /* ����ң�صĲ���ͨ������ */
    if(htim == &FLYER_THUMBWHEEL_TIM)
    {
        /* ��ʱ��������� */
        if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE))
        {
            // ��ʱ���������
            ic_flyer_thumbwheel.overflow_cnt++;
        }
        
        /* ��ʱ��ͨ������ */
        if(__HAL_TIM_GET_FLAG(htim, FLYER_THUMBWHEEL_FLAG_CCx))
        {
            __HAL_TIM_CLEAR_FLAG(htim, FLYER_THUMBWHEEL_FLAG_CCx);

            /* ���������� */
            if(ic_flyer_thumbwheel.state == IC_WAIT_POSEDGE)
            {
                // �����������
                ic_flyer_thumbwheel.overflow_cnt = 0;
                // ��¼���������ص�ʱ��
                ic_flyer_thumbwheel.posedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_THUMBWHEEL_CHNL);
                // ����Ϊ�����½���
                ic_flyer_thumbwheel.state = IC_WAIT_NEGEDGE;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_THUMBWHEEL_CHNL, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            /* �����½��� */
            else if(ic_flyer_thumbwheel.state == IC_WAIT_NEGEDGE)
            {
                // ��¼�����½��ص�ʱ��
                ic_flyer_thumbwheel.negedge_cap_val = HAL_TIM_ReadCapturedValue(htim, FLYER_THUMBWHEEL_CHNL);
                // ���㲶��ĸߵ�ƽ����
                ic_flyer_thumbwheel.highlevel_val = ic_flyer_thumbwheel.negedge_cap_val - ic_flyer_thumbwheel.posedge_cap_val + ic_flyer_thumbwheel.overflow_cnt*65536;
                // ��ɷ���ң�ز���ͨ���Ĳ���
                FLYER_THUMBWHEEL_ICHandler(ic_flyer_thumbwheel.highlevel_val);
                // ����Ϊ����������
                ic_flyer_thumbwheel.state = IC_WAIT_POSEDGE;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FLYER_THUMBWHEEL_CHNL, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }        
    }

    // �����ʱ��������±�־λ
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
}

/* IC Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] ��Ҫ��Potocol Layer��ʵ�־���� ����ң��ͨ������ ����Э��
 */
__WEAK void FLYER_SW2_ICHandler(uint32_t val)
{
}

__WEAK void FLYER_THUMBWHEEL_ICHandler(uint32_t val)
{	
}
