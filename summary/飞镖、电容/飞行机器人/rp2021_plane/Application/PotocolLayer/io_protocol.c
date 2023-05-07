/**
 * @file        io_potocol.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        3-June-2021
 * @brief       IO Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "io_protocol.h"

#include "drv_haltick.h"
#include "button_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
GPIO_PinState IO_ReadPin(drv_io_t *io)
{
    GPIO_PinState io_state;
    
    io_state = HAL_GPIO_ReadPin(io->GPIOx, io->GPIO_Pin);
    
    return io_state;
}

void IO_WritePin(drv_io_t *io, GPIO_PinState io_state)
{
    HAL_GPIO_WritePin(io->GPIOx, io->GPIO_Pin, io_state);
}

void button_sensor_init(button_sensor_t *btn_sen)
{
//    drv_io_t *btn_drv = btn_sen->driver;
//    button_info_t *btn_info = btn_sen->info;
//    
//    if(btn_sen->id == DEV_ID_GUN_BTN_L || 
//        btn_sen->id == DEV_ID_GUN_BTN_R) 
//    {
//        // 低电平为按下
//        btn_info->state = (button_state_t)(!btn_drv->read_pin(btn_drv));
//        btn_sen->errno = NONE_ERR;
//    } else {
//        btn_sen->errno = DEV_ID_ERR;
//    }
//    
//    btn_info->flip = BTN_NONE_FLIP;
//    btn_info->hold_time = 0;
//    btn_info->update_time = millis();
}

void button_sensor_update(button_sensor_t *btn_sen)
{
//    button_state_t new_btn_state;
//    button_flip_t trig = BTN_NONE_FLIP;
//    drv_io_t *btn_drv = btn_sen->driver;
//    button_info_t *btn_info = btn_sen->info;
//    
//    if(btn_sen->id == DEV_ID_GUN_BTN_L || 
//        btn_sen->id == DEV_ID_GUN_BTN_R) 
//    {
//        // 低电平为按下
//        new_btn_state = (button_state_t)(!btn_drv->read_pin(btn_drv));
//    }
//    
//    /* 按键状态跳变 */
//    if(new_btn_state != btn_info->state) {
//        btn_info->hold_time = 0;
//        /* RELEASE -> PRESS */
//        if(new_btn_state == PRESS) {
//            trig = RELEASE_TO_PRESS;
//        }
//        /* PRESS -> RELEASE */
//        else if(new_btn_state == RELEASE) {
//            trig = PRESS_TO_RELEASE;
//        }
//    } 
//    /* 按键状态保持 */
//    else {
//        btn_info->hold_time += millis() - btn_info->update_time;
//    }
//    
//    btn_info->update_time = millis();
//    btn_info->state = new_btn_state;
//    btn_info->flip = trig;
}
