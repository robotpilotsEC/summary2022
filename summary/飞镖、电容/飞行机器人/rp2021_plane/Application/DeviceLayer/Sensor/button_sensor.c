/**
 * @file        button_sensor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        3-June-2021
 * @brief       Button's Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "button_sensor.h"

#include "drv_io.h"
#include "io_protocol.h"

extern void button_sensor_init(button_sensor_t *btn_sen);
extern void button_sensor_update(button_sensor_t *btn_sen);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void button_sensor_check(button_sensor_t *btn_sen);
static void button_sensor_heart_beat(button_sensor_t *btn_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
//drv_io_t 	button_sensor_driver[BTN_SENSOR_CNT] = {
//    [BTN_GUN_L] = {
//        .type = DRV_IO,
//        .GPIOx = GUN_TRIG_L_GPIO_Port,
//        .GPIO_Pin = GUN_TRIG_L_Pin,
//        .read_pin = IO_ReadPin,
//        .write_pin = IO_WritePin,
//    },
//    [BTN_GUN_R] = {
//        .type = DRV_IO,
//        .GPIOx = GUN_TRIG_R_GPIO_Port,
//        .GPIO_Pin = GUN_TRIG_R_Pin,
//        .read_pin = IO_ReadPin,
//        .write_pin = IO_WritePin,        
//    }
//};

//button_info_t 	button_sensor_info[BTN_SENSOR_CNT];

//button_sensor_t 	button_sensor[BTN_SENSOR_CNT] = {
//    [BTN_GUN_L] = {
//        .info = &button_sensor_info[BTN_GUN_L],
//        .driver = &button_sensor_driver[BTN_GUN_L],
//        .init = button_sensor_init,
//        .update = button_sensor_update,
//        .check = button_sensor_check,
//        .heart_beat = button_sensor_heart_beat,
//        .work_state = DEV_ONLINE,
//        .id = DEV_ID_GUN_BTN_L
//    },
//    [BTN_GUN_R] = {
//        .info = &button_sensor_info[BTN_GUN_R],
//        .driver = &button_sensor_driver[BTN_GUN_R],
//        .init = button_sensor_init,
//        .update = button_sensor_update,
//        .check = button_sensor_check,
//        .heart_beat = button_sensor_heart_beat,
//        .work_state = DEV_ONLINE,
//        .id = DEV_ID_GUN_BTN_R
//    },
//};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
static void button_sensor_check(button_sensor_t *btn_sen)
{
    btn_sen->errno = NONE_ERR;
}

static void button_sensor_heart_beat(button_sensor_t *btn_sen)
{
	btn_sen->work_state = DEV_ONLINE;
}
