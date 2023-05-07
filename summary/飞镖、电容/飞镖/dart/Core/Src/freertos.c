/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "pid.h"
#include "rc.h"
#include "motor.h"
#include "bmi_solve.h"
#include "judge_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED */
osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for heart_beat */
osThreadId_t heart_beatHandle;
const osThreadAttr_t heart_beat_attributes = {
  .name = "heart_beat",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rc_control */
osThreadId_t rc_controlHandle;
const osThreadAttr_t rc_control_attributes = {
  .name = "rc_control",
  .stack_size = 3072 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LED_Task(void *argument);
void heart_beat_task(void *argument);
void rc_control_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LED */
  LEDHandle = osThreadNew(LED_Task, NULL, &LED_attributes);

  /* creation of heart_beat */
  heart_beatHandle = osThreadNew(heart_beat_task, NULL, &heart_beat_attributes);

  /* creation of rc_control */
  rc_controlHandle = osThreadNew(rc_control_task, NULL, &rc_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LED_Task */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Task */
void LED_Task(void *argument)
{
  /* USER CODE BEGIN LED_Task */
  /* Infinite loop */
  for(;;)
  {
    led_program();
    osDelay(1);
  }
  /* USER CODE END LED_Task */
}

/* USER CODE BEGIN Header_heart_beat_task */
/**
* @brief Function implementing the heart_beat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_heart_beat_task */
void heart_beat_task(void *argument)
{
  /* USER CODE BEGIN heart_beat_task */
  /* Infinite loop */
    for(;;)
    {
        rc_sensor.heart_beat(&rc_sensor);
        judge_sensor.heart_beat(&judge_sensor);
        motor[BELT_MID].heart_beat(&motor[BELT_MID]);
        motor[BELT_SIDE].heart_beat(&motor[BELT_SIDE]);
        motor[PUSH_TURN].heart_beat(&motor[PUSH_TURN]);
        motor[DART_WHEEL].heart_beat(&motor[DART_WHEEL]);
        motor[PITCH_M].heart_beat(&motor[PITCH_M]);
        motor[YAW_M].heart_beat(&motor[YAW_M]);
        motor[GIMBAL].heart_beat(&motor[GIMBAL]);
        osDelay(1);
    }
  /* USER CODE END heart_beat_task */
}

/* USER CODE BEGIN Header_rc_control_task */
/**
* @brief Function implementing the rc_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rc_control_task */
void rc_control_task(void *argument)
{
  /* USER CODE BEGIN rc_control_task */
  /* Infinite loop */
    for(;;)
    {
        rc_control();
        osDelay(1);
    }
  /* USER CODE END rc_control_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
