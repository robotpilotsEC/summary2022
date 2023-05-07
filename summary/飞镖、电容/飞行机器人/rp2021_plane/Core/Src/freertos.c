/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SystemTask */
osThreadId_t SystemTaskHandle;
const osThreadAttr_t SystemTask_attributes = {
  .name = "SystemTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ShootTask */
osThreadId_t ShootTaskHandle;
const osThreadAttr_t ShootTask_attributes = {
  .name = "ShootTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for VisionTask */
osThreadId_t VisionTaskHandle;
const osThreadAttr_t VisionTask_attributes = {
  .name = "VisionTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for JudgeTask */
osThreadId_t JudgeTaskHandle;
const osThreadAttr_t JudgeTask_attributes = {
  .name = "JudgeTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartSystemTask(void *argument);
extern void StartShootTask(void *argument);
extern void StartGimbalTask(void *argument);
extern void StartMonitorTask(void *argument);
extern void StartVisionTask(void *argument);
extern void StartJudgeTask(void *argument);

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

  /* creation of SystemTask */
  SystemTaskHandle = osThreadNew(StartSystemTask, NULL, &SystemTask_attributes);

  /* creation of ShootTask */
  ShootTaskHandle = osThreadNew(StartShootTask, NULL, &ShootTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &GimbalTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* creation of VisionTask */
  VisionTaskHandle = osThreadNew(StartVisionTask, NULL, &VisionTask_attributes);

  /* creation of JudgeTask */
  JudgeTaskHandle = osThreadNew(StartJudgeTask, NULL, &JudgeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
