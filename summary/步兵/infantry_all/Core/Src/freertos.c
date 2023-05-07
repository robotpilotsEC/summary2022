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
osThreadId defaultTaskHandle;
osThreadId LED_TASKHandle;
osThreadId Chassis_TaskHandle;
osThreadId Judge_TaskHandle;
osThreadId Imu_taskHandle;
osThreadId Normal_taskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId Shoot_TaskHandle;
osThreadId Super_taskHandle;
osThreadId Vision_TaskHandle;
osThreadId Trigger_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_led(void const * argument);
void Start_chassis_task(void const * argument);
extern void Start_judge_task(void const * argument);
void Start_imu_task(void const * argument);
void Start_normal_task(void const * argument);
void Start_gimbal_Task(void const * argument);
void Start_shoot_task(void const * argument);
void Start_Super_Task(void const * argument);
void Start_Vision_Task(void const * argument);
void Start_Trigger_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED_TASK */
  osThreadDef(LED_TASK, Start_led, osPriorityRealtime, 0, 128);
  LED_TASKHandle = osThreadCreate(osThread(LED_TASK), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Start_chassis_task, osPriorityNormal, 0, 128);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

//  /* definition and creation of Judge_Task */
//  osThreadDef(Judge_Task, Start_judge_task, osPriorityIdle, 0, 512);
//  Judge_TaskHandle = osThreadCreate(osThread(Judge_Task), NULL);

//  /* definition and creation of Imu_task */
//  osThreadDef(Imu_task, Start_imu_task, osPriorityHigh, 0, 128);
//  Imu_taskHandle = osThreadCreate(osThread(Imu_task), NULL);

  /* definition and creation of Normal_task */
  osThreadDef(Normal_task, Start_normal_task, osPriorityAboveNormal, 0, 256);
  Normal_taskHandle = osThreadCreate(osThread(Normal_task), NULL);

//  /* definition and creation of Gimbal_Task */
//  osThreadDef(Gimbal_Task, Start_gimbal_Task, osPriorityNormal, 0, 128);
//  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);

//  /* definition and creation of Shoot_Task */
//  osThreadDef(Shoot_Task, Start_shoot_task, osPriorityNormal, 0, 128);
//  Shoot_TaskHandle = osThreadCreate(osThread(Shoot_Task), NULL);

//  /* definition and creation of Super_task */
//  osThreadDef(Super_task, Start_Super_Task, osPriorityNormal, 0, 128);
//  Super_taskHandle = osThreadCreate(osThread(Super_task), NULL);

//  /* definition and creation of Vision_Task */
//  osThreadDef(Vision_Task, Start_Vision_Task, osPriorityNormal, 0, 256);
//  Vision_TaskHandle = osThreadCreate(osThread(Vision_Task), NULL);

//  /* definition and creation of Trigger_Task */
//  osThreadDef(Trigger_Task, Start_Trigger_Task, osPriorityIdle, 0, 128);
//  Trigger_TaskHandle = osThreadCreate(osThread(Trigger_Task), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_led */
/**
* @brief Function implementing the LED_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_led */
__weak void Start_led(void const * argument)
{
  /* USER CODE BEGIN Start_led */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_led */
}

/* USER CODE BEGIN Header_Start_chassis_task */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_chassis_task */
__weak void Start_chassis_task(void const * argument)
{
  /* USER CODE BEGIN Start_chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_chassis_task */
}

/* USER CODE BEGIN Header_Start_imu_task */
/**
* @brief Function implementing the Imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_imu_task */
__weak void Start_imu_task(void const * argument)
{
  /* USER CODE BEGIN Start_imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_imu_task */
}

/* USER CODE BEGIN Header_Start_normal_task */
/**
* @brief Function implementing the Normal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_normal_task */
__weak void Start_normal_task(void const * argument)
{
  /* USER CODE BEGIN Start_normal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_normal_task */
}

/* USER CODE BEGIN Header_Start_gimbal_Task */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_gimbal_Task */
__weak void Start_gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Start_gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_gimbal_Task */
}

/* USER CODE BEGIN Header_Start_shoot_task */
/**
* @brief Function implementing the Shoot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_shoot_task */
__weak void Start_shoot_task(void const * argument)
{
  /* USER CODE BEGIN Start_shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_shoot_task */
}

/* USER CODE BEGIN Header_Start_Super_Task */
/**
* @brief Function implementing the Super_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Super_Task */
__weak void Start_Super_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Super_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Super_Task */
}

/* USER CODE BEGIN Header_Start_Vision_Task */
/**
* @brief Function implementing the Vision_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Vision_Task */
__weak void Start_Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Vision_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Vision_Task */
}

/* USER CODE BEGIN Header_Start_Trigger_Task */
/**
* @brief Function implementing the Trigger_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Trigger_Task */
__weak void Start_Trigger_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Trigger_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Trigger_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
