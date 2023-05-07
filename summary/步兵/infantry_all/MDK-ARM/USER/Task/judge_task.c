/**
 * @file        judge_task.c
 * @author      Clumsy_Bird@2021
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Judgement interact messages send task
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "UI.h"



void Start_judge_task(void const * argument)
{

	for(;;)
	{
	
		Client_task();
		
		osDelay(1);
	}

}



