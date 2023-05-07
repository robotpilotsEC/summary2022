#include "Vision_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "State.h"

/*

·自瞄
发送2ms周期
收集陀螺仪数据后打开相机

·打符
硬触发周期8ms，任务4ms
直接硬触发就行

*/

void Start_Vision_Task(void const * argument)
{
	for(;;)
	{
		if(VISION_MD)Vision_Get();
		Vision_TX();	

//		AUTO_SHOOT_CTRL();
		osDelay(2);
	}
}

void Start_Trigger_Task(void const * argument)
{
	
	for(;;)
	{
//		Hard_Trigger();
		osDelay(4);
	}
	
}





