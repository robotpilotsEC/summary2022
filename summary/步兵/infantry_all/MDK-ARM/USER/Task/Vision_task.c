#include "Vision_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "State.h"

/*

������
����2ms����
�ռ����������ݺ�����

�����
Ӳ��������8ms������4ms
ֱ��Ӳ��������

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





