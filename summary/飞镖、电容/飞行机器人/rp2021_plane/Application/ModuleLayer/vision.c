/**
 * @file        vision.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Vision Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision.h"
#include "gimbal.h"
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
STATIC void VISION_Init(void);
STATIC void VISION_Ctrl(void);
STATIC void VISION_Reset(void);
STATIC void VISION_Test(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static vision_info_t *vis_info;
static vision_sensor_t *vis_sen;
static judge_sensor_t *judge_sen;

/* Exported variables --------------------------------------------------------*/
vision_dev_t vision_dev = {
    .vision_sensor = &vision_sensor,
    .rc_sensor = &rc_sensor,
    .judge_sensor = &judge_sensor
};

vision_info_t vision_info = {
    .mode = VISION_MODE_AIM_OUTPOST,
};

vision_t vision = {
	.dev = &vision_dev,
    .info = &vision_info,
    .init = VISION_Init,
    .ctrl = VISION_Ctrl,
    .test = VISION_Test,
    .self_protect = VISION_Reset,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

STATIC void VISION_GetJudgeInfo(void)
{
    color_t color = judge_sen->my_color(judge_sen);
    vis_sen->set_color(vis_sen, color);
    vis_sen->info->tx_packet.tx_data.pitch = gimbal.dev->pit_motor->info->angle;
    vis_sen->info->tx_packet.tx_data.yaw = gimbal.dev->yaw_motor->info->angle;
}

STATIC void VISION_GetSysInfo(void)
{
	/*----本地模式修改----*/
	switch(sys.mode)
	{
		case SYS_MODE_NORMAL:
		{
			vis_info->mode = VISION_MODE_MANUAL;
		}break;
		case SYS_MODE_AUTO:
		{
            if(sys.act == SYS_ACT_AIM_OUTPOST)
                vis_info->mode = VISION_MODE_AIM_OUTPOST;
            else if(sys.act == SYS_ACT_AIM_ARMY)
                vis_info->mode = VISION_MODE_AIM_ARMY;
		}break;
		default:
		{
		}break;
	}    
}

/* 应用层 --------------------------------------------------------------------*/
// 裁判系统
void VISION_GetInfo(void)
{
    VISION_GetJudgeInfo();
    VISION_GetSysInfo();
}

/* 任务层 --------------------------------------------------------------------*/
void VISION_Init(void)
{
    // 载入视觉本地驱动
    vis_info = &vision_info;
    vis_sen = vision_dev.vision_sensor;
    judge_sen = vision_dev.judge_sensor;
}

void VISION_Reset(void)
{
    VISION_GetInfo();
    
    vis_sen->send_to_pc(vis_sen, CMD_AIM_OFF);
}

void VISION_Ctrl(void)
{
    VISION_GetInfo();
    
    switch(vis_info->mode)
    {
        case(VISION_MODE_MANUAL):
            vis_sen->send_to_pc(vis_sen, CMD_AIM_OFF);
            break;
        case(VISION_MODE_AIM_OUTPOST):
            vis_sen->send_to_pc(vis_sen, CMD_AIM_OUTPOST);
            break;
        case(VISION_MODE_AIM_ARMY):
            vis_sen->send_to_pc(vis_sen, CMD_AIM_ARMY);
            break;
        default:
            vis_sen->send_to_pc(vis_sen, CMD_AIM_OFF);
            break;
    }
}

void VISION_Test(void)
{
    vis_sen->info->tx_packet.tx_data.my_color = RED;
//    vis_sen->info->tx_packet.tx_data.test2 = vis_sen->info->rx_packet.rx_data.pit_err +
//                                             vis_sen->info->rx_packet.rx_data.yaw_err +
//                                             vis_sen->info->rx_packet.rx_data.distance +
//                                             vis_sen->info->rx_packet.rx_data.lose_target +
//                                             vis_sen->info->rx_packet.rx_data.lock_target +
//                                             vis_sen->info->rx_packet.rx_data.res0 +
//                                             vis_sen->info->rx_packet.rx_data.res1 +
//                                             vis_sen->info->rx_packet.rx_data.res2 +
//                                             vis_sen->info->rx_packet.rx_data.res3;
    vis_sen->send_to_pc(vis_sen, CMD_AIM_ARMY);
}

