/**
 * @file        judge_sensor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        5-November-2020
 * @brief       Device Judge.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "judge_sensor.h"
#include "rc.h"
#include "drv_uart.h"

/* Exported functions --------------------------------------------------------*/
extern void judge_sensor_init(judge_sensor_t *judge_sen);
extern void judge_sensor_update(judge_sensor_t *judge_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
#define JUDGE_OFFLINE_MAX_CNT	100
/* Private function prototypes -----------------------------------------------*/
static void judge_sensor_check(judge_sensor_t *judge_sen);
static void judge_sensor_heart_beat(judge_sensor_t *judge_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 裁判系统信息
judge_info_t 	judge_info = {
	.offline_max_cnt = JUDGE_OFFLINE_MAX_CNT,
};

// 裁判系统
judge_sensor_t	judge_sensor = {
	.info = &judge_info,
	.init = judge_sensor_init,
	.update = judge_sensor_update,
	.check = judge_sensor_check,
	.heart_beat = judge_sensor_heart_beat,
	.work_state = DEV_OFFLINE,
	.errno = NONE_ERR,
	.id = DEV_ID_JUDGE,
};
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	裁判系统数据检查
 */
//typedef __packed struct
//{
//    uint8_t dart_launch_opening_status;     //当前飞镖发射口的状态 1：关闭 2：正在开启或者关闭中 0：已经开启
//    uint8_t dart_attack_target;     //飞镖的打击目标 0：前哨站 1：基地。
//    uint16_t target_change_time;    //切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
//    uint16_t operate_launch_cmd_time;   //最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0
//    uint8_t dart_remaining_time;    //飞镖发射口倒计时
//    uint8_t game_type : 4;          //当前比赛类型，1：RoboMaster机甲大师赛；2：RoboMaster机甲大师单项赛；
//    uint8_t game_progress : 4;      //当前比赛阶段，4：对战中；
//    uint16_t stage_remain_time;     //当前阶段剩余时间，单位s
//    uint8_t robot_id;               //本机器人id，8：红方飞镖机器人；108：蓝方飞镖机器人
//    uint16_t red_outpost_HP;        //红方前哨站血量
//    uint16_t red_base_HP;           //红方基地血量
//    uint16_t blue_outpost_HP;       //蓝方前哨站血量
//    uint16_t blue_base_HP;          //蓝方基地血量
//} referee_data_t;

static void judge_sensor_check(judge_sensor_t *judge_sen)
{
	judge_info_t *judge_info = judge_sen->info;
	
	if(    judge_info->dart_client.dart_launch_opening_status <= 2 \
        && judge_info->dart_client.dart_attack_target <= 1 \
        && (   judge_info->game_robot_status.robot_id == 8 \
            || judge_info->game_robot_status.robot_id == 108))
    {
        referee_data.dart_launch_opening_status = judge_info->dart_client.dart_launch_opening_status;
        referee_data.dart_attack_target = judge_info->dart_client.dart_attack_target;
        referee_data.operate_launch_cmd_time = judge_info->dart_client.operate_launch_cmd_time;
        referee_data.dart_remaining_time = judge_info->dart_remaining_time.dart_remaining_time;
        referee_data.game_type = judge_info->game_status.game_type;
        referee_data.game_progress = judge_info->game_status.game_progress;
        referee_data.stage_remain_time = judge_info->game_status.stage_remain_time;
        referee_data.robot_id = judge_info->game_robot_status.robot_id;
        referee_data.red_outpost_HP = judge_info->game_robot_HP.red_outpost_HP;
        referee_data.red_base_HP = judge_info->game_robot_HP.red_base_HP;
        referee_data.blue_outpost_HP = judge_info->game_robot_HP.blue_outpost_HP;
        referee_data.blue_base_HP = judge_info->game_robot_HP.blue_base_HP;
    }
}

/**
 *	@brief	裁判系统心跳包
 */
static void judge_sensor_heart_beat(judge_sensor_t *judge_sen)
{
	judge_info_t *judge_info = judge_sen->info;

	judge_info->offline_cnt++;
	if(judge_info->offline_cnt > judge_info->offline_max_cnt) {
		judge_info->offline_cnt = judge_info->offline_max_cnt;
		judge_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(judge_sen->work_state == DEV_OFFLINE)
			judge_sen->work_state = DEV_ONLINE;
	}
}

