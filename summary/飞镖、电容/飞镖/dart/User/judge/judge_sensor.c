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
// ����ϵͳ��Ϣ
judge_info_t 	judge_info = {
	.offline_max_cnt = JUDGE_OFFLINE_MAX_CNT,
};

// ����ϵͳ
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
 *	@brief	����ϵͳ���ݼ��
 */
//typedef __packed struct
//{
//    uint8_t dart_launch_opening_status;     //��ǰ���ڷ���ڵ�״̬ 1���ر� 2�����ڿ������߹ر��� 0���Ѿ�����
//    uint8_t dart_attack_target;     //���ڵĴ��Ŀ�� 0��ǰ��վ 1�����ء�
//    uint16_t target_change_time;    //�л����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ�룬��δ�л�Ĭ��Ϊ 0��
//    uint16_t operate_launch_cmd_time;   //���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��, ��ʼֵΪ 0
//    uint8_t dart_remaining_time;    //���ڷ���ڵ���ʱ
//    uint8_t game_type : 4;          //��ǰ�������ͣ�1��RoboMaster���״�ʦ����2��RoboMaster���״�ʦ��������
//    uint8_t game_progress : 4;      //��ǰ�����׶Σ�4����ս�У�
//    uint16_t stage_remain_time;     //��ǰ�׶�ʣ��ʱ�䣬��λs
//    uint8_t robot_id;               //��������id��8���췽���ڻ����ˣ�108���������ڻ�����
//    uint16_t red_outpost_HP;        //�췽ǰ��վѪ��
//    uint16_t red_base_HP;           //�췽����Ѫ��
//    uint16_t blue_outpost_HP;       //����ǰ��վѪ��
//    uint16_t blue_base_HP;          //��������Ѫ��
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
 *	@brief	����ϵͳ������
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
		/* ����->���� */
		if(judge_sen->work_state == DEV_OFFLINE)
			judge_sen->work_state = DEV_ONLINE;
	}
}

