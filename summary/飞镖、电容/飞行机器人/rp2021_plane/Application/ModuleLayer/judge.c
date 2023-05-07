/**
 * @file        judge.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        5-November-2020
 * @brief       Judge Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "judge.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
STATIC bool JUDGE_IfDataValid(void);
STATIC color_t JUDGE_GetMyColor(void);
STATIC uint16_t JUDGE_usGetShooterRealHeat17(void);
STATIC uint16_t JUDGE_usGetShooterLimitHeat17(void);
STATIC uint8_t JUDGE_ucGetBulletLimitSpeed17(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static judge_info_t *sJudgeInfo = &judge_info;

/* Exported variables --------------------------------------------------------*/
judge_t judge = {
	.dev = &judge_sensor,
	.if_data_valid = JUDGE_IfDataValid,
    .my_color = JUDGE_GetMyColor,
	.get_17mm_heat_limit = JUDGE_usGetShooterLimitHeat17,
	.get_17mm_heat_real = JUDGE_usGetShooterRealHeat17,
	.get_17mm_speed_limit = JUDGE_ucGetBulletLimitSpeed17,
};
/* Private functions ---------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/
/**
 *	@brief	�����жϴ�ʱ����ϵͳ���ݵ���ȷ��
 */
STATIC bool JUDGE_IfDataValid(void)
{
	return sJudgeInfo->data_valid;
}

STATIC color_t JUDGE_GetMyColor(void)
{
    if(sJudgeInfo->game_robot_status.robot_id == 6)
        return RED;
    else
        return BLUE;
}

/**
 *	@brief	����������17mmǹ��ʵʱ����
 */
STATIC uint16_t JUDGE_usGetShooterRealHeat17(void)
{
	return (sJudgeInfo->power_heat_data.shooter_id1_17mm_cooling_heat);
}

/**
 *	@brief	����������17mmǹ����������
 */
STATIC uint16_t JUDGE_usGetShooterLimitHeat17(void)
{
	return (sJudgeInfo->game_robot_status.shooter_id1_17mm_cooling_limit);
}

/**
 *	@brief	����������17mm������������
 */
STATIC uint8_t JUDGE_ucGetBulletLimitSpeed17(void)
{
	return (sJudgeInfo->game_robot_status.shooter_id1_17mm_speed_limit);
}

/* Ӧ�ò� --------------------------------------------------------------------*/
/* ����� --------------------------------------------------------------------*/
