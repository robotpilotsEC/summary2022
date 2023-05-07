/**
 * @file        judge_potocol.c
 * @author      Sentry@2021
 * @Version     V1.0
 * @date        19-February-2021
 * @brief       .
 */

/* Includes ------------------------------------------------------------------*/
#include  "judge_protocol.h"
#include  "judge_sensor.h"
#include "string.h"
/* Private macro -------------------------------------------------------------*/
#define JUDGE_FRAME_HEADER		(0xA5)
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* ֡�ֽ�ƫ�� */
typedef enum {
    FRAME_HEADER	= 0,
    CMD_ID			= 5,
    DATA_SEG		= 7
} Judge_Frame_Offset_t;

/* ֡ͷ�ֽ�ƫ�� */
typedef enum {
    J_SOF			= 0,
    DATA_LENGTH	= 1,
    SEQ			= 3,
    J_CRC8		= 4
} Judge_Frame_Header_Offset_t;

typedef enum {
    ID_GAME_STATUS 					= 0x0001,	// ����״̬
    ID_GAME_RESULT 					= 0x0002,	// �������
    ID_GAME_ROBOT_HP 				= 0x0003,	// ������Ѫ������
    ID_DART_STATUS					= 0x0004,	// ���ڷ���״̬
    ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005,	// �˹�������ս���ӳ���ͷ���״̬

    ID_EVENT_DATA 					= 0x0101,	// �����¼�����
    ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// ����վ������ʶ
    //ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// ���󲹸�վ�����ӵ�
    ID_REFEREE_WARNING 				= 0x0104,	// ���о�����Ϣ
    ID_DART_REMAINING_TIME			= 0x0105,	// ���ڷ���ڵ���ʱ

    ID_GAME_ROBOT_STATUS 			= 0x0201,	// ����������״̬
    ID_POWER_HEAT_DATA 				= 0x0202,	// ʵʱ������������
    ID_GAME_ROBOT_POS				= 0x0203,	// ������λ��
    ID_BUFF							= 0x0204,	// ����������
    ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// ���л���������״̬
    ID_ROBOT_HURT					= 0x0206,	// �������˺�״̬
    ID_SHOOT_DATA					= 0x0207,	// ʵʱ�����Ϣ
    ID_BULLET_REMAINING				= 0x0208,	// �ӵ�ʣ�෢����
    ID_RFID_STATUS					= 0x0209,	// ������RFID״̬

    ID_COMMUNICATION				= 0x0301,	// �����˼佻������(���ͷ���������)
    ID_COMMAND						= 0x0303,	// �����˼佻������(���ͷ���������)	
} Judge_Cmd_ID_t;

typedef enum {
    /* Std */
    LEN_FRAME_HEAD 	= 5,	// ֡ͷ����
    LEN_CMD_ID 		= 2,	// �����볤��
    LEN_FRAME_TAIL 	= 2,	// ֡βCRC16
    /* Ext */
    // 0x000x
    LEN_GAME_STATUS 				= 11,
    LEN_GAME_RESULT 				= 1,
    LEN_GAME_ROBOT_HP 				= 32,
    LEN_DART_STATUS					= 3,
    LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS= 11,

    // 0x010x
    LEN_EVENT_DATA					= 4,
    LEN_SUPPLY_PROJECTILE_ACTION	= 4,
    LEN_REFEREE_WARNING				= 2,
    LEN_DART_REMAINING_TIME			= 1,

    // 0x020x
    LEN_AERAL_DATA			        = 8,
    LEN_GAME_ROBOT_STATUS			= 27,
    LEN_POWER_HEAT_DATA 			= 16,
    LEN_GAME_ROBOT_POS				= 16,
    LEN_BUFF		 				= 1,
    LEN_AERIAL_ROBOT_ENERGY 		= 1,
    LEN_ROBOT_HURT					= 1,
    LEN_SHOOT_DATA					= 7,
    LEN_BULLET_REMAINING	 		= 6,
    LEN_RFID_STATUS					= 4,
} Judge_Data_Length_t;
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void judge_init(judge_sensor_t *judge)
{
    // ��ʼ��Ϊ����״̬
    judge->info->offline_cnt = judge->info->offline_max_cnt + 1;
    judge->work_state = DEV_OFFLINE;

    if(judge->id == DEV_ID_VISION)
        judge->errno = NONE_ERR;
    else
        judge->errno = DEV_ID_ERR;
}
void judge_update(uint8_t *rxBuf)
{

    uint8_t  res = false;
    uint16_t frame_length;
    uint16_t cmd_id;
    judge_info_t *judge_info = judge_sensor.info;
    memcpy(&judge_info->FrameHeader, rxBuf, LEN_FRAME_HEAD);

    /* ֡���ֽ��Ƿ�Ϊ0xA5 */
    if(rxBuf[J_SOF] == 0xa5)
    {
        /* ֡ͷCRC8У�� */
        if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true)
        {
            /* ͳ��һ֡�������ݳ��ȣ�����CRC16У�� */
            frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->FrameHeader.data_length + LEN_FRAME_TAIL;
            judge_info->frame_length = frame_length;

            if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
            {
                res = true;
                judge_info->offline_cnt = 0;
                cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
                judge_info->cmd_id = cmd_id;

                switch(cmd_id)
                {
                case ID_GAME_STATUS: {
                    memcpy(&judge_info->GameStatus, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
                }
                break;

                case ID_GAME_RESULT: {
                    memcpy(&judge_info->GameResult,  (rxBuf+DATA_SEG), LEN_GAME_RESULT);
                }
                break;

                case ID_GAME_ROBOT_HP: {
                    memcpy(&judge_info->GameRobotHP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
                }
                break;

                case ID_DART_STATUS: {
                    memcpy(&judge_info->DartStatus, (rxBuf+DATA_SEG), LEN_DART_STATUS);
                    judge_info->dart_data_update = true;	// �������ݸ���
                }
                break;

                case ID_EVENT_DATA: {
                    memcpy(&judge_info->EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
                }
                break;

                case ID_SUPPLY_PROJECTILE_ACTION: {
                    memcpy(&judge_info->SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
                    judge_info->supply_data_update = true;	// ����վ���ݸ���
                }
                break;

                case ID_REFEREE_WARNING: {
                    memcpy(&judge_info->RefereeWarning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
                }
                break;

                case ID_DART_REMAINING_TIME: {
                    memcpy(&judge_info->DartRemainingTime, (rxBuf+DATA_SEG), LEN_DART_REMAINING_TIME);
                }
                break;

                case ID_GAME_ROBOT_STATUS: {
                    memcpy(&judge_info->GameRobotStatus, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
                }
                break;

                case ID_POWER_HEAT_DATA: {
                    memcpy(&judge_info->PowerHeatData, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
                    judge_info->power_heat_update = true;
                }
                break;

                case ID_GAME_ROBOT_POS: {
                    memcpy(&judge_info->GameRobotPos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
                }
                break;

                case ID_BUFF: {
                    memcpy(&judge_info->Buff, (rxBuf+DATA_SEG), LEN_BUFF);
                }
                break;

                case ID_AERIAL_ROBOT_ENERGY: {
                    memcpy(&judge_info->AerialRobotEnergy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
                }
                break;

                case ID_ROBOT_HURT: {
                    memcpy(&judge_info->RobotHurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
                    judge_info->hurt_data_update = true;	// �˺����ݸ���
                }
                break;

                case ID_SHOOT_DATA: {
                    memcpy(&judge_info->ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
//							JUDGE_ShootNumCount();	// ���㷢����
                    judge_info->shoot_update = true;
                }
                break;

                case ID_BULLET_REMAINING: {
                    memcpy(&judge_info->BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
                }
                break;

                case ID_RFID_STATUS: {
                    memcpy(&judge_info->RfidStatus, (rxBuf+DATA_SEG), LEN_RFID_STATUS);
                }
                break;

                case ID_COMMUNICATION: {
                    memcpy(&judge_info->AerialData, (rxBuf+DATA_SEG), LEN_AERAL_DATA);  //��̨������
					judge_info->communication_data_update = true;					
                }
                break;
				
				case ID_COMMAND:{
					memcpy(&judge_info->command, (rxBuf+DATA_SEG), 15);  
					judge_info->command_data_update = true;
				}
                }
            }
        }
        /* ֡βCRC16��һ�ֽ��Ƿ�Ϊ0xA5 */
        if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
        {
            /* ���һ�����ݰ������˶�֡���ݾ��ٴζ�ȡ */
            judge_update( &rxBuf[frame_length] );
        }
    }

    judge_info->data_valid = res;
    if(judge_info->data_valid != true) {
        judge_info->err_cnt++;
        judge_info->data_valid = false;
    }
    else {
        judge_info->data_valid = true;
    }

//	return res;
}
/**
 *	@brief	�ڴ���2�н���ң������Э��
 */
void USART5_rxDataHandler(uint8_t *rxBuf)
{
    // ���²���ϵͳ����
    judge_sensor.update(rxBuf);
    judge_sensor.check(&judge_sensor);
}
