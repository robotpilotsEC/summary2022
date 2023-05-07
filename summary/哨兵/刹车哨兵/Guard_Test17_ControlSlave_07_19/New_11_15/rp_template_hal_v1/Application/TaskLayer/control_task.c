/**
 * @file        control_task.c
 * @author      RobotPilots
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "driver.h"
#include "chassis.h"
#include "gimbal.h"
#include "dial.h"
#include "cmsis_os.h"
#include "can_protocol.h"
#include "judge_sensor.h"
#include "string.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void leader_init(leader_t *leader);
static void leader_update(leader_t *leader, uint8_t *rxBuf);
static void leader_check(leader_t *leader);
static void leader_heart_beat(leader_t *leader);

static void rc_leader_init(rc_leader_t *rc_leader);
static void rc_leader_update(rc_leader_t *rc_leader, uint8_t *rxBuf);
static void rc_leader_check(rc_leader_t *rc_leader);
static void rc_leader_heart_beat(rc_leader_t *rc_leader);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t NormalPwm[FRICTION_MOTOR_CNT] = {0,0};
// Leader����
drv_can_t		leader_driver = {
	
		.id = DRV_CAN1,
		.rx_id = LEADER_CAN_ID,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 4,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
				.add_twoword = CAN_AddTwoWord,
        .manual_tx = CAN_ManualTx
	
};

// leader��Ϣ
leader_info_t 	leader_info = {
        .init_flag = false,
		.offline_max_cnt = 50,
};

// leader������
leader_t	leader_sensor = {
		.info = &leader_info,
		.driver = &leader_driver,
		.init = leader_init,
		.update = leader_update,
		.check = leader_check,
		.heart_beat = leader_heart_beat,
		.work_state = DEV_OFFLINE,
};

// RC_Leader����
drv_can_t		rc_leader_driver = {
	
		.id = DRV_CAN1,
		.rx_id = RC_SLAVE_CAN_ID,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 4,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
				.add_twoword = CAN_AddTwoWord,
        .manual_tx = CAN_ManualTx
	
};

// RC_leader��Ϣ
rc_leader_info_t 	rc_leader_info = {
        .init_flag = false,
		.offline_max_cnt = 50,
};

// RC_leader������
rc_leader_t	rc_leader_sensor = {
		.info = &rc_leader_info,
		.driver = &rc_leader_driver,
		.init = rc_leader_init,
		.update = rc_leader_update,
		.check = rc_leader_check,
		.heart_beat = rc_leader_heart_beat,
		.work_state = DEV_OFFLINE,
};

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Leader */
static void leader_init(leader_t *leader)
{
	leader_info_t *leader_info = leader->info;	
	drv_can_t *drv_can = leader->driver;
	if( !leader_info->init_flag )
	{
		leader_info->init_flag = true;
		leader_info->offline_cnt = 0;  //����Ҫ��������̨�����ݣ��������������ݸ��´�
	}
	drv_can->data_idx = 0;
	drv_can->tx_id = LEADER_CAN_ID;
}
static void leader_update(leader_t *leader, uint8_t *rxBuf)
{
	
}

static void leader_check(leader_t *leader)
{
	
}

static void leader_heart_beat(leader_t *leader)
{
	leader_sensor.work_state = DEV_ONLINE;
}

/* RC_Slave */
static void rc_leader_init(rc_leader_t *rc_leader)
{
	rc_leader_info_t *rc_leader_info = rc_leader->info;	
	drv_can_t *drv_can = rc_leader->driver;
	if( !rc_leader_info->init_flag )
	{
		rc_leader_info->init_flag = true;
		rc_leader_info->offline_cnt = 0;  //����Ҫ��������̨�����ݣ��������������ݸ��´�
	}
	drv_can->data_idx = 0;
	drv_can->tx_id = RC_SLAVE_CAN_ID;
}
static void rc_leader_update(rc_leader_t *rc_leader, uint8_t *rxBuf)
{
	
}

static void rc_leader_check(rc_leader_t *rc_leader)
{
	
}

static void rc_leader_heart_beat(rc_leader_t *rc_leader)
{
	rc_leader_sensor.work_state = DEV_ONLINE;
}

/* Exported functions --------------------------------------------------------*/
void LEADER_GetInfo(void)
{
	leader_info_t *leader_info = leader_sensor.info;
	/*----����ң��������λ----*/
	if(rc_sensor.work_state == DEV_ONLINE)    
	{
		leader_info->TxPacket.ctrl_mode.online = DEV_ONLINE;
	}
	else
	{
		leader_info->TxPacket.ctrl_mode.online = DEV_OFFLINE;		
	}
	/*----���¿���ģʽλ----*/
	if(sys.remote_mode == RC) 
	{
		leader_info->TxPacket.ctrl_mode.mode = RC;
	}
	else if(sys.remote_mode == KEY) 
	{
		leader_info->TxPacket.ctrl_mode.mode = KEY;
	}     
	else if(sys.remote_mode == AUTO)
	{
		leader_info->TxPacket.ctrl_mode.mode = AUTO;
	}
	else if(sys.remote_mode == INSPECTION) 
	{
		leader_info->TxPacket.ctrl_mode.mode = INSPECTION;
	}
	//����̨ǹ������(ǹ������������)
	if(judge_sensor_info.ShootData.shooter_id == 2)       
	{
		leader_info->TxPacket.bullet_speed = judge_sensor_info.ShootData.bullet_speed;
		if(judge_sensor_info.shoot_update)
		{
			leader_info->TxPacket.shoot_update = true;  //��������̨�����Ϣ
			judge_sensor_info.shoot_update = false;
		}
	}
	leader_info->TxPacket.cooling_heat = judge_sensor_info.PowerHeatData.shooter_id2_17mm_cooling_heat;   //����id2��ǹ����Ϣ
	/*----������ɫʶ��λ----*/
	if(judge_sensor.info->GameRobotStatus.robot_id == 7)//��ɫid
	{
		leader_info->TxPacket.ctrl_mode.attack_colour = 1;   //�з�����ɫ
	}
	else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//��ɫid
	{
		leader_info->TxPacket.ctrl_mode.attack_colour = 0;   //�����Ǻ�ɫ
	}
	
}

void RC_LEADER_GetInfo(void)
{
	
}

void LEADER_ShootCtrl(void)
{
	leader_info_t *leader_info = leader_sensor.info;
	
	//������̨Ħ����ͬ��
	if(dial.fric_open)
	{
		leader_info->TxPacket.ctrl_mode.fric_open = FRIC_OPEN;
	}
	else
	{
		leader_info->TxPacket.ctrl_mode.fric_open = FRIC_CLOSE;
	}
	//������̨���̽���ͬ��
	if(dial.lock)
	{
		leader_info->TxPacket.ctrl_mode.dial_lock_state = DIAL_LOCK;
	}
	else
	{
		leader_info->TxPacket.ctrl_mode.dial_lock_state = DIAL_UNLOCK;
	}

}


/**
*	@brief	leader���ݷ��ͣ�8���ֽ����ݣ�
 */
static void LEADER_DataSend(leader_t *leader)
{
	uint8_t txbuff[8];
	drv_can_t *leader_drv = leader->driver;	
	leader_info_t *leader_info = leader->info;
	
	memcpy(txbuff, &leader_info->TxPacket, LEADER_LEN_PACKED);
	leader_drv->add_twoword(leader_drv,	txbuff);
}

/**
*	@brief	leader���ݷ��ͣ�4���ֽ����ݣ�
 */
static void RC_LEADER_DataSend(rc_leader_t *rc_leader)
{
	uint8_t txbuff[4];
	drv_can_t *rc_leader_drv = rc_leader->driver;	
	rc_leader_info_t *rc_leader_info = rc_leader->info;
	
	memcpy(txbuff, &rc_leader_info->RC_Slave_TxPacket, RC_SLAVE_LEN_PACKED);
	CAN_AddMsgByDriver(rc_leader_drv, txbuff, 4);
}

/**
 *	@brief	ң��������������̨
 */
static void RC_ControlGimbal(void)
{
	rc_leader_info_t *rc_leader_info = rc_leader_sensor.info;
  static bool control_slave = false;
  static bool control_slave_access = false;
	
	if(sys.remote_mode == RC)   //ң����ģʽ�¿�ѡ�����������̨
	{
		if(RC_THUMB_WHEEL_VALUE <= -600)
		{
			if(control_slave_access)
			{
				if(control_slave)
				{
					control_slave = false;
				}
				else
				{
					control_slave = true;
				}
				
				control_slave_access = false;
			}
		}
		else
		{
			control_slave_access = true;
		}
	}

	if(control_slave)   //��������̨
	{
		rc_leader_info->RC_Slave_TxPacket.rc_slave_ch0 = rc_sensor.info->ch0;  //����ң����ͨ��ֵ������̨
		rc_leader_info->RC_Slave_TxPacket.rc_slave_ch1 = rc_sensor.info->ch1;		
		gimbal.rc_ch0 = 0;   //�������̨ͨ��ֵ
		gimbal.rc_ch1 = 0;
	}
	else if(!control_slave)  //��������̨
	{
		gimbal.rc_ch0 = rc_sensor.info->ch0;
		gimbal.rc_ch1 = rc_sensor.info->ch1;
		rc_leader_info->RC_Slave_TxPacket.rc_slave_ch0 = 0;   //�������̨ͨ��ֵ
		rc_leader_info->RC_Slave_TxPacket.rc_slave_ch1 = 0;
	}

}

/**
 *	@brief	������̨����
 */
static void Rc_Gimbal_Reset(void)
{
	rc_leader_info_t *rc_leader_info = rc_leader_sensor.info;
	
	gimbal.rc_ch0 = 0;   //�������̨ͨ��ֵ
	gimbal.rc_ch1 = 0;

	rc_leader_info->RC_Slave_TxPacket.rc_slave_ch0 = 0;  //�������̨ͨ��ֵ
	rc_leader_info->RC_Slave_TxPacket.rc_slave_ch1 = 0;
	
}

/**
 *	@brief	��������
 */
static void Control_Normal(void)
{	
	RC_ControlGimbal();  //ң��������������̨
	LED_RED_OFF();     //�����    
  LED_GREEN_ON();   //�̵���
	LASER_ON();      //���⿪
//  LASER_OFF();        //�����

}

/**
 *	@brief	leader����
 */
static void Control_Leader(void)
{
	/*----��Ϣ����----*/  
	LEADER_GetInfo();	 
	RC_LEADER_GetInfo();	 
	/*----����̨����ʽ����----*/
	LEADER_ShootCtrl();
	/*----���շ���----*/	
	LEADER_DataSend(&leader_sensor);
	RC_LEADER_DataSend(&rc_leader_sensor);
	
	
}

/**
 *	@brief	ֹͣ���ƣ�ϵͳ�쳣��
 */
static void Control_Stop()
{
	Rc_Gimbal_Reset();    //����������̨
  LED_RED_ON();        //�����
  LED_GREEN_OFF();    //�̵���
  LASER_ON();        //���⿪
//  LASER_OFF();        //�����

}

/**
 *	@brief	��������
 */
void StartControlTask(void const * argument)
{
	for(;;)
	{
		Control_Leader();
		if(sys.state == SYS_STATE_NORMAL) 
		{
			Control_Normal();
		} 
		else 
		{
			Control_Stop();
		}
		
		osDelay(2);
	}
}
