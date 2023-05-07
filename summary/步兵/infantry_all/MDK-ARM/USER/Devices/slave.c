#include "slave.h"
#include "string.h"
#include "imu.h"
#include "motor.h"
#include "UI.h"

/*-------------------------------------------------------------

����ʹ��can1
���Ͳ���ϵͳ����
����������


����ʹ��can2
��������������
���Ͷ������ٶȽǶȣ��жϷ��ͣ�����״̬�������ͣ�

-------------------------------------------------------------*/



extern Client_Slave_Flag Slaver_flag;
extern judge_sensor_t judge_sensor;
extern imu_sensor_t 	imu_sensor;

uint8_t can_judge_data[8];

char Get_Rudder_WorkState(void);

slave_t SLAVE = 
{
 .work_state = DEV_OFFLINE,	
 .offline_max_cnt = 5000,
};


//ң��ʧ��ʱ�����Ϸ��͵������0ռ��can,����ʧ��
void Up_RX(uint32_t ID, uint8_t *date)
{
	judge_info_t *info = judge_sensor.info;
	
	
	switch(ID)
	{
#if CARR_MODE == 0 || CARR_MODE == 2 || CARR_MODE == 4
		
		case 0x101:
		memcpy(&SLAVE.pack1, date, 8);
	  info->power_heat_data.chassis_power                 = SLAVE.pack1.chassis_power;
	  info->power_heat_data.chassis_power_buffer          = SLAVE.pack1.chassis_power_buffer;
	  info->power_heat_data.shooter_id1_17mm_cooling_heat = SLAVE.pack1.shooter_id1_17mm_cooling_heat;
    info->offline_cnt = 0;
		SLAVE.offline_cnt = 0;
		break;
		
		case 0x302:
		memcpy(&SLAVE.pack2, date, 8);
		info->game_robot_status.chassis_power_limit            = SLAVE.pack2.chassis_power_limit;
		info->game_robot_status.shooter_id1_17mm_cooling_limit = SLAVE.pack2.shooter_id1_17mm_cooling_limit;
		info->game_robot_status.shooter_id1_17mm_speed_limit   = SLAVE.pack2.shooter_id1_17mm_speed_limit;
		info->game_robot_status.robot_id = SLAVE.pack2.id;	
		info->offline_cnt = 0;		
		SLAVE.pack2.imu_work_state = SLAVE.pack2.imu_work_state;		
		SLAVE.offline_cnt = 0;
		break;
		
		case 0x303:
		memcpy(&SLAVE.pack3, date,  8);
		SLAVE.pack3.slave_pit         = SLAVE.pack3.slave_pit;
		SLAVE.pack3.slave_roll        = SLAVE.pack3.slave_roll;
		info->shoot_data.bullet_speed = SLAVE.pack3.bullet_speed;
		SLAVE.offline_cnt = 0;
		break;
		
#endif
		
#if CARR_MODE == 1

		case 0x305:
		memcpy(&SLAVE.pack4, date,  8);
		SLAVE.offline_cnt = 0;
		break;
		
#endif
		
#if CARR_MODE == 3
		
		case 0x101:
		memcpy(&SLAVE.pack1, date, 8);
	  info->power_heat_data.chassis_power                 = SLAVE.pack1.chassis_power;
	  info->power_heat_data.chassis_power_buffer          = SLAVE.pack1.chassis_power_buffer;
	  info->power_heat_data.shooter_id1_17mm_cooling_heat = SLAVE.pack1.shooter_id1_17mm_cooling_heat;
    info->offline_cnt = 0;
		SLAVE.offline_cnt = 0;
		break;

		case 0x102:
		memcpy(&SLAVE.pack6, date, 6);
		info->power_heat_data.shooter_id2_17mm_cooling_heat = SLAVE.pack6.shooter_id2_17mm_cooling_heat;
		info->game_robot_status.shooter_id2_17mm_cooling_limit = SLAVE.pack6.shooter_id2_17mm_cooling_limit;		
		info->game_robot_status.shooter_id2_17mm_speed_limit = SLAVE.pack6.shooter_id2_17mm_speed_limit;
		info->offline_cnt = 0;		
		SLAVE.offline_cnt = 0;
		break;
		
		case 0x300:
		memcpy(&SLAVE.pack3, date,  8);
		SLAVE.pack3.slave_pit         = SLAVE.pack3.slave_pit;
		SLAVE.pack3.slave_roll        = SLAVE.pack3.slave_roll;
		info->shoot_data.bullet_speed = SLAVE.pack3.bullet_speed;
		SLAVE.offline_cnt = 0;
		break;
		
		case 0x302:
		memcpy(&SLAVE.pack2, date, 8);
		info->game_robot_status.chassis_power_limit            = SLAVE.pack2.chassis_power_limit;
		info->game_robot_status.shooter_id1_17mm_cooling_limit = SLAVE.pack2.shooter_id1_17mm_cooling_limit;
		info->game_robot_status.shooter_id1_17mm_speed_limit   = SLAVE.pack2.shooter_id1_17mm_speed_limit;
		info->game_robot_status.robot_id = SLAVE.pack2.id;	
		info->offline_cnt = 0;		
		SLAVE.pack2.imu_work_state = SLAVE.pack2.imu_work_state;		
		SLAVE.offline_cnt = 0;
		break;

#endif		
		
		
	}
}



void Down_Send(judge_sensor_t *judge)
{
	static uint32_t i;
	i++;
	
#if CARR_MODE == 0 || CARR_MODE == 2 || CARR_MODE == 4
	
	judge_info_t *info = judge->info;

	if(JUDGE_ONLINE)
	{
		if(i%3)
		{
			SLAVE.pack1.chassis_power = info->power_heat_data.chassis_power;
			SLAVE.pack1.chassis_power_buffer = info->power_heat_data.chassis_power_buffer;
			SLAVE.pack1.shooter_id1_17mm_cooling_heat = info->power_heat_data.shooter_id1_17mm_cooling_heat;	
			memcpy(can_judge_data, &SLAVE.pack1, 8);
			HXZP_Tx_uint8(0x101, can_judge_data,2,0x08);
		}
		
		SLAVE.pack2.chassis_power_limit = info->game_robot_status.chassis_power_limit;
		SLAVE.pack2.shooter_id1_17mm_cooling_limit = info->game_robot_status.shooter_id1_17mm_cooling_limit;
		SLAVE.pack2.shooter_id1_17mm_speed_limit = info->game_robot_status.shooter_id1_17mm_speed_limit;
	//	SLAVE.pack2.judge_offline_cnt = judge_sensor.info->offline_cnt;	
		SLAVE.pack2.id = info->game_robot_status.robot_id;
		SLAVE.pack2.imu_work_state = imu_sensor.work_state;
		memcpy(can_judge_data, &SLAVE.pack2, 8);
		HXZP_Tx_uint8(0x302, can_judge_data,2,0x08);
	}
	
	SLAVE.pack3.bullet_speed = info->shoot_data.bullet_speed;
	if(imu_sensor.work_state == DEV_ONLINE){
		SLAVE.pack3.slave_pit = imu_sensor.info->pitch;
		SLAVE.pack3.slave_roll= imu_sensor.info->roll;
	}
	else{
		SLAVE.pack3.slave_pit = 4095;
		SLAVE.pack3.slave_roll= 0;
	}	
	memcpy(can_judge_data, &SLAVE.pack3,  8);
	HXZP_Tx_uint8(0x303, can_judge_data,2,0x08);	
	

#endif

#if CARR_MODE == 1 

	SLAVE.pack4.slave_pit = imu_sensor.info->pitch;
	SLAVE.pack4.slave_roll= imu_sensor.info->roll;
	SLAVE.pack4.imu_work_state = imu_sensor.work_state;
	memcpy(can_judge_data, &SLAVE.pack4,  8);
	HXZP_Tx_uint8(0x305, can_judge_data,2,0x08);		

#endif

#if CARR_MODE == 3	
	judge_info_t *info = judge->info;

	if(JUDGE_ONLINE)
	{
		if(i%3)
		{
			SLAVE.pack1.chassis_power = info->power_heat_data.chassis_power;
			SLAVE.pack1.chassis_power_buffer = info->power_heat_data.chassis_power_buffer;
			SLAVE.pack1.shooter_id1_17mm_cooling_heat = info->power_heat_data.shooter_id1_17mm_cooling_heat;	
			memcpy(can_judge_data, &SLAVE.pack1, 8);
			HXZP_Tx_uint8(0x101, can_judge_data,2,0x08);
		}

		SLAVE.pack6.shooter_id2_17mm_cooling_heat = info->power_heat_data.shooter_id2_17mm_cooling_heat;	
		SLAVE.pack6.shooter_id2_17mm_cooling_limit = info->game_robot_status.shooter_id2_17mm_cooling_limit;		
		SLAVE.pack6.shooter_id2_17mm_speed_limit = info->game_robot_status.shooter_id2_17mm_speed_limit;
		memcpy(can_judge_data, &SLAVE.pack6, 6);
		HXZP_Tx_uint8(0x102, can_judge_data,2,0x06);	

		SLAVE.pack2.chassis_power_limit = info->game_robot_status.chassis_power_limit;
		SLAVE.pack2.shooter_id1_17mm_cooling_limit = info->game_robot_status.shooter_id1_17mm_cooling_limit;
		SLAVE.pack2.shooter_id1_17mm_speed_limit = info->game_robot_status.shooter_id1_17mm_speed_limit;
		SLAVE.pack2.id = info->game_robot_status.robot_id;
		SLAVE.pack2.imu_work_state = imu_sensor.work_state;
		memcpy(can_judge_data, &SLAVE.pack2, 8);
		HXZP_Tx_uint8(0x302, can_judge_data,2,0x08);
	}
	
	SLAVE.pack3.bullet_speed = info->shoot_data.bullet_speed;
	if(imu_sensor.work_state == DEV_ONLINE)
	{
		SLAVE.pack3.slave_pit = imu_sensor.info->pitch;
		SLAVE.pack3.slave_roll= imu_sensor.info->roll;
	}
	else
	{
		SLAVE.pack3.slave_pit = 4095;
		SLAVE.pack3.slave_roll= 0;
	}	
	memcpy(can_judge_data, &SLAVE.pack3,  8);
	HXZP_Tx_uint8(0x300, can_judge_data,2,0x08);
	
#endif
	
}

/*------------------------------------------------------------------------------------------*/

extern Client_Slave_Flag Slaver_flag;
void Up_Send(void)
{
#if CARR_MODE == 0	
	
	SLAVE.pack5.fiction = Slaver_flag.global_fiction;
	SLAVE.pack5.clip = Slaver_flag.global_clip;
	SLAVE.pack5.spin = Slaver_flag.global_spin;
	SLAVE.pack5.auto_aim = Slaver_flag.global_auto_aim;
	SLAVE.pack5.auto_arrive = Slaver_flag.global_auto_arrive;
	SLAVE.pack5.move_mode = Slaver_flag.move_mode;
	SLAVE.pack5.barrel_type = Slaver_flag.barrel_type;	
	memcpy(can_judge_data, &SLAVE.pack5,  8);
	HXZP_Tx_uint8(0x301, can_judge_data,1,0x08);	
	
//	SLAVE.pack7.yaw_angle = Slaver_flag.barrel_type;	
//	memcpy(can_judge_data, &SLAVE.pack5,  8);
//	HXZP_Tx_uint8(0x304, can_judge_data,1,0x08);		
//		
#endif	
	
#if CARR_MODE != 0	
	
	SLAVE.pack5.fiction = Slaver_flag.global_fiction;
	SLAVE.pack5.clip = Slaver_flag.global_clip;
	SLAVE.pack5.spin = Slaver_flag.global_spin;
	SLAVE.pack5.auto_aim = Slaver_flag.global_auto_aim;
	SLAVE.pack5.auto_arrive = Slaver_flag.global_auto_arrive;
	SLAVE.pack5.move_mode = Slaver_flag.move_mode;
	SLAVE.pack5.barrel_type = Slaver_flag.barrel_type;	
	memcpy(can_judge_data, &SLAVE.pack5,  8);
	HXZP_Tx_uint8(0x301, can_judge_data,2,0x08);	
	
#endif	
}



void Down_RX(uint32_t ID, uint8_t *date)
{
	switch(ID)
	{
		case 0x301:
		memcpy(&SLAVE.pack5, date, 8);
	  Slaver_flag.global_fiction = SLAVE.pack5.fiction;
	  Slaver_flag.global_clip = SLAVE.pack5.clip;
	  Slaver_flag.global_spin = SLAVE.pack5.spin;
		Slaver_flag.global_auto_aim = SLAVE.pack5.auto_aim;
		Slaver_flag.global_auto_arrive = SLAVE.pack5.auto_arrive;
		Slaver_flag.move_mode = SLAVE.pack5.move_mode;
		Slaver_flag.barrel_type = SLAVE.pack5.barrel_type;
		SLAVE.offline_cnt = 0;
		break;
	}

}








void slave_heart_beat(void)
{
	SLAVE.offline_cnt++;
	if(SLAVE.offline_cnt > SLAVE.offline_max_cnt) 
	{
		SLAVE.offline_cnt = SLAVE.offline_max_cnt;
		SLAVE.work_state = DEV_OFFLINE;
	}
	else 
	{
		if(SLAVE.work_state == DEV_OFFLINE)
			 SLAVE.work_state =  DEV_ONLINE;
	}
	
	if(SLAVE.work_state == DEV_OFFLINE)
	{
		
		SLAVE.pack4.imu_work_state = DEV_OFFLINE;
		SLAVE.pack2.imu_work_state = DEV_OFFLINE;
		
	}
}


