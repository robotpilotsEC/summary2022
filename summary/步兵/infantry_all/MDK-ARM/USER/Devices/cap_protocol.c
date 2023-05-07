#include "cap_protocol.h"
#include "judge_sensor.h"
#include "can_drv.h"
#include "super.h"

cap_send_data_t    cap_send_data;
cap_receive_data_t cap_receive_data;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAP_RP_t CAP_RP_2022;

int16_t can_data_0x2E[4] = {0};
int16_t can_data_0x2F[4] = {0};

extern judge_sensor_t	judge_sensor;

int16_t cap_output_limit = CAP_OUTPUT_LIMIT;

void set_message()
{
    cap_send_data.chassis_power_buffer = judge_sensor.info->power_heat_data.chassis_power_buffer;
    cap_send_data.chassis_power_limit = judge_sensor.info->game_robot_status.chassis_power_limit;
    cap_send_data.chassis_volt = judge_sensor.info->power_heat_data.chassis_volt;	//**
    cap_send_data.chassis_current = judge_sensor.info->power_heat_data.chassis_current;	//**
		cap_send_data.output_power_limit = cap_output_limit;
    cap_send_data.input_power_limit = 150;
	
    cap_send_data.cap_control.bit.cap_switch = 1;
    cap_send_data.cap_control.bit.cap_record = 0;
	
    if(judge_sensor.info->game_status.game_progress == 4)	//***
    {
    	cap_send_data.cap_control.bit.gamegoing = 1;
		}
		else
		{
			cap_send_data.cap_control.bit.gamegoing = 0;
		}	
}

//����ϵͳ���͹��ʻ����Ƶ��Ϊ50Hz���ú�����Ҫÿ20ms����һ�Ρ�
void can_send_0x2E(char can)
{
    can_data_0x2E[0] = cap_send_data.chassis_power_buffer;    //���̹��ʻ��壬0~60J
    can_data_0x2E[1] = cap_send_data.chassis_volt;    	//���������ѹ ��λ ���� **
    can_data_0x2E[2] = cap_send_data.chassis_current;    //����������� ��λ ���� **
	
		if(can == 1)
			CAN_SendData(&hcan1, (uint32_t)0x2E, can_data_0x2E);
		
		else if(can == 2)
			CAN_SendData(&hcan2, (uint32_t)0x2E, can_data_0x2E);
}

void can_send_0x2F(char can)
{
    can_data_0x2F[0] = cap_send_data.chassis_power_limit;   //���̹����������ޣ�0~120W
    can_data_0x2F[1] = cap_send_data.output_power_limit;    //���ݷŵ繦�����ƣ�-120~300W
    can_data_0x2F[2] = cap_send_data.input_power_limit;    //���ݳ�繦�����ƣ�0~150W
    can_data_0x2F[3] = cap_send_data.cap_control.all;            //���ݿ��أ�0���رգ���1��������
	
		if(can == 1)
			CAN_SendData(&hcan1, (uint32_t)0x2F, can_data_0x2F);
		
		else if(can == 2)
			CAN_SendData(&hcan2, (uint32_t)0x2F, can_data_0x2F);
}

void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x30)
    {
        cap_receive_data.cap_Ucr = int16_to_float(((uint16_t)rxBuf[0] << 8| rxBuf[1]), 32000, -32000, 30, 0);
        cap_receive_data.cap_I = int16_to_float(((uint16_t)rxBuf[2] << 8| rxBuf[3]), 32000, -32000, 20, -20);
        cap_receive_data.cap_state.state = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);    
        CAP_RP_2022.offline_cnt = 0;			
    }
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;   //��0.5ʹ����ȡ�������������
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}


