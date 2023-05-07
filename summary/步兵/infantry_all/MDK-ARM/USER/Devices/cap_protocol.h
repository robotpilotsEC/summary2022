#ifndef __CAP_PROTOCOL_H
#define __CAP_PROTOCOL_H

#include "main.h"

typedef struct
{
    uint16_t chassis_power_buffer;  //���̹��ʻ��壬0~60J
    uint16_t chassis_power_limit;   //�����˵��̹����������ޣ�0~120W
    int16_t  output_power_limit;     //���ݷŵ繦�����ƣ�-120~300W
    uint16_t input_power_limit;     //���ݳ�繦�����ƣ�0~150W
    uint16_t chassis_volt;			//���������ѹ ��λ ���� **
	  uint16_t chassis_current;		//����������� ��λ ���� **
 
		union{
					uint16_t all;
					struct
					{
							uint16_t cap_switch : 1;    //���ݿ���
							uint16_t cap_record : 1;    //��¼���ܿ���
              uint16_t gamegoing : 1;     //����������Ϊ1������Ϊ0 ***						
					}bit;
			}cap_control;
		
}cap_send_data_t;

typedef struct
{
    float cap_Ucr;    //�������˵�ѹUcr��0~30V
    float cap_I;    //���ݵ���I��-20~20A
    union
    {
        uint16_t state;     //����״̬
        struct
        {
            uint16_t warning : 1;   //����
            uint16_t cap_U_over : 1;    //���ݹ�ѹ
            uint16_t cap_I_over : 1;    //���ݹ���
            uint16_t cap_U_low : 1;     //����Ƿѹ
            uint16_t bat_I_low : 1;     //����ϵͳǷѹ
            uint16_t can_receive_miss : 1;    //����δ���յ�CANͨ������
        }bit;
    }cap_state;
}cap_receive_data_t;

extern int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
extern float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
void can_send_0x2E(char can);
void can_send_0x2F(char can);
void set_message(void);
void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
#endif

