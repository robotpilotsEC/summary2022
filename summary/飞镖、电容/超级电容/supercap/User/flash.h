#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"

#define BASE_ADDRESS 0x0800C000     //��ʼ������ַ��ע�ⲻҪռ���Ѿ�������ռ���˵�λ��
#define END_ADDRESS  0x08010000     //���һ������ĩβ��ַ+1

typedef __packed struct{
    uint8_t frame_header;   //0xA5
    uint8_t cap_control;
    uint16_t chassis_power_buffer;  //���̹��ʻ���
    uint16_t chassis_volt;
    uint16_t chassis_current;
    int16_t output_power_limit;     //���ݷŵ繦������
    uint16_t cap_v;
}flash_info_t;

typedef struct{
    uint32_t flash_head_offset;     //��һ�����ݵĵ�ַ
    int32_t flash_tail_offset;     //���һ�����ݵĵ�ַ+1
    uint8_t read_flag;
    uint8_t erase_flag;
    uint8_t send_flag;
    uint8_t empty_check_flag;
    uint32_t empty_check_pin;
    uint32_t read_offset_num;
}flash_cmd_t;

extern flash_info_t flash_info;

void Erase_Flash_SECTOR_ALL(void);
void Flash_Init(void);
void Flash_Write(void);
void Flash_Read(void);
void Flash_Erase(void);
void Flash_Empty_Check(void);
void Flash_usart_solve(void);

#endif

