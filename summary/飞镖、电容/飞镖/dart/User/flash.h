#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"

#define SECTOR_11_ADDRESS 0x080E0000    //�洢������ʼ��ַ

typedef __packed struct{
    uint8_t frame_header;   //0xA5
    uint8_t launch_num;  //0~3���������
    uint8_t target;     //0:ǰ��վ��1:���أ�2:�ֶ�ģʽ
    uint8_t hit;        //0:δ���У�1:���У�2:�ֶ�ģʽ
    int32_t pitch;
    int32_t yaw;
    int32_t gimbal;
}flash_info_t;

typedef struct{
    uint32_t flash_idle_offset; //������д��λ��
    uint8_t read_flag;          //��ȡ��־λ����debug����1���Խ�read_offset_numλ�õ����ݶ�ȡ��flash_info�ṹ����
    uint8_t erase_flag;         //������������
    uint8_t send_flag;          //��1���Խ��洢�����ݴӴ��ڷ���
    uint8_t empty_check_flag;   //��1���Լ��洢�����flash�Ƿ�հ�
    uint32_t empty_check_pin;   //�Ӵ洢������ʼ��ַ��ʼ�ж��ٸ��ֽ��ǿհ׵�
    uint32_t read_offset_num;   //���read_flagʹ��
}flash_cmd_t;

extern flash_info_t flash_info;
extern flash_cmd_t flash_cmd;

void Erase_Flash_SECTOR_11(void);
void Flash_Init(void);
void Flash_Write(void);
void Flash_Read(void);
void Flash_Erase(void);
void Flash_Empty_Check(void);
void Flash_usart_send(void);

#endif

