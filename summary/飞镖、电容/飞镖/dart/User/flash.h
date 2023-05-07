#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"

#define SECTOR_11_ADDRESS 0x080E0000    //存储区域起始地址

typedef __packed struct{
    uint8_t frame_header;   //0xA5
    uint8_t launch_num;  //0~3，发射序号
    uint8_t target;     //0:前哨站，1:基地，2:手动模式
    uint8_t hit;        //0:未命中，1:命中，2:手动模式
    int32_t pitch;
    int32_t yaw;
    int32_t gimbal;
}flash_info_t;

typedef struct{
    uint32_t flash_idle_offset; //新数据写入位置
    uint8_t read_flag;          //读取标志位，在debug中置1可以将read_offset_num位置的数据读取到flash_info结构体中
    uint8_t erase_flag;         //擦除所有数据
    uint8_t send_flag;          //置1可以将存储的数据从串口发送
    uint8_t empty_check_flag;   //置1可以检测存储区域的flash是否空白
    uint32_t empty_check_pin;   //从存储区域起始地址开始有多少个字节是空白的
    uint32_t read_offset_num;   //配合read_flag使用
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

