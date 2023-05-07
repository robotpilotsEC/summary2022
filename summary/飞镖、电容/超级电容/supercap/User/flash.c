#include "flash.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"

flash_info_t flash_info = {
    .frame_header = 0xA5,   //帧头
};
flash_cmd_t flash_cmd;

void Erase_Flash_SECTOR_ALL()//使用的扇区全部擦除
{
	uint32_t F;
	FLASH_EraseInitTypeDef FLash;
	HAL_FLASH_Unlock();
	FLash.TypeErase = FLASH_TYPEERASE_PAGES;
	FLash.PageAddress = BASE_ADDRESS;
	FLash.NbPages = 8;
	HAL_FLASHEx_Erase(&FLash, &F);
	HAL_FLASH_Lock();
}

void Erase_Flash_SECTOR_NOW()//擦除当前写入数据最后一个字节所在的扇区
{
    uint32_t F;
	FLASH_EraseInitTypeDef FLash;
	HAL_FLASH_Unlock();
	FLash.TypeErase = FLASH_TYPEERASE_PAGES;
	FLash.PageAddress = BASE_ADDRESS + (((flash_cmd.flash_tail_offset + 1) * sizeof(flash_info_t) - 1) / 2048) * 2048;  //F334C8T6每页2048个字节
	FLash.NbPages = 1;
	HAL_FLASHEx_Erase(&FLash, &F);
	HAL_FLASH_Lock();
}

void Flash_Init()//上电后读flash寻找可以写入的初始位置
{
    uint32_t i=0;
	for(; i<(END_ADDRESS - BASE_ADDRESS)/sizeof(flash_info_t); i++)
	{
		if(((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->frame_header == 0xFF)
		{
            //帧头位置为空，退出循环
			break;
		}
	}
    flash_cmd.flash_tail_offset = i < (END_ADDRESS - BASE_ADDRESS)/sizeof(flash_info_t) ? i : 0;
    
    for(; i<(END_ADDRESS - BASE_ADDRESS)/sizeof(flash_info_t); i++)
	{
		if(((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->frame_header == 0xA5)
		{
			break;
		}
	}
    flash_cmd.flash_head_offset = i < (END_ADDRESS - BASE_ADDRESS)/sizeof(flash_info_t) ? i : 0;
}

extern int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
extern float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);

uint32_t data_temp[3];  //建立一个数组，与要写入的数据大小相同
void Flash_Write()  //保存结构体
{
    if(((flash_info_t*)(BASE_ADDRESS + flash_cmd.flash_tail_offset*sizeof(flash_info_t)))->cap_v != 0xFFFF)
    {
        //如果写入数据的结尾位置不为空，则擦除结尾所在片区
        Erase_Flash_SECTOR_NOW();
    }
    memcpy(data_temp, &flash_info, sizeof(flash_info_t));
    HAL_FLASH_Unlock();
    for(uint8_t i=0; i<3; i++)
    {
        //12字节数据，每次写入4字节，写入3次
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BASE_ADDRESS + flash_cmd.flash_tail_offset*sizeof(flash_info_t) + 4*i, data_temp[i]);
    }
	HAL_FLASH_Lock();
    if(flash_cmd.flash_tail_offset < (END_ADDRESS - BASE_ADDRESS)/sizeof(flash_info_t) - 1)
    {
        flash_cmd.flash_tail_offset++;
    }
    else
    {
        flash_cmd.flash_tail_offset = 0;
    }
}

void Flash_Read()//读取结构体
{
    if(flash_cmd.read_flag == 1)
    {
        flash_cmd.read_flag = 0;
        memcpy(&flash_info, (void *)(BASE_ADDRESS + flash_cmd.read_offset_num*sizeof(flash_info_t)), sizeof(flash_info_t));
    }
}

void Flash_Erase()
{
    if(flash_cmd.erase_flag == 1)
    {
        flash_cmd.erase_flag = 0;
        Erase_Flash_SECTOR_ALL();
        flash_cmd.flash_head_offset = 0;
        flash_cmd.flash_tail_offset = 0;
    }
}

void Flash_Empty_Check()
{
    if(flash_cmd.empty_check_flag == 1)
    {
        flash_cmd.empty_check_flag = 0;
        for(flash_cmd.empty_check_pin = 0; flash_cmd.empty_check_pin < END_ADDRESS - BASE_ADDRESS; flash_cmd.empty_check_pin++)
        {
            if(*((uint8_t *)(BASE_ADDRESS + flash_cmd.empty_check_pin)) != 0xFF)
            {
                //遇到非空字节退出循环，若全为空，则empty_check_pin等于使用空间的字节数
                break;
            }
        }
    }
}

void Flash_usart_solve()
{
    uint8_t sendbuff[15] = {0};
    uint8_t receive_flag = 0;
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
    {
        //检测到RXNE接收非空标志位置1就读一个字节数据
        HAL_UART_Receive(&huart2, &receive_flag, 1, 10);
    }
    if(flash_cmd.send_flag == 1 || receive_flag == '?')
    {
        flash_cmd.send_flag = 0;
        receive_flag = 0;
        int32_t i = flash_cmd.flash_head_offset;
        if(flash_cmd.flash_head_offset >= flash_cmd.flash_tail_offset)
        {
            for(; i<(END_ADDRESS - BASE_ADDRESS)/sizeof(flash_info_t); i++)
            {
                if(((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->frame_header != 0xA5)break;
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->cap_control);
                HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->chassis_power_buffer);
                HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->chassis_volt);
                HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->chassis_current);
                HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->output_power_limit);
                HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%.1f \r\n", int16_to_float(((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->cap_v, 32000, -32000, 30, 0));
                HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
            }
            i = 0;
        }
        for(; i < flash_cmd.flash_tail_offset - 1; i++)
        {
            if(((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->frame_header != 0xA5)break;
            sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->cap_control);
            HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
            sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->chassis_power_buffer);
            HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
            sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->chassis_volt);
            HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
            sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->chassis_current);
            HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
            sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->output_power_limit);
            HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
            sprintf((char *)sendbuff, "%.1f \r\n", int16_to_float(((flash_info_t*)(BASE_ADDRESS + i*sizeof(flash_info_t)))->cap_v, 32000, -32000, 30, 0));
            HAL_UART_Transmit(&huart2, sendbuff, strlen((char *)sendbuff), 100);
        }
    }
    else if(receive_flag == '/'){
        flash_cmd.erase_flag = 1;
        receive_flag = 0;
    }
}

