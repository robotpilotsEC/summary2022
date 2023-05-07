#include "flash.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "motor.h"

flash_info_t flash_info = {
    .frame_header = 0xA5,
};

flash_cmd_t flash_cmd;

void Erase_Flash_SECTOR_11()//扇区11擦除
{
	uint32_t F;
	FLASH_EraseInitTypeDef FLash;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR);
	FLash.TypeErase = FLASH_TYPEERASE_SECTORS;
	FLash.Banks = FLASH_BANK_1;
	FLash.Sector = FLASH_SECTOR_11;
	FLash.NbSectors = 1;
	HAL_FLASHEx_Erase(&FLash, &F);
	HAL_FLASH_Lock();
}

void Flash_Init()//上电后读flash寻找可以写入的初始位置
{
	for(uint32_t i=0; i<128*1024/sizeof(flash_info_t); i++)
	{
		if(((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->frame_header == 0xFF)
		{
			flash_cmd.flash_idle_offset=i;
			break;
		}
	}
    motor[PITCH_M].res->position = ((flash_info_t*)(SECTOR_11_ADDRESS + (flash_cmd.flash_idle_offset-1)*sizeof(flash_info_t)))->pitch;
    motor[YAW_M].res->position = ((flash_info_t*)(SECTOR_11_ADDRESS + (flash_cmd.flash_idle_offset-1)*sizeof(flash_info_t)))->yaw;
    motor[GIMBAL].res->position = ((flash_info_t*)(SECTOR_11_ADDRESS + (flash_cmd.flash_idle_offset-1)*sizeof(flash_info_t)))->gimbal;
//	if(((flash_info_t*)(SECTOR_11_ADDRESS))->frame_header != 0xA5 && R_Flash_i==0)
//	{
//		Erase_Flash_SECTOR_11();
//	}
}

uint32_t data_temp[4];
void Flash_Write()  //保存结构体
{
    if(flash_cmd.flash_idle_offset >= 128*1024/sizeof(flash_info_t)-1)return;
    memcpy(data_temp, &flash_info, sizeof(flash_info_t));
    HAL_FLASH_Unlock();
    for(uint8_t i=0; i<4; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SECTOR_11_ADDRESS + flash_cmd.flash_idle_offset*sizeof(flash_info_t) + 4*i, data_temp[i]);
    }
	HAL_FLASH_Lock();
    flash_cmd.flash_idle_offset++;
}

void Flash_Read()//读取结构体
{
    if(flash_cmd.read_flag == 1)
    {
        flash_cmd.read_flag = 0;
        memcpy(&flash_info, (void *)(SECTOR_11_ADDRESS + flash_cmd.read_offset_num*sizeof(flash_info_t)), 16);
    }
}

void Flash_Erase()
{
    if(flash_cmd.erase_flag == 1)
    {
        flash_cmd.erase_flag = 0;
        Erase_Flash_SECTOR_11();
    }
}

void Flash_Empty_Check()
{
    if(flash_cmd.empty_check_flag == 1)
    {
        flash_cmd.empty_check_flag = 0;
        for(flash_cmd.empty_check_pin = 0; flash_cmd.empty_check_pin < 128 * 1024; flash_cmd.empty_check_pin++)
        {
            if(*((uint8_t *)(SECTOR_11_ADDRESS + flash_cmd.empty_check_pin)) != 0xFF)
            {
                break;
            }
        }
    }
}

void Flash_usart_send()
{
    uint8_t sendbuff[15] = {0};
    uint8_t receive_flag = 0;
    if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        HAL_UART_Receive(&huart3, &receive_flag, 1, 10);
    }
    if(flash_cmd.send_flag == 1 || receive_flag == '?')
    {
        flash_cmd.send_flag = 0;
        receive_flag = 0;
        for(uint32_t i=0; i<128*1024/sizeof(flash_info_t); i++)
        {
            if(((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->frame_header == 0xA5)
            {
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->launch_num);
                HAL_UART_Transmit(&huart3, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->target);
                HAL_UART_Transmit(&huart3, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->hit);
                HAL_UART_Transmit(&huart3, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->pitch);
                HAL_UART_Transmit(&huart3, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d ", ((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->yaw);
                HAL_UART_Transmit(&huart3, sendbuff, strlen((char *)sendbuff), 100);
                sprintf((char *)sendbuff, "%d\r\n", ((flash_info_t*)(SECTOR_11_ADDRESS + i*sizeof(flash_info_t)))->gimbal);
                HAL_UART_Transmit(&huart3, sendbuff, strlen((char *)sendbuff), 100);
            }
            else break;
        }
    }
}

